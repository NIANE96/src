/**
 *
 * PGE MASTER SME ROBOT MOBILE
 * Tous droits réservés.
 *
 * Copyright (c) 2014 - 2016 Shanghai Slamtec Co., Ltd.
 * http://www.slamtec.com
 * 
 * Système LIDAR ROBOT MOBILE
 * 
 * @file map_to_image_node.cpp
 * Fichier map_to_image_node cpp
 * @author NIANE
 * @author DIOUME
 * @author HOURI
 * @author BOUBACAR
 * @author DOUKI
 * @author CAMARA
 * @date 2022
 * @version 1.0 
 * 
 * 
 */
#include "ros/ros.h"

#include <nav_msgs/GetMap.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <Eigen/Geometry>

#include <hector_map_tools/HectorMapTools.h>

using namespace std;

/**
 * @brief Ce nœud fournit des cartes de grille d'occupation sous forme d'images via image_transport, de sorte que la transmission consomme moins de bande passante.
 * Le code fourni est une preuve de concept incomplète.
 */
class MapAsImageProvider
{
public:
  MapAsImageProvider()
    : pn_("~")
  {

    image_transport_ = new image_transport::ImageTransport(n_);
    image_transport_publisher_full_ = image_transport_->advertise("map_image/full", 1);
    image_transport_publisher_tile_ = image_transport_->advertise("map_image/tile", 1);

    pose_sub_ = n_.subscribe("pose", 1, &MapAsImageProvider::poseCallback, this);
    map_sub_ = n_.subscribe("map", 1, &MapAsImageProvider::mapCallback, this);

     /** @brief Quel frame_id a du sens ?*/
    cv_img_full_.header.frame_id = "map_image";
    cv_img_full_.encoding = sensor_msgs::image_encodings::MONO8;

    cv_img_tile_.header.frame_id = "map_image";
    cv_img_tile_.encoding = sensor_msgs::image_encodings::MONO8;

     /** @brief Largeur de cellule fixe pour les images basées sur des tiles, utilisez dynamic_reconfigure pour cela plus tard*/
    p_size_tiled_map_image_x_ = 64;
    p_size_tiled_map_image_y_ = 64;

    ROS_INFO("Map to Image node started.");
  }

  ~MapAsImageProvider()
  {
    delete image_transport_;
  }

   /** @brief Nous supposons que la position du robot est disponible sous la forme d'un PoseStamped ici (interroger tf serait l'option la plus générale) */
  void poseCallback(const geometry_msgs::PoseStampedConstPtr& pose)
  {
    pose_ptr_ = pose;
  }

   /** @brief La conversion carte->image s'exécute à chaque fois qu'une nouvelle carte est reçue en ce moment*/.
  void mapCallback(const nav_msgs::OccupancyGridConstPtr& map)
  {
    int size_x = map->info.width;
    int size_y = map->info.height;

    if ((size_x < 3) || (size_y < 3) ){
      ROS_INFO("Map size is only x: %d,  y: %d . Not running map to image conversion", size_x, size_y);
      return;
    }

     /** @brief Seulement si quelqu'un y est abonné, faites le travail et publiez l'image complète de la carte*/.
    if (image_transport_publisher_full_.getNumSubscribers() > 0){
      cv::Mat* map_mat  = &cv_img_full_.image;

       /** @brief redimensionner l'image cv si elle n'a pas les mêmes dimensions que la carte*/
      if ( (map_mat->rows != size_y) && (map_mat->cols != size_x)){
        *map_mat = cv::Mat(size_y, size_x, CV_8U);
      }

      const std::vector<int8_t>& map_data (map->data);

      unsigned char *map_mat_data_p=(unsigned char*) map_mat->data;

       /** @brief Nous devons inverser l'axe y, y pour l'image commence en haut et y pour la carte en bas.*/
      int size_y_rev = size_y-1;

      for (int y = size_y_rev; y >= 0; --y){

        int idx_map_y = size_x * (size_y_rev - y);
        int idx_img_y = size_x * y;

        for (int x = 0; x < size_x; ++x){

          int idx = idx_img_y + x;

          switch (map_data[idx_map_y + x])
          {
          case -1:
            map_mat_data_p[idx] = 127;
            break;

          case 0:
            map_mat_data_p[idx] = 255;
            break;

          case 100:
            map_mat_data_p[idx] = 0;
            break;
          }
        }
      }
      image_transport_publisher_full_.publish(cv_img_full_.toImageMsg());
    }

     /** @brief Seulement si quelqu'un y est abonné, faites le travail et publiez l'image de la carte basée sur les tuiles Vérifiez aussi si pose_ptr_ est valide*/
    if ((image_transport_publisher_tile_.getNumSubscribers() > 0) && (pose_ptr_)){

      world_map_transformer_.setTransforms(*map);

      Eigen::Vector2f rob_position_world (pose_ptr_->pose.position.x, pose_ptr_->pose.position.y);
      Eigen::Vector2f rob_position_map (world_map_transformer_.getC2Coords(rob_position_world));

      Eigen::Vector2i rob_position_mapi (rob_position_map.cast<int>());

      Eigen::Vector2i tile_size_lower_halfi (p_size_tiled_map_image_x_ / 2, p_size_tiled_map_image_y_ / 2);

      Eigen::Vector2i min_coords_map (rob_position_mapi - tile_size_lower_halfi);

       /** @brief Pince pour les coordonnées inférieures de la carte*/
      if (min_coords_map[0] < 0){
        min_coords_map[0] = 0;
      }

      if (min_coords_map[1] < 0){
        min_coords_map[1] = 0;
      }

      Eigen::Vector2i max_coords_map (min_coords_map + Eigen::Vector2i(p_size_tiled_map_image_x_,p_size_tiled_map_image_y_));

       /** @brief Fixation des coordonnées supérieures de la carte*/
      if (max_coords_map[0] > size_x){

        int diff = max_coords_map[0] - size_x;
        min_coords_map[0] -= diff;

        max_coords_map[0] = size_x;
      }

      if (max_coords_map[1] > size_y){

        int diff = max_coords_map[1] - size_y;
        min_coords_map[1] -= diff;

        max_coords_map[1] = size_y;
      }

       /** @brief Redescendre la pince (dans le cas où la carte est plus petite que la fenêtre de visualisation sélectionnée)*/
      if (min_coords_map[0] < 0){
        min_coords_map[0] = 0;
      }

      if (min_coords_map[1] < 0){
        min_coords_map[1] = 0;
      }

      Eigen::Vector2i actual_map_dimensions(max_coords_map - min_coords_map);

      cv::Mat* map_mat  = &cv_img_tile_.image;

       /** @brief redimensionner l'image cv si elle n'a pas les mêmes dimensions que la fenêtre de visualisation sélectionnée*/
      if ( (map_mat->rows != actual_map_dimensions[0]) || (map_mat->cols != actual_map_dimensions[1])){
        *map_mat = cv::Mat(actual_map_dimensions[0], actual_map_dimensions[1], CV_8U);
      }

      const std::vector<int8_t>& map_data (map->data);

      unsigned char *map_mat_data_p=(unsigned char*) map_mat->data;

      /** @brief Nous devons inverser l'axe y, y pour l'image commence en haut et y pour la carte en bas*/
      int y_img = max_coords_map[1]-1;

      for (int y = min_coords_map[1]; y < max_coords_map[1];++y){

        int idx_map_y = y_img-- * size_x;
        int idx_img_y = (y-min_coords_map[1]) * actual_map_dimensions.x();

        for (int x = min_coords_map[0]; x < max_coords_map[0];++x){

          int img_index = idx_img_y + (x-min_coords_map[0]);

          switch (map_data[idx_map_y+x])
          {
          case 0:
            map_mat_data_p[img_index] = 255;
            break;

          case -1:
            map_mat_data_p[img_index] = 127;
            break;

          case 100:
            map_mat_data_p[img_index] = 0;
            break;
          }
        }        
      }
      image_transport_publisher_tile_.publish(cv_img_tile_.toImageMsg());
    }
  }

  ros::Subscriber map_sub_;
  ros::Subscriber pose_sub_;

  image_transport::Publisher image_transport_publisher_full_;
  image_transport::Publisher image_transport_publisher_tile_;

  image_transport::ImageTransport* image_transport_;

  geometry_msgs::PoseStampedConstPtr pose_ptr_;

  cv_bridge::CvImage cv_img_full_;
  cv_bridge::CvImage cv_img_tile_;

  ros::NodeHandle n_;
  ros::NodeHandle pn_;

  int p_size_tiled_map_image_x_;
  int p_size_tiled_map_image_y_;

  HectorMapTools::CoordinateTransformer<float> world_map_transformer_;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_to_image_node");

  MapAsImageProvider map_image_provider;

  ros::spin();

  return 0;
}
