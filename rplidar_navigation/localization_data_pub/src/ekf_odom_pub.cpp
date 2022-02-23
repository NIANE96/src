/**
 *
 * PGE MASTER SME ROBOT MOBILE
 * Tous droits réservés.
 * date 2022
 * version ros 1 kinetic
 *
 * Copyright (c) 2014 - 2016 Shanghai Slamtec Co., Ltd.
 * http://www.slamtec.com
 * 
 * Système LIDAR ROBOT MOBILE
 * Publie des informations d'odométrie à utiliser avec le paquet robot_pose_ekf.
 * Ces informations d'odométrie sont basées sur le nombre de tics de l'encodeur de roue.
 * Subscribe : Nœud ROS qui s'abonne aux sujets suivants :
 * right_ticks : Comptes de tics de l'encodeur du moteur droit (std_msgs/Int16)
 * 
 * left_ticks : nombre de tics provenant de l'encodeur du moteur gauche (std_msgs/Int16).
 * 
 * initial_2d : La position et l'orientation initiales du robot.
 * (geometry_msgs/PoseStamped)
 *
 * Publier : Ce nœud publiera vers les sujets suivants :
 * odom_data_euler : Estimation de la position et de la vitesse. La variable orientation.z 
 * est un angle d'Euler représentant l'angle de lacet.
 * (nav_msgs/Odometry)
 * odom_data_quat : Estimation de la position et de la vitesse. L'orientation est 
 * au format quaternion.
 * (nav_msgs/Odometry)
 * Modifié à partir du livre Practical Robotics in C++ (ISBN-10 : 9389423465)
 * par loyd Brombach
 * 
 * @file hector_trajectory_server.cpp
 * Fichier hector_trajectory_server cpp
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

// Include various libraries
#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <cmath>

/** @brief Create odometry data publishers*/
ros::Publisher odom_data_pub;
ros::Publisher odom_data_pub_quat;
nav_msgs::Odometry odomNew;
nav_msgs::Odometry odomOld;

/** @brief Initial pose*/
const double initialX = 0.0;
const double initialY = 0.0;
const double initialTheta = 0.00000000001;
const double PI = 3.141592;

/** @brief Constantes physiques du robot*/
const double TICKS_PER_REVOLUTION = 620; // À titre de référence.
const double WHEEL_RADIUS = 0.033; //  Rayon de la roue en mètres
const double WHEEL_BASE = 0.17; // Du centre du pneu gauche au centre du pneu droit
const double TICKS_PER_METER = 3100; // L'original était de 2800

/** @brief Distance parcourue par les deux roues*/
double distanceLeft = 0;
double distanceRight = 0;

/** @brief Drapeau pour voir si la pose initiale a été reçue*/
bool initialPoseRecieved = false;

using namespace std;

/** @brief Obtenir le message initial_2d à partir de clics Rviz ou d'un éditeur de pose manuel
* @fn void set_initial_2d(const geometry_msgs::PoseStamped &rvizClick) */
void set_initial_2d(const geometry_msgs::PoseStamped &rvizClick) {

  odomOld.pose.pose.position.x = rvizClick.pose.position.x;
  odomOld.pose.pose.position.y = rvizClick.pose.position.y;
  odomOld.pose.pose.orientation.z = rvizClick.pose.orientation.z;
  initialPoseRecieved = true;
}

/** @brief Calculer la distance parcourue par la roue gauche depuis le dernier cycle.
* @fn void Calc_Left(const std_msgs::Int16& leftCount) */
void Calc_Left(const std_msgs::Int16& leftCount) {

  static int lastCountL = 0;
  if(leftCount.data != 0 && lastCountL != 0) {
		
    int leftTicks = (leftCount.data - lastCountL);

    if (leftTicks > 10000) {
      leftTicks = 0 - (65535 - leftTicks);
    }
    else if (leftTicks < -10000) {
      leftTicks = 65535-leftTicks;
    }
    else{}
    distanceLeft = leftTicks/TICKS_PER_METER;
  }
  lastCountL = leftCount.data;
}

/** @brief Calculer la distance parcourue par la roue droite depuis le dernier cycle.
* @fn void Calc_Right(const std_msgs::Int16& rightCount) */
void Calc_Right(const std_msgs::Int16& rightCount) {
  
  static int lastCountR = 0;
  if(rightCount.data != 0 && lastCountR != 0) {

    int rightTicks = rightCount.data - lastCountR;
    
    if (distanceRight > 10000) {
      distanceRight = (0 - (65535 - distanceRight))/TICKS_PER_METER;
    }
    else if (rightTicks < -10000) {
      rightTicks = 65535 - rightTicks;
    }
    else{}
    distanceRight = rightTicks/TICKS_PER_METER;
  }
  lastCountR = rightCount.data;
}

// Publish a nav_msgs::Odometry message in quaternion format
void publish_quat() {

  tf2::Quaternion q;
		
  q.setRPY(0, 0, odomNew.pose.pose.orientation.z);

  nav_msgs::Odometry quatOdom;
  quatOdom.header.stamp = odomNew.header.stamp;
  quatOdom.header.frame_id = "odom";
  quatOdom.child_frame_id = "base_link";
  quatOdom.pose.pose.position.x = odomNew.pose.pose.position.x;
  quatOdom.pose.pose.position.y = odomNew.pose.pose.position.y;
  quatOdom.pose.pose.position.z = odomNew.pose.pose.position.z;
  quatOdom.pose.pose.orientation.x = q.x();
  quatOdom.pose.pose.orientation.y = q.y();
  quatOdom.pose.pose.orientation.z = q.z();
  quatOdom.pose.pose.orientation.w = q.w();
  quatOdom.twist.twist.linear.x = odomNew.twist.twist.linear.x;
  quatOdom.twist.twist.linear.y = odomNew.twist.twist.linear.y;
  quatOdom.twist.twist.linear.z = odomNew.twist.twist.linear.z;
  quatOdom.twist.twist.angular.x = odomNew.twist.twist.angular.x;
  quatOdom.twist.twist.angular.y = odomNew.twist.twist.angular.y;
  quatOdom.twist.twist.angular.z = odomNew.twist.twist.angular.z;

  for(int i = 0; i<36; i++) {
    if(i == 0 || i == 7 || i == 14) {
      quatOdom.pose.covariance[i] = .01;
     }
     else if (i == 21 || i == 28 || i== 35) {
       quatOdom.pose.covariance[i] += 0.1;
     }
     else {
       quatOdom.pose.covariance[i] = 0;
     }
  }

  odom_data_pub_quat.publish(quatOdom);
}

/** @brief Mise à jour des informations d'odométrie
* @fn void update_odom() */
void update_odom() {

  /** @brief Calculer la distance moyenne*/
  double cycleDistance = (distanceRight + distanceLeft) / 2;
  
  /** @brief Calcule le nombre de radians que le robot a tourné depuis le dernier cycle*/.
  double cycleAngle = asin((distanceRight-distanceLeft)/WHEEL_BASE);

  /** @brief Angle moyen pendant le dernier cycle*/
  double avgAngle = cycleAngle/2 + odomOld.pose.pose.orientation.z;
	
  if (avgAngle > PI) {
    avgAngle -= 2*PI;
  }
  else if (avgAngle < -PI) {
    avgAngle += 2*PI;
  }
  else{}

  /** @brief Calculer la nouvelle pose (x, y, et thêta)*/
  odomNew.pose.pose.position.x = odomOld.pose.pose.position.x + cos(avgAngle)*cycleDistance;
  odomNew.pose.pose.position.y = odomOld.pose.pose.position.y + sin(avgAngle)*cycleDistance;
  odomNew.pose.pose.orientation.z = cycleAngle + odomOld.pose.pose.orientation.z;

 /** @brief Empêcher le verrouillage d'un seul mauvais cycle*/
  if (isnan(odomNew.pose.pose.position.x) || isnan(odomNew.pose.pose.position.y)
     || isnan(odomNew.pose.pose.position.z)) {
    odomNew.pose.pose.position.x = odomOld.pose.pose.position.x;
    odomNew.pose.pose.position.y = odomOld.pose.pose.position.y;
    odomNew.pose.pose.orientation.z = odomOld.pose.pose.orientation.z;
  }

  /** @brief S'assurer que le thêta reste dans la bonne fourchette*/.
  if (odomNew.pose.pose.orientation.z > PI) {
    odomNew.pose.pose.orientation.z -= 2 * PI;
  }
  else if (odomNew.pose.pose.orientation.z < -PI) {
    odomNew.pose.pose.orientation.z += 2 * PI;
  }
  else{}

  /** @brief Calcul de la vélocité*/
  odomNew.header.stamp = ros::Time::now();
  odomNew.twist.twist.linear.x = cycleDistance/(odomNew.header.stamp.toSec() - odomOld.header.stamp.toSec());
  odomNew.twist.twist.angular.z = cycleAngle/(odomNew.header.stamp.toSec() - odomOld.header.stamp.toSec());

   /** @brief Sauvegarde des données de pose pour le prochain cycle*/
  odomOld.pose.pose.position.x = odomNew.pose.pose.position.x;
  odomOld.pose.pose.position.y = odomNew.pose.pose.position.y;
  odomOld.pose.pose.orientation.z = odomNew.pose.pose.orientation.z;
  odomOld.header.stamp = odomNew.header.stamp;

/** @brief Publier le message d'odométrie*/
  odom_data_pub.publish(odomNew);
}

int main(int argc, char **argv) {
  
    /** @brief Définir les champs de données du message d'odométrie*/
  odomNew.header.frame_id = "odom";
  odomNew.pose.pose.position.z = 0;
  odomNew.pose.pose.orientation.x = 0;
  odomNew.pose.pose.orientation.y = 0;
  odomNew.twist.twist.linear.x = 0;
  odomNew.twist.twist.linear.y = 0;
  odomNew.twist.twist.linear.z = 0;
  odomNew.twist.twist.angular.x = 0;
  odomNew.twist.twist.angular.y = 0;
  odomNew.twist.twist.angular.z = 0;
  odomOld.pose.pose.position.x = initialX;
  odomOld.pose.pose.position.y = initialY;
  odomOld.pose.pose.orientation.z = initialTheta;


  /** @brief Lancer ROS et créer un nœud*/
  ros::init(argc, argv, "ekf_odom_pub");
  ros::NodeHandle node;

    /** @brief S'abonner à des sujets ROS*/
  ros::Subscriber subForRightCounts = node.subscribe("right_ticks", 100, Calc_Right, ros::TransportHints().tcpNoDelay());
  ros::Subscriber subForLeftCounts = node.subscribe("left_ticks", 100, Calc_Left, ros::TransportHints().tcpNoDelay());
  ros::Subscriber subInitialPose = node.subscribe("initial_2d", 1, set_initial_2d);

/** @brief Publisher of simple odom message ou orientation.z es un angle euler*/
  odom_data_pub = node.advertise<nav_msgs::Odometry>("odom_data_euler", 100);

   /** @brief Editeur du message odom complet où l'orientation est quaternion*/
  odom_data_pub_quat = node.advertise<nav_msgs::Odometry>("odom_data_quat", 100);

  ros::Rate loop_rate(30); 
	
  while(ros::ok()) {
    
    if(initialPoseRecieved) {
      update_odom();
      publish_quat();
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
