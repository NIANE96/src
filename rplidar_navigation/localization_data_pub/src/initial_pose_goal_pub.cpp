/**
 *
 * PGE MASTER SME ROBOT MOBILE
 * Tous droits réservés.
 *
 * Copyright (c) 2014 - 2016 Shanghai Slamtec Co., Ltd.
 * http://www.slamtec.com
 * 
 * Système LIDAR ROBOT MOBILE
 *  * Site web : https://automaticaddison.com
 * Nœud ROS qui demande à l'utilisateur de saisir une pose initiale ou une pose cible.
 * Publier : Ce nœud publie vers les sujets suivants :   
 * goal_2d : Position et orientation du but (geometry_msgs::PoseStamped)
 *  
 * move_base_simple/goal : Position et orientation de l'objectif 
 * orientation (geometry_msgs::PoseStamped) initial_2d : La position et l'orientation initiales du robot en utilisant les * angles d'Euler. 
 * angles d'Euler. (geometry_msgs/PoseStamped)
 * initialpose : Position et orientation initiales du robot à l'aide de quaternions. 
 * quaternions. (geometry_msgs/PoseWithCovarianceStamped)
 * Modifié à partir du livre Practical Robotics in C++ (ISBN-10 : 9389423465)
 * par Lloyd Brombach
 * @file initial_pose_goal_pub.cpp
 * Fichier initial_pose_goal_pub cpp
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



 
// Inclure les messages
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf2/LinearMath/Quaternion.h>
#include <iostream>

using namespace std;

// Déclarer nos éditeurs
ros::Publisher pub;
ros::Publisher pub1;
ros::Publisher pub2;
ros::Publisher pub3;

// Publier une pose ou un objectif au format Euler ou quaternion.
void pub_msg(float x, float y, float yaw, int choice) {
  geometry_msgs::PoseWithCovarianceStamped pose;
  geometry_msgs::PoseStamped goal;
  geometry_msgs::Pose rpy;
  rpy.position.x = x;
  rpy.position.y = y;
  rpy.position.z = 0;
  rpy.orientation.x = 0;
  rpy.orientation.y = 0;
  rpy.orientation.z = yaw;
  rpy.orientation.w = 0;

 // Publiez une pose au format Euler (roulis, tangage, lacet). 
  // La pose est relative au cadre de coordonnées de la carte.
  if (choice == 1) { 
    pose.header.frame_id = "map";
    pose.header.stamp = ros::Time::now();
    pose.pose.pose = rpy;
    pub2.publish(pose);
  }
  // Publier un objectif
  else {
    cout << "publishing goal" << endl;

    goal.header.frame_id = "map";
    goal.header.stamp = ros::Time::now();
    goal.pose = rpy;
    pub.publish(goal);
  }

  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  rpy.orientation.x = q.x();
  rpy.orientation.y = q.y();
  rpy.orientation.z = q.z();
  rpy.orientation.w = q.w();

  // Publier une pose au format quaternion.
  if(choice == 1) { 
    
    pose.pose.pose = rpy;
    pub3.publish(pose);
  }
  //  Publier un objectif
  else {     
    goal.pose = rpy;
    pub1.publish(goal);
  }
}

int main(int argc, char **argv) {
  
  // / Se connecter à ROS
  ros::init(argc, argv, "initial_pose_goal_pub");
  ros::NodeHandle node;
	
  // Configurer nos éditeurs ROS
  pub = node.advertise<geometry_msgs::PoseStamped>("goal_2d", 10);
  pub1 = node.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 10);
  pub2 = node.advertise<geometry_msgs::PoseStamped>("initial_2d", 10);
  pub3 = node.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 10);

   // Continuer à tourner en boucle tant que le ROS fonctionne.
  while (ros::ok()) {
		
    // Initialiser les variables de pose
    float x = -1;
    float y = -1;
    float yaw = -11;
    int choice  = -1;

    while(choice < 1 || choice > 2
         || x < 0 || y < 0 || yaw < -3.141592 || yaw > 3.141592) {
      
      cout << "\nEnter positive float value for x : " << endl;
      cin >> x;
      cout << "\nEnter positive float value for y : " << endl;
      cin >> y;
      cout << "\nEnter float value for yaw in radians from -PI to +PI (e.g. 90 deg = 1.57 rad): " << endl;
      cin >> yaw;
      cout << "\nEnter 1 if this is a pose, 2 if it is a goal" << endl;
      cin >> choice;
      if (cin.fail()) {
        cin.clear();
        cin.ignore(50, '\n');
        choice = -1;
      }
    }
    
    // Remplissez les champs de pose ou d'objectif et publiez.
    pub_msg(x, y, yaw, choice);

  }
  return 0;
}
