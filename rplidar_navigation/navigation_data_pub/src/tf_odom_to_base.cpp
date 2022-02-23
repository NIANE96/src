/*
 * Site web : https://automaticaddison.com
 * Nœud ROS qui s'abonne à la pose du robot et qui publie 
 * une transformation odom to base_footprint basée sur les données.
 * S'abonner :  
 * robot_pose_ekf/odom_combined : Position actuelle et estimation de la vitesse. 
 * L'orientation est un quaternion 
 * (geometry_msgs/PoseWithCovarianceStamped).
 *
 * Publier : Ce nœud va publier vers les sujets suivants :
 * odom to base_footprint : La pose du cadre de coordonnées base_footprint
 * à l'intérieur du cadre de coordonnées odom (tf/tfMessage)
 * Modifié à partir du livre Practical Robotics in C++ (ISBN-10 : 9389423465)
 * par Lloyd Brombach
 */
 
 // Include the relevant libraries
#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf/transform_broadcaster.h>
#include "std_msgs/Float32.h"

using namespace std;

// Callback message that broadcasts the robot pose information as a transform
void pose_callback(const geometry_msgs::PoseWithCovarianceStamped &currentOdom) {

  // Create a TransformBroadcaster object that will publish transforms over ROS
  static tf::TransformBroadcaster br;
  
  // Copy the information from the robot's current 2D pose into the 3D transform
  tf::Transform odom_base_tf;	
  odom_base_tf.setOrigin( tf::Vector3(currentOdom.pose.pose.position.x, currentOdom.pose.pose.position.y, 0.0) );
  tf::Quaternion tf_q(currentOdom.pose.pose.orientation.x, currentOdom.pose.pose.orientation.y,
                        currentOdom.pose.pose.orientation.z, currentOdom.pose.pose.orientation.w);

  // Set the rotation
  odom_base_tf.setRotation(tf_q);

  br.sendTransform(tf::StampedTransform(odom_base_tf, currentOdom.header.stamp, "odom", "base_footprint"));
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "tf_odom_to_base");
  ros::NodeHandle node;

  ros::Subscriber subOdom = node.subscribe("robot_pose_ekf/odom_combined", 10, pose_callback);

  ros::Rate loop_rate(10);
  while (ros::ok()) {
		
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
