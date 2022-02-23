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
 * @file pose_and_orientation_to_imu_node.cpp
 * Fichier pose_and_orientation_to_imu_node cpp
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
#include "tf/transform_broadcaster.h"
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

std::string p_map_frame_;
std::string p_base_footprint_frame_;
std::string p_base_stabilized_frame_;
std::string p_base_frame_;
tf::TransformBroadcaster* tfB_;
tf::StampedTransform transform_;

tf::Quaternion robot_pose_quaternion_;
tf::Point robot_pose_position_;
tf::Transform robot_pose_transform_;

tf::Quaternion tmp_;
tf::Quaternion orientation_quaternion_;

sensor_msgs::ImuConstPtr last_imu_msg_;
sensor_msgs::Imu fused_imu_msg_;
nav_msgs::Odometry odom_msg_;
geometry_msgs::PoseStampedConstPtr last_pose_msg_;

ros::Publisher fused_imu_publisher_;
ros::Publisher odometry_publisher_;

size_t callback_count_;

#ifndef TF_MATRIX3x3_H
typedef btScalar tfScalar;
namespace tf { typedef btMatrix3x3 Matrix3x3; }
#endif

void imuMsgCallback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
  callback_count_++;

  tf::quaternionMsgToTF(imu_msg->orientation, tmp_);

  tfScalar imu_yaw, imu_pitch, imu_roll;
  tf::Matrix3x3(tmp_).getRPY(imu_roll, imu_pitch, imu_yaw);

  tf::Transform transform;
  transform.setIdentity();
  tf::Quaternion quat;

  quat.setRPY(imu_roll, imu_pitch, 0.0);

  if (true){
    transform.setRotation(quat);
    tfB_->sendTransform(tf::StampedTransform(transform, imu_msg->header.stamp, p_base_stabilized_frame_, p_base_frame_));
  }

  tfScalar pose_yaw, pose_pitch, pose_roll;

  if (last_pose_msg_ != 0){
    tf::quaternionMsgToTF(last_pose_msg_->pose.orientation, tmp_);

    tf::Matrix3x3(tmp_).getRPY(pose_roll, pose_pitch, pose_yaw);
  }else{
    pose_yaw = 0.0;
  }

  orientation_quaternion_.setRPY(imu_roll, imu_pitch, pose_yaw);

  fused_imu_msg_.header.stamp = imu_msg->header.stamp;

  fused_imu_msg_.orientation.x = orientation_quaternion_.getX();
  fused_imu_msg_.orientation.y = orientation_quaternion_.getY();
  fused_imu_msg_.orientation.z = orientation_quaternion_.getZ();
  fused_imu_msg_.orientation.w = orientation_quaternion_.getW();

  fused_imu_publisher_.publish(fused_imu_msg_);

  //If no pose message received, yaw is set to 0.
  //@TODO: Check for timestamp of pose and disable sending if too old
  if (last_pose_msg_ != 0){
    if ( (callback_count_ % 5) == 0){
      odom_msg_.header.stamp = imu_msg->header.stamp;
      odom_msg_.pose.pose.orientation = fused_imu_msg_.orientation;
      odom_msg_.pose.pose.position = last_pose_msg_->pose.position;

      odometry_publisher_.publish(odom_msg_);
    }
  }

}

void poseMsgCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
{

  std::vector<tf::StampedTransform> transforms;
  transforms.resize(2);

  tf::quaternionMsgToTF(pose_msg->pose.orientation, robot_pose_quaternion_);
  tf::pointMsgToTF(pose_msg->pose.position, robot_pose_position_);

  robot_pose_transform_.setRotation(robot_pose_quaternion_);
  robot_pose_transform_.setOrigin(robot_pose_position_);

  tf::Transform height_transform;
  height_transform.setIdentity();
  height_transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));

  transforms[0] = tf::StampedTransform(robot_pose_transform_, pose_msg->header.stamp, p_map_frame_, p_base_footprint_frame_);
  transforms[1] = tf::StampedTransform(height_transform, pose_msg->header.stamp, p_base_footprint_frame_, p_base_stabilized_frame_);

  tfB_->sendTransform(transforms);

  // Perform simple estimation of vehicle altitude based on orientation
  if (last_pose_msg_){
    tf::Vector3 plane_normal = tf::Matrix3x3(orientation_quaternion_) * tf::Vector3(0.0, 0.0, 1.0);

    tf::Vector3 last_position;
    tf::pointMsgToTF(last_pose_msg_->pose.position, last_position);

    double height_difference =
        (-plane_normal.getX() * (robot_pose_position_.getX() - last_position.getX())
         -plane_normal.getY() * (robot_pose_position_.getY() - last_position.getY())
         +plane_normal.getZ() * last_position.getZ()) / last_position.getZ();

  }



  last_pose_msg_ = pose_msg;

}

int main(int argc, char **argv) {
  ros::init(argc, argv, ROS_PACKAGE_NAME);

  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  pn.param("map_frame", p_map_frame_, std::string("map"));
  pn.param("base_footprint_frame", p_base_footprint_frame_, std::string("base_footprint"));
  pn.param("base_stabilized_frame", p_base_stabilized_frame_, std::string("base_stabilized"));
  pn.param("base_frame", p_base_frame_, std::string("base_link"));

  fused_imu_msg_.header.frame_id = p_base_stabilized_frame_;
  odom_msg_.header.frame_id = "map";

  tfB_ = new tf::TransformBroadcaster();

  fused_imu_publisher_ = n.advertise<sensor_msgs::Imu>("/fused_imu",1,false);
  odometry_publisher_ = n.advertise<nav_msgs::Odometry>("/state", 1, false);

  ros::Subscriber imu_subscriber = n.subscribe("/imu", 10, imuMsgCallback);
  ros::Subscriber pose_subscriber = n.subscribe("/pose", 10, poseMsgCallback);

  callback_count_ = 0;

  ros::spin();

  delete tfB_;

  return 0;
}
