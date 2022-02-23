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
 * @file imu_altitude_to_tf_node.cpp
 * Fichier imu_altitude_to_tf_node cpp
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
#include "sensor_msgs/Imu.h"

std::string p_base_stabilized_frame_;
std::string p_base_frame_;
tf::TransformBroadcaster* tfB_;
tf::StampedTransform transform_;
tf::Quaternion tmp_;

#ifndef TF_MATRIX3x3_H
  typedef btScalar tfScalar;
  namespace tf { typedef btMatrix3x3 Matrix3x3; }
#endif

void imuMsgCallback(const sensor_msgs::Imu& imu_msg)
{
  tf::quaternionMsgToTF(imu_msg.orientation, tmp_);

  tfScalar yaw, pitch, roll;
  tf::Matrix3x3(tmp_).getRPY(roll, pitch, yaw);

  tmp_.setRPY(roll, pitch, 0.0);

  transform_.setRotation(tmp_);

  transform_.stamp_ = imu_msg.header.stamp;

  tfB_->sendTransform(transform_);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, ROS_PACKAGE_NAME);

  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  pn.param("base_stabilized_frame", p_base_stabilized_frame_, std::string("base_stabilized"));
  pn.param("base_frame", p_base_frame_, std::string("base_link"));
  
  tfB_ = new tf::TransformBroadcaster();
  transform_.getOrigin().setX(0.0);
  transform_.getOrigin().setY(0.0);
  transform_.getOrigin().setZ(0.0);
  transform_.frame_id_ = p_base_stabilized_frame_;
  transform_.child_frame_id_ = p_base_frame_;

  ros::Subscriber imu_subscriber = n.subscribe("imu_topic", 10, imuMsgCallback);

  ros::spin();

  delete tfB_;

  return 0;
}
