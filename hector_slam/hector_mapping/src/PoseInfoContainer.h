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
 * @file PoseInfoContainer.h
 * Fichier PoseInfoContainer h
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
#ifndef POSE_INFO_CONTAINER_H__
#define POSE_INFO_CONTAINER_H__

#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <Eigen/Core>

class PoseInfoContainer{
public:

  void update(const Eigen::Vector3f& slamPose, const Eigen::Matrix3f& slamCov, const ros::Time& stamp, const std::string& frame_id);

  const geometry_msgs::PoseStamped& getPoseStamped() { return stampedPose_; };
  const geometry_msgs::PoseWithCovarianceStamped& getPoseWithCovarianceStamped() { return covPose_; };
  const tf::Transform& getTfTransform() { return poseTransform_; };

protected:
  geometry_msgs::PoseStamped stampedPose_;
  geometry_msgs::PoseWithCovarianceStamped covPose_;
  tf::Transform poseTransform_;

};

#endif

