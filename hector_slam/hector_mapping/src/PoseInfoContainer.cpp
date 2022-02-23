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
 * @file PoseInfoContainer.cpp
 * Fichier PoseInfoContainer cpp
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
#include "PoseInfoContainer.h"

void PoseInfoContainer::update(const Eigen::Vector3f& slamPose, const Eigen::Matrix3f& slamCov, const ros::Time& stamp, const std::string& frame_id)
{
  /** @brief Fill stampedPose */
  std_msgs::Header& header = stampedPose_.header;
  header.stamp = stamp;
  header.frame_id = frame_id;

  geometry_msgs::Pose& pose = stampedPose_.pose;
  pose.position.x = slamPose.x();
  pose.position.y = slamPose.y();

  pose.orientation.w = cos(slamPose.z()*0.5f);
  pose.orientation.z = sin(slamPose.z()*0.5f);

   /** @brief Fill covPose */
  //geometry_msgs::PoseWithCovarianceStamped covPose;
  covPose_.header = header;
  covPose_.pose.pose = pose;

  boost::array<double, 36>& cov(covPose_.pose.covariance);

  cov[0] = static_cast<double>(slamCov(0,0));
  cov[7] = static_cast<double>(slamCov(1,1));
  cov[35] = static_cast<double>(slamCov(2,2));

  double xyC = static_cast<double>(slamCov(0,1));
  cov[1] = xyC;
  cov[6] = xyC;

  double xaC = static_cast<double>(slamCov(0,2));
  cov[5] = xaC;
  cov[30] = xaC;

  double yaC = static_cast<double>(slamCov(1,2));
  cov[11] = yaC;
  cov[31] = yaC;

   /** @brief Fill tf tansform*/
  tf::poseMsgToTF(pose, poseTransform_);
}
