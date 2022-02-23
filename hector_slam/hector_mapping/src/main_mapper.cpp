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
 * @file main_mapper.cpp
 * Fichier main_mapper cpp
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
#include <ros/ros.h>

#include "HectorMapperRos.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hector_slam");

  HectorMapperRos sm;

  ros::spin();

  return(0);
}

