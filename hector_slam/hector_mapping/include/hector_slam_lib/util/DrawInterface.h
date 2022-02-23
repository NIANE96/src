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
 * @file DrawInterface.h
 * Fichier DrawInterface h
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
#ifndef drawinterface_h__
#define drawinterface_h__

class DrawInterface{
public:
  virtual void drawPoint(const Eigen::Vector2f& pointWorldFrame) = 0;
  virtual void drawArrow(const Eigen::Vector3f& poseWorld) = 0;
  virtual void drawCovariance(const Eigen::Vector2f& mean, const Eigen::Matrix2f& cov) = 0;

  virtual void setScale(double scale) = 0;
  virtual void setColor(double r, double g, double b, double a = 1.0) = 0;

  virtual void sendAndResetData() = 0;
};

#endif
