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
 * @file HectorDebugInterface.h
 * Fichier HectorDebugInterface h
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
#ifndef hectordebuginfointerface_h__
#define hectordebuginfointerface_h__

class HectorDebugInfoInterface{
public:

  virtual void sendAndResetData() = 0;
  virtual void addHessianMatrix(const Eigen::Matrix3f& hessian) = 0;
  virtual void addPoseLikelihood(float lh) = 0;
};

#endif
