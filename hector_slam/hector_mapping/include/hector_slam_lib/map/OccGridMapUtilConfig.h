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
 * @file OccGridMapUtilConfi.h
 * Fichier OccGridMapUtilConfi h
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
#ifndef __OccGridMapUtilConfig_h_
#define __OccGridMapUtilConfig_h_

#include "OccGridMapUtil.h"

//#define SLAM_USE_HASH_CACHING
#ifdef SLAM_USE_HASH_CACHING
#include "GridMapCacheHash.h"
typedef GridMapCacheHash GridMapCacheMethod;
#else
#include "GridMapCacheArray.h"
typedef GridMapCacheArray GridMapCacheMethod;
#endif

namespace hectorslam {

template<typename ConcreteOccGridMap>
class OccGridMapUtilConfig
  : public OccGridMapUtil<ConcreteOccGridMap, GridMapCacheMethod>
{
public:

  OccGridMapUtilConfig(ConcreteOccGridMap* gridMap = 0)
    : OccGridMapUtil<ConcreteOccGridMap, GridMapCacheMethod>(gridMap)
  {}
};

}

#endif
