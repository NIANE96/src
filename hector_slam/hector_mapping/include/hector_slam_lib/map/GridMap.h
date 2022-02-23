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
 * @file GridMap.h
 * Fichier GridMap h
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
#ifndef __GridMap_h_
#define __GridMap_h_

#include "OccGridMapBase.h"
#include "GridMapLogOdds.h"
#include "GridMapReflectanceCount.h"
#include "GridMapSimpleCount.h"

namespace hectorslam {

typedef OccGridMapBase<LogOddsCell, GridMapLogOddsFunctions> GridMap;
//typedef OccGridMapBase<SimpleCountCell, GridMapSimpleCountFunctions> GridMap;
//typedef OccGridMapBase<ReflectanceCell, GridMapReflectanceFunctions> GridMap;

}

#endif
