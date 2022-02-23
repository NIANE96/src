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
 * @file map_writer_interface.h
 * Fichier map_writer_interface h
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
#ifndef _MAPWRITERPLUGININTERFACE_H__
#define _MAPWRITERPLUGININTERFACE_H__

#include "map_writer_interface.h"

namespace hector_geotiff{

class MapWriterPluginInterface{

public:

  virtual void initialize(const std::string& name) = 0;
  virtual void draw(MapWriterInterface* map_writer_interface) = 0;

};

} //namespace hector_geotiff

#endif
