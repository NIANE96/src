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
 * @file GridMapBase.h
 * Fichier GridMapBase h
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
#ifndef __GridMapBase_h_
#define __GridMapBase_h_

#include <Eigen/Geometry>
#include <Eigen/LU>

#include "MapDimensionProperties.h"

namespace hectorslam {

/**
 * @brief GridMapBase fournit la fonctionnalité de base des cartes à grille (crée la grille, fournit la transformation des coordonnées mondiales).
 * @brief Elle sert de classe de base pour différentes représentations cartographiques qui peuvent étendre ses fonctionnalités.
 */
template<typename ConcreteCellType>
class GridMapBase
{

public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

   /**
   * @brief Indique si les coordonnées x et y sont dans les limites de la carte.
   * @return True si les coordonnées sont dans les limites de la carte
   */
  bool hasGridValue(int x, int y) const
  {
    return (x >= 0) && (y >= 0) && (x < this->getSizeX()) && (y < this->getSizeY());
  }

  const Eigen::Vector2i& getMapDimensions() const { return mapDimensionProperties.getMapDimensions(); };
  int getSizeX() const { return mapDimensionProperties.getSizeX(); };
  int getSizeY() const { return mapDimensionProperties.getSizeY(); };

  bool pointOutOfMapBounds(const Eigen::Vector2f& pointMapCoords) const
  {
    return mapDimensionProperties.pointOutOfMapBounds(pointMapCoords);
  }

  virtual void reset()
  {
    this->clear();
  }

  /**
   * @brief Réinitialise les valeurs des cellules de la grille en utilisant la fonction 
   * @fn resetGridCell().
   */
  void clear()
  {
    int size = this->getSizeX() * this->getSizeY();

    for (int i = 0; i < size; ++i) {
      this->mapArray[i].resetGridCell();
    }

    //this->mapArray[0].set(1.0f);
    //this->mapArray[size-1].set(1.0f);
  }


  const MapDimensionProperties& getMapDimProperties() const { return mapDimensionProperties; };

  /**
   * @brief Constructeur, crée la représentation de la grille et les transformations
   */
  GridMapBase(float mapResolution, const Eigen::Vector2i& size, const Eigen::Vector2f& offset)
    : mapArray(0)
    , lastUpdateIndex(-1)
  {
    Eigen::Vector2i newMapDimensions (size);

    this->setMapGridSize(newMapDimensions);
    sizeX = size[0];

    setMapTransformation(offset, mapResolution);

    this->clear();
  }

  /**
   * @brief Destructeur
   */
  virtual ~GridMapBase()
  {
    deleteArray();
  }

  /**
   * @brief Alloue de la mémoire pour le tableau de pointeurs à deux dimensions pour la représentation de la carte.
   */
  void allocateArray(const Eigen::Vector2i& newMapDims)
  {
    int sizeX = newMapDims.x();
    int sizeY = newMapDims.y();

    mapArray = new ConcreteCellType [sizeX*sizeY];

    mapDimensionProperties.setMapCellDims(newMapDims);
  }

  void deleteArray()
  {
    if (mapArray != 0){

      delete[] mapArray;

      mapArray = 0;
      mapDimensionProperties.setMapCellDims(Eigen::Vector2i(-1,-1));
    }
  }

  ConcreteCellType& getCell(int x, int y)
  {
    return mapArray[y * sizeX + x];
  }

  const ConcreteCellType& getCell(int x, int y) const
  {
    return mapArray[y * sizeX + x];
  }

  ConcreteCellType& getCell(int index)
  {
    return mapArray[index];
  }

  const ConcreteCellType& getCell(int index) const
  {
    return mapArray[index];
  }

  void setMapGridSize(const Eigen::Vector2i& newMapDims)
  {
    if (newMapDims != mapDimensionProperties.getMapDimensions() ){
     deleteArray();
     allocateArray(newMapDims);
     this->reset();
    }
  }

  /**
   * @brief Constructeur de copie, nécessaire uniquement si des membres pointeurs sont présents.
   */
  GridMapBase(const GridMapBase& other)
  {
    allocateArray(other.getMapDimensions());
    *this = other;
  }

  /**
   * @brief Opérateur d'assignation, uniquement nécessaire si des membres pointeurs sont présents.
   */
  GridMapBase& operator=(const GridMapBase& other)
  {
    if ( !(this->mapDimensionProperties == other.mapDimensionProperties)){
      this->setMapGridSize(other.mapDimensionProperties.getMapDimensions());
    }

    this->mapDimensionProperties = other.mapDimensionProperties;

    this->worldTmap = other.worldTmap;
    this->mapTworld = other.mapTworld;
    this->worldTmap3D = other.worldTmap3D;

    this->scaleToMap = other.scaleToMap;

    /** @todo possibilité de redimensionnement*/
    int sizeX = this->getSizeX();
    int sizeY = this->getSizeY();

    size_t concreteCellSize = sizeof(ConcreteCellType);

    memcpy(this->mapArray, other.mapArray, sizeX*sizeY*concreteCellSize);

    return *this;
  }

  /**
   * @brief Retourne les coordonnées mondiales pour les coordonnées de la carte donnée.
   */
  inline Eigen::Vector2f getWorldCoords(const Eigen::Vector2f& mapCoords) const
  {
    return worldTmap * mapCoords;
  }

  /**
   * @brief Retourne les coordonnées de la carte pour les coordonnées du monde données.
   */
  inline Eigen::Vector2f getMapCoords(const Eigen::Vector2f& worldCoords) const
  {
    return mapTworld * worldCoords;
  }

  /**
   * brief Retourne la pose du monde pour la pose de la carte donnée.
   */
  inline Eigen::Vector3f getWorldCoordsPose(const Eigen::Vector3f& mapPose) const
  {
    Eigen::Vector2f worldCoords (worldTmap * mapPose.head<2>());
    return Eigen::Vector3f(worldCoords[0], worldCoords[1], mapPose[2]);
  }

  /**
   * @brief Retourne la pose de la carte pour la pose du monde donnée.
   */
  inline Eigen::Vector3f getMapCoordsPose(const Eigen::Vector3f& worldPose) const
  {
    Eigen::Vector2f mapCoords (mapTworld * worldPose.head<2>());
    return Eigen::Vector3f(mapCoords[0], mapCoords[1], worldPose[2]);
  }

  void setDimensionProperties(const Eigen::Vector2f& topLeftOffsetIn, const Eigen::Vector2i& mapDimensionsIn, float cellLengthIn)
  {
    setDimensionProperties(MapDimensionProperties(topLeftOffsetIn,mapDimensionsIn,cellLengthIn));
  }

  void setDimensionProperties(const MapDimensionProperties& newMapDimProps)
  {
    /** @brief Le numéro de cellule de la carte de la grille a changé*/
    if (!newMapDimProps.hasEqualDimensionProperties(this->mapDimensionProperties)){
      this->setMapGridSize(newMapDimProps.getMapDimensions());
    }

    /** @brief la transformation de la carte/la taille des cellules a changé*/
    if(!newMapDimProps.hasEqualTransformationProperties(this->mapDimensionProperties)){
      this->setMapTransformation(newMapDimProps.getTopLeftOffset(), newMapDimProps.getCellLength());
    }
  }

  /**
   * Définir les transformations de la carte
   * @param xWorld Origine du système de coordonnées de la carte sur l'axe x, en coordonnées mondiales.
   * @param yWorld L'origine du système de coordonnées de la carte sur l'axe y en coordonnées mondiales
   * @param La longueur de la cellule de la carte de la grille
   */
  void setMapTransformation(const Eigen::Vector2f& topLeftOffset, float cellLength)
  {
    mapDimensionProperties.setCellLength(cellLength);
    mapDimensionProperties.setTopLeftOffset(topLeftOffset);

    scaleToMap = 1.0f / cellLength;

    mapTworld = Eigen::AlignedScaling2f(scaleToMap, scaleToMap) * Eigen::Translation2f(topLeftOffset[0], topLeftOffset[1]);

    worldTmap3D = Eigen::AlignedScaling3f(scaleToMap, scaleToMap, 1.0f) * Eigen::Translation3f(topLeftOffset[0], topLeftOffset[1], 0);

    //std::cout << worldTmap3D.matrix() << std::endl;
    worldTmap3D = worldTmap3D.inverse();

    worldTmap = mapTworld.inverse();
  }


  /**
   * Renvoie le facteur d'échelle d'une unité dans les coordonnées du monde à une unité dans les coordonnées de la carte.
   * @return The scale factor
   */
  float getScaleToMap() const
  {
    return scaleToMap;
  }

  /**
   * Renvoie la longueur des bords des cellules de la grille en millimètres.
   * @return la longueur du bord de la cellule en millimètres.
   */
  float getCellLength() const
  {
    return mapDimensionProperties.getCellLength();
  }

  /**
   * Renvoie une référence à la transformation 2D homogène de la carte en coordonnées mondiales.
   * @return La transformation homogène en 2D.
   */
  const Eigen::Affine2f& getWorldTmap() const
  {
    return worldTmap;
  }

  /**
   * Renvoie une référence à la transformation 3D homogène de la carte en coordonnées mondiales.
   * @return La transformation 3D homogène.
   */
  const Eigen::Affine3f& getWorldTmap3D() const
  {
    return worldTmap3D;
  }

  /**
   * Renvoie une référence à la transformation 2D homogène des coordonnées du monde vers la carte.
   * @return La transformation homogène en 2D.
   */
  const Eigen::Affine2f& getMapTworld() const
  {
    return mapTworld;
  }

  void setUpdated() { lastUpdateIndex++; };
  int getUpdateIndex() const { return lastUpdateIndex; };

  /**
    * Renvoie le rectangle ([xMin,yMin], [xMax,xMax]) contenant les valeurs de cellule non par défaut
    */
  bool getMapExtends(int& xMax, int& yMax, int& xMin, int& yMin) const
  {
    int lowerStart = -1;
    int upperStart = 10000;

    int xMaxTemp = lowerStart;
    int yMaxTemp = lowerStart;
    int xMinTemp = upperStart;
    int yMinTemp = upperStart;

    int sizeX = this->getSizeX();
    int sizeY = this->getSizeY();

    for (int x = 0; x < sizeX; ++x) {
      for (int y = 0; y < sizeY; ++y) {
        if (this->mapArray[x][y].getValue() != 0.0f) {

          if (x > xMaxTemp) {
            xMaxTemp = x;
          }

          if (x < xMinTemp) {
            xMinTemp = x;
          }

          if (y > yMaxTemp) {
            yMaxTemp = y;
          }

          if (y < yMinTemp) {
            yMinTemp = y;
          }
        }
      }
    }

    if ((xMaxTemp != lowerStart) &&
        (yMaxTemp != lowerStart) &&
        (xMinTemp != upperStart) &&
        (yMinTemp != upperStart)) {

      xMax = xMaxTemp;
      yMax = yMaxTemp;
      xMin = xMinTemp;
      yMin = yMinTemp;
      return true;
    } else {
      return false;
    }
  }

protected:

  ConcreteCellType *mapArray;    ///< Map representation used with plain pointer array.

  float scaleToMap;              ///< Scaling factor from world to map.

  Eigen::Affine2f worldTmap;     ///< Homogenous 2D transform from map to world coordinates.
  Eigen::Affine3f worldTmap3D;   ///< Homogenous 3D transform from map to world coordinates.
  Eigen::Affine2f mapTworld;     ///< Homogenous 2D transform from world to map coordinates.

  MapDimensionProperties mapDimensionProperties;
  int sizeX;

private:
  int lastUpdateIndex;
};

}

#endif
