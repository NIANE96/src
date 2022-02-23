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
 * @file GridMapCase.h
 * Fichier GridMapCase h
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
#ifndef __GridMapCacheArray_h_
#define __GridMapCacheArray_h_

#include <Eigen/Core>

class CachedMapElement
{
public:
  float val;
  int index;
};

/**
 * @brief Met en cache les accès à la carte de la grille filtrée dans un tableau bidimensionnel de la même taille que la carte.
 */
class GridMapCacheArray
{
public:

  /**
   * Constructeur
   */
  GridMapCacheArray()
    : cacheArray(0)
    , arrayDimensions(-1,-1)
  {
    currCacheIndex = 0;
  }

  /**
   * Destructeur
   */
  ~GridMapCacheArray()
  {
    deleteCacheArray();
  }

  /**
   * Réinitialisation/suppression des données mises en cache
   */
  void resetCache()
  {
    currCacheIndex++;
  }

  /**
   * Vérifie si les données mises en cache pour les coordonnées sont disponibles. Si c'est le cas, écrit les données dans val.
   * @param coords Les coordonnées
   * @param val Référence à un flottant dans lequel les données sont écrites si disponible
   * @return Indique si les données mises en cache sont disponibles
   */
  bool containsCachedData(int index, float& val)
  {
    const CachedMapElement& elem (cacheArray[index]);

    if (elem.index == currCacheIndex) {
      val = elem.val;
      return true;
    } else {
      return false;
    }
  }

  /**
   * Met en cache la valeur flottante val pour les coordonnées coords.
   * @param coords Les coordonnées
   * @param val la valeur à mettre en cache pour les coordonnées.
   */
  void cacheData(int index, float val)
  {
    CachedMapElement& elem (cacheArray[index]);
    elem.index = currCacheIndex;
    elem.val = val;
  }

  /**
   * Définit la taille de la carte et redimensionne le tableau de cache en conséquence.
   * @param La taille de la carte.
   */
  void setMapSize(const Eigen::Vector2i& newDimensions)
  {
    setArraySize(newDimensions);
  }

protected:

  /**
   * Crée un tableau de cache de taille sizeIn.
   * @param sizeIn La taille du tableau
   */
  void createCacheArray(const Eigen::Vector2i& newDimensions)
  {
    arrayDimensions = newDimensions;

    int sizeX = arrayDimensions[0];
    int sizeY = arrayDimensions[1];

    int size = sizeX * sizeY;

    cacheArray = new CachedMapElement [size];

    for (int x = 0; x < size; ++x) {
      cacheArray[x].index = -1;
    }
  }

  /**
   * Supprime le tableau de cache existant.
   */
  void deleteCacheArray()
  {
    delete[] cacheArray;
  }

  /**
   * Définir une nouvelle taille de tableau de cache
   */
  void setArraySize(const Eigen::Vector2i& newDimensions)
  {
    if (this->arrayDimensions != newDimensions) {
      if (cacheArray != 0) {
        deleteCacheArray();
        cacheArray = 0;
      }
      createCacheArray(newDimensions);
    }
  }

protected:

  CachedMapElement* cacheArray;    ///< Array used for caching data.
  int currCacheIndex;              ///< The cache iteration index value

  Eigen::Vector2i arrayDimensions; ///< The size of the array

};


#endif
