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
 * @file GridReflectance.h
 * Fichier GridReflectance h
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
#ifndef __GridMapReflectanceCount_h_
#define __GridMapReflectanceCount_h_

/**
 * Fournit une représentation du nombre de réflectances pour les cellules dans une carte de grille d'occupation.
 */
class ReflectanceCell
{
public:

  void set(float val)
  {
    probOccupied = val;
  }

  float getValue() const
  {
    return probOccupied;
  }

  bool isOccupied() const
  {
    return probOccupied > 0.5f;
  }

  bool isFree() const{
    return probOccupied < 0.5f;
  }

  void resetGridCell()
  {
    probOccupied = 0.5f;
    visitedCount = 0.0f;
    reflectedCount = 0.0f;
    updateIndex = -1;
  }

//protected:

  float visitedCount;
  float reflectedCount;
  float probOccupied;
  int updateIndex;
};


class GridMapReflectanceFunctions
{
public:

  GridMapReflectanceFunctions()
  {}

  void updateSetOccupied(ReflectanceCell& cell) const
  {
    ++cell.reflectedCount;
    ++cell.visitedCount;
    cell.probOccupied = cell.reflectedCount / cell.visitedCount;
  }

  void updateSetFree(ReflectanceCell& cell) const
  {
    ++cell.visitedCount;
    cell.probOccupied = cell.reflectedCount / cell.visitedCount;
  }

  void updateUnsetFree(ReflectanceCell& cell) const
  {
    --cell.visitedCount;
    cell.probOccupied = cell.reflectedCount / cell.visitedCount;
  }

  float getGridProbability(const ReflectanceCell& cell) const
  {
    return cell.probOccupied;
  }

protected:

};


#endif
