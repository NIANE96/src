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
 * @file GridMapSimpleCount.h
 * Fichier GridMapSimpleCount h
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
#ifndef __GridMapSimpleCount_h_
#define __GridMapSimpleCount_h_


/**
 * Fournit une représentation (très simple) de l'occupation basée sur le nombre de personnes.
 */
class SimpleCountCell
{
public:

  /**
   * Définit la valeur de la cellule à val.
   * @param val La valeur de la cote logarithmique.
   */
  void set(float val)
  {
    simpleOccVal = val;
  }

  /**
   * Retourne la valeur de la cellule.
   * @return La valeur de la cote logarithmique.
   */
  float getValue() const
  {
    return simpleOccVal;
  }

  /**
   * Retourne si la cellule est occupée.
   * @return Cell est occupé
   */
  bool isOccupied() const
  {
    return (simpleOccVal > 0.5f);
  }

  bool isFree() const
  {
    return (simpleOccVal < 0.5f);
  }

   /**
   * Remettre la cellule à la probabilité antérieure.
   */
  void resetGridCell()
  {
    simpleOccVal = 0.5f;
    updateIndex = -1;
  }

//protected:

public:

  float simpleOccVal; 
  int updateIndex;


};

/**
 * Fournit des fonctions liées à une représentation de la probabilité d'occupation en logarithme pour les cellules dans une carte à grille d'occupation.
 */
class GridMapSimpleCountFunctions
{
public:

  /**
   * Constructeur, définit les paramètres comme les rapports de cotes logarithmiques libres et occupés.
   */
  GridMapSimpleCountFunctions()
  {
    updateFreeVal = -0.10f;
    updateOccVal  =  0.15f;

    updateFreeLimit = -updateFreeVal + updateFreeVal/100.0f;
    updateOccLimit  = 1.0f - (updateOccVal + updateOccVal/100.0f);
  }

 /**
   * Met à jour la cellule comme occupée
   * @param cell la cellule.
   */
  void updateSetOccupied(SimpleCountCell& cell) const
  {
    if (cell.simpleOccVal < updateOccLimit){
      cell.simpleOccVal += updateOccVal;
    }
  }

  /**
   * Met à jour la cellule comme libre
   * @param cell la cellule.
   */
  void updateSetFree(SimpleCountCell& cell) const
  {
    if (cell.simpleOccVal > updateFreeLimit){
      cell.simpleOccVal += updateFreeVal;
    }
  }

  void updateUnsetFree(SimpleCountCell& cell) const
  {
    cell.simpleOccVal -= updateFreeVal;
  }

  /**
   * Obtenez la valeur de probabilité représentée par la cellule de la grille.
   * @param cell la cellule.
   * @return la probabilité
   */
  float getGridProbability(const SimpleCountCell& cell) const
  {
    return cell.simpleOccVal;
  }

protected:

  float updateFreeVal;
  float updateOccVal;

  float updateFreeLimit;
  float updateOccLimit;


};


#endif
