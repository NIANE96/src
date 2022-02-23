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
 * @file GridMapLogo.h
 * Fichier GridMapLogo h
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
#ifndef __GridMapLogOdds_h_
#define __GridMapLogOdds_h_

#include <cmath>

/**
 * Fournit une représentation des probabilités d'occupation en logarithme pour les cellules d'une carte de grille d'occupation.
 */
class LogOddsCell
{
public:

  /*
  void setOccupied()
  {
    logOddsVal += 0.7f;
  };

  void setFree()
  {
    logOddsVal -= 0.4f;
  };
  */


  /**
   * Attribue la valeur val à la cellule.
   * @param val La valeur logarithmique des chances.
   */
  void set(float val)
  {
    logOddsVal = val;
  }

  /**
   * Renvoie la valeur de la cellule.
   * @return La valeur de la probabilité logarithmique.
   */
  float getValue() const
  {
    return logOddsVal;
  }

  /**
   * Retourne si la cellule est occupée.
   * @return La cellule est occupée
   */
  bool isOccupied() const
  {
    return logOddsVal > 0.0f;
  }

  bool isFree() const
  {
    return logOddsVal < 0.0f;
  }

  /**
   * Remettre la cellule à la probabilité antérieure.
   */
  void resetGridCell()
  {
    logOddsVal = 0.0f;
    updateIndex = -1;
  }

  //protected:

public:

  float logOddsVal; ///< The log odds representation of occupancy probability.
  int updateIndex;


};

/**
 * Fournit des fonctions liées à une représentation des probabilités d'occupation en logarithme pour les cellules dans une carte de grille d'occupation.
 */
class GridMapLogOddsFunctions
{
public:

  /**
   * Constructeur, définit les paramètres tels que les logarithmes des odds ratios libres et occupés.
   */
  GridMapLogOddsFunctions()
  {
    this->setUpdateFreeFactor(0.4f);
    this->setUpdateOccupiedFactor(0.6f);
    /*
    //float probOccupied = 0.6f;
    float probOccupied = 0.9f;
    float oddsOccupied = probOccupied / (1.0f - probOccupied);
    logOddsOccupied = log(oddsOccupied);

    float probFree = 0.4f;
    float oddsFree = probFree / (1.0f - probFree);
    logOddsFree = log(oddsFree);
    */
  }

  /**
   * Mettre à jour la cellule comme étant occupée
   * @param La cellule.
   */
  void updateSetOccupied(LogOddsCell& cell) const
  {
    if (cell.logOddsVal < 50.0f){
      cell.logOddsVal += logOddsOccupied;
    }
  }

  /**
   * Mettre à jour la cellule comme étant libre
   * @param La cellule.
   */
  void updateSetFree(LogOddsCell& cell) const
  {

    cell.logOddsVal += logOddsFree;

  }

  void updateUnsetFree(LogOddsCell& cell) const
  {
    cell.logOddsVal -= logOddsFree;
  }

  /**
   * Obtenir la valeur de probabilité représentée par la cellule de la grille.
   * @param la cellule.
   * @return La probabilité
   */
  float getGridProbability(const LogOddsCell& cell) const
  {
    float odds = exp(cell.logOddsVal);
    return odds / (odds + 1.0f);

    /*
    float val = cell.logOddsVal;

    //prevent #IND when doing exp(large number).
    if (val > 50.0f) {
      return 1.0f;
    } else {
      float odds = exp(val);
      return odds / (odds + 1.0f);
    }
    */
    //return 0.5f;
  }

  //void getGridValueMap( const LogOddsCell& cell) const{};
  //void isOccupied(LogOddsCell& cell) {};

  //void resetGridCell() {};

  void setUpdateFreeFactor(float factor)
  {
    logOddsFree = probToLogOdds(factor);
  }

  void setUpdateOccupiedFactor(float factor)
  {
    logOddsOccupied = probToLogOdds(factor);
  }

protected:

  float probToLogOdds(float prob)
  {
    float odds = prob / (1.0f - prob);
    return log(odds);
  }

  float logOddsOccupied; /// < The log odds representation of probability used for updating cells as occupied
  float logOddsFree;     /// < The log odds representation of probability used for updating cells as free


};


#endif
