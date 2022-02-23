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
 * @file pratical_camon.h
 * Fichier pratical_camon h
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

#ifndef PRACTICAL_COMMON_H_
#define PRACTICAL_COMMON_H_
#include "nav_msgs/OccupancyGrid.h"
#include <math.h>
#include<iostream>

/** @distance brève du centre du robot au point extérieur le plus éloigné, en mètres*/
const int OCCUPIED_THRESHOLD = 50;

/** @distance brève du centre du robot au point extérieur le plus éloigné, en mètres*/
const double ROBOT_RADIUS = .14;

/** @brief 3 fonctions d'aide pour obtenir les coordonnées x, y à partir de l'index, et l'index à partir des coordonnées x, y, sur la base de index = ogm.info.width * y + x */
int getX(int index, const nav_msgs::OccupancyGridPtr &map)
{
    return index % map->info.width;
}
int getY(int index, const nav_msgs::OccupancyGridPtr &map)
{
    return index / map->info.width;
}
int getIndex(int x, int y, const nav_msgs::OccupancyGridPtr &map)
{
    return map->info.width * y + x;
}


/** @brief fonction d'aide pour rester dans les limites de la carte et éviter les erreurs de segmentation*/
bool is_in_bounds(int x, int y, const nav_msgs::OccupancyGridPtr &map)
{
    return (x >= 0 && x < map->info.width && y >= 0 && y < map->info.height);
}

/** @brief helper pour vérifier si la cellule est marquée comme étant inconnue*/
bool is_unknown(int x, int y, const nav_msgs::OccupancyGridPtr &map)
{
    return ((int)map->data[getIndex(x, y, map)] == -1);
}


/** @brief helper pour vérifier si la cellule doit être considérée comme un obstacle - inclut les cellules marquées inconnues*/
bool is_obstacle(int x, int y, const nav_msgs::OccupancyGridPtr &map)
{
 //   std::cout<<x<<", "<<y<< " .... "<<(int)map->data[getIndex(x, y, map)]<<std::endl;
    return ((int)map->data[getIndex(x, y, map)] > OCCUPIED_THRESHOLD) || is_unknown(x, y, map);
}


/** @brieff helper to return map resolution*/
double map_resolution(const nav_msgs::OccupancyGridPtr &map)
{
    return map->info.resolution;
}

//returns slope m from slope intercept formula y=m*x+b from two coordinate pairs
//don't forget all coordinates must be either pose in meters or grid cells numbers
// ( grid cell number = pose(in meters) / map_resolution  )
//DO NOT MIX POSE COORDINATES WITH GRID CELL COORDINATES - MAKE ALL THE SAME
double get_m(double x1, double y1, double x2, double y2)
{
    //****CAUTION< WILL THROW ERROR IF WE DIVIDE BY ZERO
    return (y1 - y2) / (x1 - x2);
}

// b as is the offset from slope intercept formula y=m*x+b
//for b = y-(m*x)
//DO NOT MIX POSE COORDINATES WITH GRID CELL COORDINATES - MAKE ALL THE SAME
double get_b(double x1, double y1, double x2, double y2)
{
  //  ne peut pas diviser par zer0
    if(x1 != x2)
    {
    return y1 - (get_m(x1, y1, x2, y2) * x1);
    }
    else return x1; //La ligne est verticale, donc b = x1
}

/** @brief renvoie l'emplacement de y sur une ligne entre deux points fournis, pour un x donné.
* renvoie Y à partir de la forumule d'interception de la pente y=m*x+b, pour x donné.
* NE PAS MÉLANGER LES COORDONNÉES DE LA POSE AVEC LES COORDONNÉES DES CELLULES DE LA GRILLE - LES * RENDRE TOUTES IDENTIQUES
* NE GÈRE PAS LES LIGNES VERTICALES */
double get_y_intercept(double x1, double y1, double x2, double y2, double checkX)
{
    double m = get_m(x1, y1, x2, y2);
    double b = get_b(x1, y1, x2, y2);
    return m * checkX + b;
}

/** @brief renvoie la position de y sur une ligne entre deux points fournis, pour y donné.
* renvoie x à partir de la formule de l'interception de la pente y=m*x+b, pour y donné. x= (y-b)/m
* NE PAS MÉLANGER LES COORDONNÉES DE LA POSE AVEC LES COORDONNÉES DES CELLULES DE LA GRILLE - LES * RENDRE TOUTES IDENTIQUES
* NE GÈRE PAS LES LIGNES VERTICALES*/
double get_x_intercept(double x1, double y1, double x2, double y2, double checkY)
{

    double m = get_m(x1, y1, x2, y2);
    double b = get_b(x1, y1, x2, y2);
    return (checkY - b) / m;
}


#endif /* PRACTICAL_COMMON_H_ */
