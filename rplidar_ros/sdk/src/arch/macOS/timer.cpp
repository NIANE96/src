/**
 * RPLIDAR SDK
 * PGE MASTER SME ROBOT MOBILE
 * Tous droits réservés.
 *
 * Copyright (c) 2014 - 2016 Shanghai Slamtec Co., Ltd.
 * http://www.slamtec.com
 * 
 * Système LIDAR ROBOT MOBILE
 * 
 * @file timer.cpp
 * Fichier timer cpp
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

#include "arch/macOS/arch_macOS.h"


namespace rp{ namespace arch{
_u64 getus()
{
    timeval now;
    gettimeofday(&now,NULL);
    return now.tv_sec*1000000 + now.tv_usec;
}
    
_u32 rp_getms()
{
    timeval now;
    gettimeofday(&now,NULL);
    return now.tv_sec*1000L + now.tv_usec/1000L;
}
    
}}
