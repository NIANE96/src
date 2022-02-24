/**
 *
 * PGE MASTER SME ROBOT MOBILE
 * Tous droits réservés.
 *
 * @copyright (c) 2014 - 2016 Shanghai Slamtec Co., Ltd.
 * http://www.slamtec.com
 * 
 * Système LIDAR ROBOT MOBILE
 * 
 * @file rplidar_cmd.h
 * @author NIANE
 * @author DIOUME
 * @author HOURI
 * @author BOUBACAR
 * @author DOUKI
 * @author CAMARA
 * @date   2022
 * @version 1.0 
 * 
 * 
 */

#pragma once

#include "sl_lidar_cmd.h"

namespace sl {namespace crc32 {
    sl_u32 bitrev(sl_u32 input, sl_u16 bw);//reflect
    void init(sl_u32 poly); // table init
    sl_u32 cal(sl_u32 crc, void* input, sl_u16 len);
    sl_result getResult(sl_u8 *ptr, sl_u32 len);
}}
