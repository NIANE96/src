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
 * @file net_socket.cpp
 * Fichier net_socket cpp
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
#include "sdkcommon.h"
#include <mmsystem.h>
#pragma comment(lib, "Winmm.lib")

namespace rp{ namespace arch{

static LARGE_INTEGER _current_freq;

void HPtimer_reset()
{
    BOOL ans=QueryPerformanceFrequency(&_current_freq);
    _current_freq.QuadPart/=1000;
}

_u32 getHDTimer()
{
    LARGE_INTEGER current;
    QueryPerformanceCounter(&current);

    return (_u32)(current.QuadPart/_current_freq.QuadPart);
}

BEGIN_STATIC_CODE(timer_cailb)
{
    HPtimer_reset();
}END_STATIC_CODE(timer_cailb)

}}
