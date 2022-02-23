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
#include "hal/thread.h"

#if defined(_WIN32)
#include "arch/win32/winthread.hpp"
#elif defined(_MACOS)
#include "arch/macOS/thread.hpp"
#elif defined(__GNUC__)
#include "arch/linux/thread.hpp"
#else
#error no threading implemention found for this platform.
#endif


