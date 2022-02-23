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
 * @file net_serial.h
 * Fichier net_serial.h
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

#pragma once

#include "hal/abs_rxtx.h"

namespace rp{ namespace arch{ namespace net{

class raw_serial : public rp::hal::serial_rxtx
{
public:
    enum{
        SERIAL_RX_BUFFER_SIZE = 512,
        SERIAL_TX_BUFFER_SIZE = 128,
    };

    raw_serial();
    virtual ~raw_serial();
    virtual bool bind(const char * portname, uint32_t baudrate, uint32_t flags = 0);
    virtual bool open();
    virtual void close();
    virtual void flush( _u32 flags);
    
    virtual int waitfordata(size_t data_count,_u32 timeout = -1, size_t * returned_size = NULL);

    virtual int senddata(const unsigned char * data, size_t size);
    virtual int recvdata(unsigned char * data, size_t size);

    virtual int waitforsent(_u32 timeout = -1, size_t * returned_size = NULL);
    virtual int waitforrecv(_u32 timeout = -1, size_t * returned_size = NULL);

    virtual size_t rxqueue_count();

    virtual void setDTR();
    virtual void clearDTR();

    _u32 getTermBaudBitmap(_u32 baud);

    virtual void cancelOperation();

protected:
    bool open(const char * portname, uint32_t baudrate, uint32_t flags = 0);
    void _init();

    char _portName[200];
    uint32_t _baudrate;
    uint32_t _flags;

    int serial_fd;

    size_t required_tx_cnt;
    size_t required_rx_cnt;

    int    _selfpipe[2];
    bool   _operation_aborted;
};

}}}
