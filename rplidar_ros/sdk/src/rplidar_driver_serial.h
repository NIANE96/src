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
 * @file rpliar_driver_serial.h
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

namespace rp { namespace standalone{ namespace rplidar {

class SerialChannelDevice :public ChannelDevice
{
public:
    rp::hal::serial_rxtx  * _rxtxSerial;
    bool _closePending;

    SerialChannelDevice():_rxtxSerial(rp::hal::serial_rxtx::CreateRxTx()){}

    bool bind(const char * portname, uint32_t baudrate)
    {
        _closePending = false;
        return _rxtxSerial->bind(portname, baudrate);
    }
    bool open()
    {
        return _rxtxSerial->open();
    }
    void close()
    {
        _closePending = true;
        _rxtxSerial->cancelOperation();
        _rxtxSerial->close();
    }
    void flush()
    {
        _rxtxSerial->flush(0);
    }
    bool waitfordata(size_t data_count,_u32 timeout = -1, size_t * returned_size = NULL)
    {
        if (_closePending) return false;
        return (_rxtxSerial->waitfordata(data_count, timeout, returned_size) == rp::hal::serial_rxtx::ANS_OK);
    }
    int senddata(const _u8 * data, size_t size)
    {
        return _rxtxSerial->senddata(data, size) ;
    }
    int recvdata(unsigned char * data, size_t size)
    {
        size_t lenRec = 0;
        lenRec = _rxtxSerial->recvdata(data, size);
        return lenRec;
    }
    void setDTR()
    {
        _rxtxSerial->setDTR();
    }
    void clearDTR()
    {
        _rxtxSerial->clearDTR();
    }
    void ReleaseRxTx()
    {
        rp::hal::serial_rxtx::ReleaseRxTx(_rxtxSerial);
    }
};

class RPlidarDriverSerial : public RPlidarDriverImplCommon
{
public:

    RPlidarDriverSerial();
    virtual ~RPlidarDriverSerial();
    virtual u_result connect(const char * port_path,  _u32 baudrate, _u32 flag = 0);
    virtual void disconnect();

};

}}}
