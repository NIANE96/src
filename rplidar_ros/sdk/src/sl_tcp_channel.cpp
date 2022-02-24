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
 * @file sl_tcp_channel.cpp
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
#include "sl_lidar_driver.h"
#include "hal/abs_rxtx.h"
#include "hal/socket.h"


namespace sl {
    
    class TcpChannel : public IChannel
    {
    public:
        TcpChannel(const std::string& ip, int port) : _binded_socket(rp::net::StreamSocket::CreateSocket()) {
            _ip = ip;
            _port = port;
        }

        bool bind(const std::string & ip, sl_s32 port)
        {
            _socket = rp::net::SocketAddress(ip.c_str(), port);
            return SL_RESULT_OK;
        }

        bool open()
        {
            if(SL_IS_FAIL(bind(_ip, _port)))
                return false;
            return IS_OK(_binded_socket->connect(_socket));
            
        }

        void close()
        {
            _binded_socket->dispose();
            _binded_socket = NULL;
        }
        void flush()
        {
        
        }

        bool waitForData(size_t size, sl_u32 timeoutInMs, size_t* actualReady)
        {
            if (actualReady)
                *actualReady = size;
            return (_binded_socket->waitforData(timeoutInMs) == RESULT_OK);

        }

        int write(const void* data, size_t size)
        {
            return _binded_socket->send(data, size);
        }

        int read(void* buffer, size_t size)
        {
            size_t lenRec = 0;
            _binded_socket->recv(buffer, size, lenRec);
            return lenRec;
        }

        void clearReadCache() {}

        void setStatus(_u32 flag){}
    private:
        rp::net::StreamSocket * _binded_socket;
        rp::net::SocketAddress _socket;
        std::string _ip;
        int _port;
    };
    Result<IChannel*> createTcpChannel(const std::string& ip, int port)
    {
        return new  TcpChannel(ip, port);
    }
}