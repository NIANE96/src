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
 * @file sl_udp_channel.cpp
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
	class UdpChannel : public IChannel
	{
	public:
		UdpChannel(const std::string& ip, int port) : _binded_socket(rp::net::DGramSocket::CreateSocket()) {
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
            return SL_IS_OK(_binded_socket->setPairAddress(&_socket));         
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
            return _binded_socket->sendTo(NULL, data, size);
        }

        int read(void* buffer, size_t size)
        {
            u_result ans;
            size_t recCnt = 0;
            size_t lenRec = 0;
            while (size > recCnt)
            {
				sl_u8 *temp = (sl_u8 *)buffer+recCnt;
                ans = _binded_socket->recvFrom(temp, size, lenRec);
                recCnt += lenRec;
                if (ans)
                    break;
            }
            return recCnt;
        
        }

        void clearReadCache() {}

        void setStatus(_u32 flag){}

	private:
		rp::net::DGramSocket * _binded_socket;
		rp::net::SocketAddress _socket;
        std::string _ip;
        int _port;
	};

    Result<IChannel*> createUdpChannel(const std::string& ip, int port)
    {
        return new  UdpChannel(ip, port);
    }
}