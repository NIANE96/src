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
 * @file rpliar_driver_TCP.h
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

class TCPChannelDevice :public ChannelDevice
{
public:
    rp::net::StreamSocket * _binded_socket;
    TCPChannelDevice():_binded_socket(rp::net::StreamSocket::CreateSocket()){}

    bool bind(const char * ipStr, uint32_t port)
    {
        rp::net::SocketAddress socket(ipStr, port);
        return IS_OK(_binded_socket->connect(socket));
    }
    void close()
    {
        _binded_socket->dispose();
        _binded_socket = NULL;
    }
    bool waitfordata(size_t data_count,_u32 timeout = -1, size_t * returned_size = NULL)
    {
        if(returned_size)
            *returned_size = data_count;
        return (_binded_socket->waitforData(timeout) == RESULT_OK);
    }
    int senddata(const _u8 * data, size_t size)
    {
        return _binded_socket->send(data, size) ;
    }
    int recvdata(unsigned char * data, size_t size)
    {
        size_t lenRec = 0;
        _binded_socket->recv(data, size, lenRec);
        return lenRec;
    }
};


class RPlidarDriverTCP : public RPlidarDriverImplCommon
{
public:

    RPlidarDriverTCP();
    virtual ~RPlidarDriverTCP();
    virtual u_result connect(const char * ipStr, _u32 port, _u32 flag = 0);
    virtual void disconnect();
};


}}}