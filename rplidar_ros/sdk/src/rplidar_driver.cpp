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
 * @file rpliar_driver.cpp
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

#include "sdkcommon.h"
#include "hal/abs_rxtx.h"
#include "hal/thread.h"
#include "hal/types.h"
#include "hal/assert.h"
#include "hal/locker.h"
#include "hal/socket.h"
#include "hal/event.h"
#include "rplidar_driver.h"
#include "sl_crc.h" 
#include <algorithm>

namespace rp { namespace standalone{ namespace rplidar {

    RPlidarDriver::RPlidarDriver(){}

    RPlidarDriver::RPlidarDriver(sl_u32 channelType) 
        :_channelType(channelType)
    {
    }

    RPlidarDriver::~RPlidarDriver() {}

    RPlidarDriver * RPlidarDriver::CreateDriver(_u32 drivertype)
    {
        //_channelType = drivertype;
        return  new RPlidarDriver(drivertype);
    }

    void RPlidarDriver::DisposeDriver(RPlidarDriver * drv)
    {
        delete drv;
    }

    u_result RPlidarDriver::connect(const char *path, _u32 portOrBaud, _u32 flag)
    {
        switch (_channelType)
        {
        case CHANNEL_TYPE_SERIALPORT:
            _channel = (*createSerialPortChannel(path, portOrBaud));
            break;
        case CHANNEL_TYPE_TCP:
            _channel = *createTcpChannel(path, portOrBaud);
            break;
        case CHANNEL_TYPE_UDP:
            _channel = *createUdpChannel(path, portOrBaud);
            break;
        }
        if (!(bool)_channel) return SL_RESULT_OPERATION_FAIL;
        
        _lidarDrv = *createLidarDriver();

        if (!(bool)_lidarDrv) return SL_RESULT_OPERATION_FAIL;

        sl_result ans =(_lidarDrv)->connect(_channel);
        return ans;
    }

    void RPlidarDriver::disconnect()
    {
        (_lidarDrv)->disconnect();
    }

    bool RPlidarDriver::isConnected() 
    { 
        return (_lidarDrv)->isConnected();
    }
     
    u_result RPlidarDriver::reset(_u32 timeout)
    {
        return (_lidarDrv)->reset();
    }

    u_result RPlidarDriver::getAllSupportedScanModes(std::vector<RplidarScanMode>& outModes, _u32 timeoutInMs)
    {
        return (_lidarDrv)->getAllSupportedScanModes(outModes, timeoutInMs);
    }

    u_result RPlidarDriver::getTypicalScanMode(_u16& outMode, _u32 timeoutInMs)
    {
        return (_lidarDrv)->getTypicalScanMode(outMode, timeoutInMs);
    }

    u_result RPlidarDriver::startScan(bool force, bool useTypicalScan, _u32 options, RplidarScanMode* outUsedScanMode)
    {
        return (_lidarDrv)->startScan(force, useTypicalScan, options, outUsedScanMode);
    }

    u_result RPlidarDriver::startScanExpress(bool force, _u16 scanMode, _u32 options, RplidarScanMode* outUsedScanMode, _u32 timeout)
    {
        return (_lidarDrv)->startScanExpress(force, scanMode, options, outUsedScanMode, timeout);
    }
    
    u_result RPlidarDriver::getHealth(rplidar_response_device_health_t & health, _u32 timeout)
    {
        return (_lidarDrv)->getHealth(health, timeout);
    }

    u_result RPlidarDriver::getDeviceInfo(rplidar_response_device_info_t & info, _u32 timeout)
    {
        return (_lidarDrv)->getDeviceInfo(info, timeout);
    }

    u_result RPlidarDriver::setMotorPWM(_u16 pwm)
    {
        return (_lidarDrv)->setMotorSpeed(pwm);
    }   
    
    u_result RPlidarDriver::checkMotorCtrlSupport(bool & support, _u32 timeout)
    {
        MotorCtrlSupport motorSupport;
        u_result ans = (_lidarDrv)->checkMotorCtrlSupport(motorSupport, timeout);
        if (motorSupport == MotorCtrlSupportNone)
            support = false;
        return ans;
    }

    u_result RPlidarDriver::setLidarIpConf(const rplidar_ip_conf_t& conf, _u32 timeout)
	{
		return (_lidarDrv)->setLidarIpConf(conf, timeout);
	}

    u_result RPlidarDriver::getDeviceMacAddr(_u8* macAddrArray, _u32 timeoutInMs)
	{
		return (_lidarDrv)->getDeviceMacAddr(macAddrArray, timeoutInMs);
	}

    u_result RPlidarDriver::stop(_u32 timeout) 
    { 
        return (_lidarDrv)->stop(timeout);
    }

    u_result RPlidarDriver::grabScanDataHq(rplidar_response_measurement_node_hq_t * nodebuffer, size_t & count, _u32 timeout)
    {
        return (_lidarDrv)->grabScanDataHq(nodebuffer, count, timeout);
    }

    u_result RPlidarDriver::ascendScanData(rplidar_response_measurement_node_hq_t * nodebuffer, size_t count)
    {
        return (_lidarDrv)->ascendScanData(nodebuffer, count);
    }
    
    u_result RPlidarDriver::getScanDataWithInterval(rplidar_response_measurement_node_t * nodebuffer, size_t & count)
    {
        return RESULT_OPERATION_NOT_SUPPORT;
    }

    u_result RPlidarDriver::getScanDataWithIntervalHq(rplidar_response_measurement_node_hq_t * nodebuffer, size_t & count)
    {
        return (_lidarDrv)->getScanDataWithIntervalHq(nodebuffer, count);
    }

    u_result RPlidarDriver::startMotor()
    {
        return (_lidarDrv)->setMotorSpeed(600);
    }
    u_result RPlidarDriver::stopMotor()
    {
        return (_lidarDrv)->setMotorSpeed(0);
    }

}}}
