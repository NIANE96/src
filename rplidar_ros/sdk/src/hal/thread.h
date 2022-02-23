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
#pragma once

#include "hal/types.h"
#define CLASS_THREAD(c , x ) \
	rp::hal::Thread::create_member<c, &c::x>(this )

namespace rp{ namespace hal{

class Thread
{
public:
    enum priority_val_t
	{
		PRIORITY_REALTIME = 0,
		PRIORITY_HIGH     = 1,
		PRIORITY_NORMAL   = 2,
		PRIORITY_LOW      = 3,
		PRIORITY_IDLE     = 4,
	};

    template <class T, u_result (T::*PROC)(void)>
    static Thread create_member(T * pthis)
    {
		return create(_thread_thunk<T,PROC>, pthis);
	}

	template <class T, u_result (T::*PROC)(void) >
	static _word_size_t THREAD_PROC _thread_thunk(void * data)
	{
		return (static_cast<T *>(data)->*PROC)();
	}
	static Thread create(thread_proc_t proc, void * data = NULL );

public:
    ~Thread() { }
    Thread():  _data(NULL),_func(NULL),_handle(0)  {}
    _word_size_t getHandle(){ return _handle;}
    u_result terminate();
    void *getData() { return _data;}
    u_result join(unsigned long timeout = -1);
	u_result setPriority( priority_val_t p);
	priority_val_t getPriority();

    bool operator== ( const Thread & right) { return this->_handle == right._handle; }
protected:
    Thread( thread_proc_t proc, void * data ): _data(data),_func(proc), _handle(0)  {}
    void * _data;
    thread_proc_t _func;
    _word_size_t _handle;
};

}}

