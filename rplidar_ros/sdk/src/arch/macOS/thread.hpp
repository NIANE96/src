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
 * @file thread.hpp
 * Fichier thread hpp
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

#include "arch/macOS/arch_macOS.h"

namespace rp{ namespace hal{

Thread Thread::create(thread_proc_t proc, void * data)
{
    Thread newborn(proc, data);
    
    // tricky code, we assume pthread_t is not a structure but a word size value
    assert( sizeof(newborn._handle) >= sizeof(pthread_t));

    pthread_create((pthread_t *)&newborn._handle, NULL,(void * (*)(void *))proc, data);

    return newborn;
}

u_result Thread::terminate()
{
    if (!this->_handle) return RESULT_OK;
    
  //  return pthread_cancel((pthread_t)this->_handle)==0?RESULT_OK:RESULT_OPERATION_FAIL;
    return RESULT_OK;
}

u_result Thread::setPriority( priority_val_t p)
{
	if (!this->_handle) return RESULT_OPERATION_FAIL;
    // simply ignore this request
	return  RESULT_OK;
}

Thread::priority_val_t Thread::getPriority()
{
	return PRIORITY_NORMAL;
}

u_result Thread::join(unsigned long timeout)
{
    if (!this->_handle) return RESULT_OK;
    
    pthread_join((pthread_t)(this->_handle), NULL);
    return RESULT_OK;
}

}}
