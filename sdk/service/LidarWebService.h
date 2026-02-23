#ifndef __LIDARWEBSERVICE_H__
#define __LIDARWEBSERVICE_H__
#pragma once

#include <pthread.h>
#include <iostream>
#include "data.h"

extern "C"
{
	#include "cJSON.h"
}

class LidarWebService
{
    public:
        LidarWebService(uint16_t port);
        ~LidarWebService();

    public:
        void run(int lidarID);
        void stop();
        
    private:
        uint16_t m_port;
        bool     m_loop;        
};

#endif /// of __LIDARWEBSERVICE_H__