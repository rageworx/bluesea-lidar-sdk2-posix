#ifndef __LIDARCHECKSERVICE_H__
#define __LIDARCHECKSERVICE_H__
#pragma once

#include <pthread.h>
#include "Global.h"

class LidarCheckService
{
    public:
        LidarCheckService();
        ~LidarCheckService();

        void run();
        void stop();
        void clear();
        std::vector<DevConnInfo> getLidarsList();
        void uartDevInfo();

    public:
        static void getTime_HMS(char*data, size_t datalen);

    private:
        pthread_t m_thread_heart;
        bool m_close_service;
};

void* thread_heart(void* p);
void uptodate(DevConnInfo data);

#endif /// of __LIDARCHECKSERVICE_H__