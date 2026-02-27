#ifndef __LIDARSERVICE_H__
#define __LIDARSERVICE_H__
#pragma once

#include <Global.h>

class LidarService
{
    public:
        LidarService();
        ~LidarService();

    public:
        bool    Run();
        void    Stop();
        void    Clear();
        void    UpdateUartInfo();
        int32_t State();
        
    public:
        std::vector<DevConnInfo> LidarList();
        
    public:
        static void GetTime_HMS( char* data, size_t datalen );

    public: /// do not call this function directly.
        int32_t ThreadCallBack( void* param );

    private:
        pthread_t       ptHeart;
        bool            bCloseService;        
};

#endif /// of __LIDARCHECKSERVICE_H__