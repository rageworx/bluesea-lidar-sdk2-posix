#ifndef __PACECATGLOBAL_H__
#define __PACECATGLOBAL_H__
#pragma once

#ifdef _WIN32 /// not for M$VC,
    #include <windows.h>
#endif

#include <unistd.h>
#include <fcntl.h>

#include <set>
#include <cmath>
#include <cstdint>

#ifdef __linux__
    #include <sys/ioctl.h>
    #include <netinet/in.h>
    #include <netinet/tcp.h>
    #include <sys/socket.h>
    #include <termios.h>
#else
    #include <winsock.h>
#endif /// of __linux__

#include <pthread.h>
#include "data.h"

#ifndef msleep
    #define msleep(msec) usleep(msec * 1000)
#endif /// of msleep

struct UARTARG
{
    char portName[16];
    uint16_t port;
};

namespace BaseAPI 
{
    bool checkAndMerge( uint32_t type, 
                        const char* ip, const char* mask, const char* gateway, \
                        uint16_t port, 
                        char* result, size_t result_len );
    std::string stringfilter( const char *str, size_t num );
}

namespace ParseAPI 
{
    int32_t parse_data_x( uint32_t len, uint8_t* buf,UartState *uartstate, \
                          RawData& dat, int32_t& consume, int32_t with_chk, \
                          int32_t &byte, char *result, size_t result_len, \
                          CmdHeader *cmdheader, void** fan_segs );
    int32_t parse_data( uint32_t len, uint8_t* buf, 
                        UartState *uartstate, RawData& dat, \
                        int32_t& consume, int32_t with_chk );
}

namespace UserAPI 
{
    void fan_data_process( const RawData& raw, std::vector<RawData>& whole_datas );
    int32_t whole_data_process( const RawData& raw, int32_t collect_angle, \
                                std::vector<RawData> &whole_datas,std::string &error);
    int32_t autoGetFirstAngle( const RawData &raw, bool from_zero, \
                               std::vector<RawData> &raws, std::string &error);
}

namespace AlgorithmAPI
{
    //E100
    int32_t ShadowsFilter(UserData*, const ShadowsFilterParam&);
    int32_t MedianFilter(UserData*, const MedianFilterParam&);
    //E330
    bool filter( std::vector<DataPoint>& output_scan, double max_range, \
                 double min_range, double max_range_difference, \
                 size_t filter_window, double angle_increment );
}

namespace SystemAPI 
{
    int32_t GetComList(std::vector<UARTARG>& list);
    int32_t closefd(int __fd, bool isSocket);
    std::vector<std::string> GetComPort();
    int32_t open_serial_port(const char* name, uint32_t speed);
    int32_t open_socket_port(int32_t localhost);
}

int32_t GetDevInfoByVPC(const char* port_str, uint32_t speed);
int32_t GetDevInfoByUART(const char* port_str, uint32_t speed);

namespace CommunicationAPI 
{
    void send_cmd_vpc(int hCom, uint32_t mode, uint32_t sn, uint32_t len, \
                      const char* cmd);
    bool uart_talk(int hCom, size_t n, const char* cmd, int32_t nhdr, \
                   const char* hdr_str, int32_t nfetch, char* fetch, 
                   uint64_t retrycount = 100);
    bool vpc_talk(int hCom, int32_t mode, int16_t sn, size_t len, \
                  const char* cmd, int32_t nfetch, void* fetch);
    void send_cmd_udp(int fd_udp, const char* dev_ip, uint16_t dev_port, \
                      int32_t cmd, int32_t sn, size_t len, const void* snd_buf);
    bool udp_talk_C_PACK(int fd_udp, const char* lidar_ip, uint16_t lidar_port,\
                         size_t n, const char* cmd, int32_t nhdr, \
                         const char* hdr_str, int32_t nfetch, char* fetch);
    bool udp_talk_S_PACK(int fd_udp, const char* ip, uint16_t port, size_t n, \
                         const char* cmd, void* result);
    bool udp_talk_GS_PACK(int fd_udp, const char* ip, uint16_t port, size_t n, \
                          const char* cmd, void* result);
}

uint32_t stm32crc(uint32_t* ptr, uint32_t len);

#ifdef _WIN32
    void gettimeofday(timeval* tv, void*);
#endif

#ifdef _WIN32
    int32_t Open_serial_port(const char* name, uint16_t port);
#elif __linux__
    extern "C"  int32_t change_baud(int fd, uint32_t baud);
    int32_t Open_serial_port(const char* name, uint16_t port);
#endif

bool judgepcIPAddrIsValid(const char* pcIPAddr);
void initSocket();
void finalSocket();

#ifndef PRTSTDERR
    #define PRTSTDERR(...) \
            fprintf(stderr,__VA_ARGS__); fflush(stderr)
#endif /// of PRTSTDERR

#endif /// of __PACECATGLOBAL_H__