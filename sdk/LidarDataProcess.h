#ifndef __LIDARDATAPROCESS_H__
#define __LIDARDATAPROCESS_H__

#include <Global.h>

#ifdef __unix__ 
    #include <sys/time.h>
#endif

typedef void (*printfMsg)(int, void*,int);

enum ACTION
{
    OFFLINE=0,
    ONLINE,
    RUN,
    CONTROL,
    GETALLPARAMS,
    SETPARAM,
    READZONE,
    WRITEZONE,
    FINISH
};
enum STATE
{
    INIT=0,
    WORK,
    STOP_ALL
};

struct Responsive
{
    int32_t mode;               /// one of : 0x0043  0x0053  0x4753
    int32_t send_len;
    char    send_cmd[1024];
    short   rand;
    long    timestamp;
};

struct RunScript
{
    //connect
    char    type[16];           /// "uart"   or   "udp"  "vpc"
    char    connectArg[16];     /// ip/com
    int32_t connectArg2;        /// port/baud
    int32_t local_port;
    //data
    int32_t from_zero;
    int32_t output_360;         /// Fan printing (0: Partial fan printing 1: Completed)

    int32_t error_circle;       /// Detect the number of turns with a length of 0
    double  error_scale;        /// Detect the Scale of turns with a length of 0

    int32_t is_open_service;    /// Enable local service
    //udp
    int32_t is_group_listener;  /// 0 Normal mode 1 Listening mode 2 Sending mode
    char    group_ip[16];       /// CN:组播IP

    // E120 scan filter
    ShadowsFilterParam shadows_filter;
    MedianFilterParam median_filter;
    //E330 scan filter
    SeparationFilterParam separation_filter;
    //get
    int32_t uuid;
    int32_t model;
    int32_t version;

    //set common
    int32_t rpm;
    int32_t resample_res;

    int32_t with_smooth;
    int32_t with_deshadow;

    int32_t with_start;
    //set uart
    int32_t with_confidence;
    int32_t unit_is_mm;
    //set udp  vpc
    int32_t alarm_msg;
    int32_t direction;
    int32_t ats;  //1udp  2vpc
    //ntp
    char    ntp_ip[16];
    int32_t ntp_port;
    int32_t ntp_enable;
};

struct RunConfig
{   
    int32_t ID;
    // STATE:
    //     -1 == stop all
    //      0 == initialize
    //      1 == running work thread
    STATE   state;

    pthread_t thread_data;

    int32_t fd;                 /// 句柄
    printfMsg  callback;        /// Information printing callback function
    UserData userdata;          /// 当前帧
    LidarMsgHdr zonemsg;        /// 当前的防区信息
    EEpromV101 eepromv101;      /// 全局变量参数
    char hardwareVersion[32];   /// 硬件版本号
    
    // ACTION : 
    //   0 No operation
    //   1. Start/Stop control command
    //   2. Get global device parameters 
    //   3. Set device parameters 
    //   4. Read zone 
    //   5. Set zone
    ACTION action;
    
    // The currently passed instruction
    int32_t mode;
    int32_t send_len;
    char    send_cmd[1024];
    int32_t recv_len;
    char    recv_cmd[1024];

    RunScript  runscript;
};

int32_t setup_lidar_uart(int32_t fd_uart, RunScript* arg, EEpromV101* eepromv101, char* version);
int32_t setup_lidar_vpc(int32_t hCom, RunScript* arg);

bool readConfig(const char* cfg_file_name, RunScript& cfg);
bool checkPointsLengthZero(UserData *tmp, float scale);
void* lidar_thread_proc_udp(void* param);
void* lidar_thread_proc_uart(void* param);

#endif /// of #define __LIDARDATAPROCESS_H__
