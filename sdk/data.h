#ifndef __LIDAR_DATA_H__
#define __LIDAR_DATA_H__

/**
 * Copyright (C),  Pacecat:(C) <LanHai>,All right reserved
 * File name:      data.h
 * Author:         *    
 * Version:        1.0  
 * Date:           2022.3.25
 * Description:    Data storage structure and data parsing capabilities for all devices
 */

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

extern "C"
{
    #include <mongoose.h>
}

#ifndef PI
    #define PI          (3.1415926535898)
#endif /// of PI

#define MAX_LIDARS      8
#define MAX_FANS        100
#define MAX_POINTS      550
#define MAX_FRAMEPOINTS 10500
#define MAX_FRAMEIDX    10000000
#define HDR_SIZE        6
#define HDR2_SIZE       8
#define HDR3_SIZE       16 
#define HDR7_SIZE       28 
#define HDR99_SIZE      32 
#define HDRAA_SIZE      48

#define BUF_SIZE        (8*1024)
#define ZONE_SIZE       1024
#define SLEEP_SIZE      5
#define USER_SIZE       10

#define ZS_PACK         0x5A53
#define ZG_PACK         0x5A47

#define GO_PACK         0x474F
#define GT_PACK         0x4754
#define G0_PACK         0x4730
#define G1_PACK         0x4731
#define GF_PACK         0x4746

#define FH_PACK         0x4648
#define F_PACK          0x0046
#define PO_PACK         0x504F
#define P0_PACK         0x5030
#define PT_PACK         0x5054
#define P1_PACK         0x5031
#define GS_PACK         0x4753
#define S_PACK          0x0053
#define T_PACK          0x0054
#define C_PACK          0x0043

#define CHECKSERVICE    6789            /// Detect local radar list port number
#define getbit(x,y)     ((x) >> (y)&1)
#define setbit(x,y)     x|=(1<<y)       /// Position X at the Y-th position 1
#define clrbit(x,y)     x&=~(1<<y)      /// Clear the Y-th bit of X to 0.
#define UNUSED(x)       (void)(x)

// Heartbeat detection package
struct KeepAlive {
    uint32_t world_clock;   /// 时间戳
    uint32_t mcu_hz;        /// mcu频率(内部使用)
    uint32_t arrive;        /// 包数据主机到雷达的时间(内部使用)
    uint32_t delay;         /// 延迟(内部使用)
    uint32_t reserved[4];   /// 预留位
};

struct DataPoint
{
    float   angle;          /// radian
    float   distance;       /// distance(Meter)
    uint8_t confidence;     /// strength
};

struct RawData
{
    uint16_t code;      /// data frame header
    uint16_t N;         /// The number of ranging points in the sector
    uint16_t angle;     /// The starting angle of the current sector *10
    uint16_t span;      /// The total angle of the sector (sector end angle - sector start angle)
    uint16_t fbase;     /// Sector start offset（NULL）
    uint16_t first;     /// first point32_t angle（NULL）
    uint16_t last;      /// last point32_t angle（NULL）
    uint16_t fend;      /// Sector end offset（NULL）
    uint32_t flags;     /// 消息类型

    DataPoint points[MAX_POINTS];   /// The specific information of the scanning point32_t (the specific initialization number is determined by N)
    uint32_t ts[2];                 /// timestamps(Second and millisecond )
};

enum DataType
{
    SPANDATA=0,
    FRAMEDATA
};

struct SpanData
{
    RawData data;
};

struct FrameData
{
    uint32_t               ts[2];   /// timestamps(Second and microseconds )
    std::vector<DataPoint> data;
    pthread_mutex_t        datalock = PTHREAD_MUTEX_INITIALIZER;
    pthread_cond_t         datacond = PTHREAD_COND_INITIALIZER;
};

//The final returned real-time radar data used by the customer
struct UserData
{
    DataType type;
    int32_t idx;            /// 0 represents the sector number, 1 represents the frame number. More than 10,000,000 (10 million) frames require callback.
    char connectArg1[16];   /// ip/com
    int32_t connectArg2;    /// port /baud
    // Data is only available for the corresponding pattern.
    SpanData spandata;
    FrameData framedata;    
};

//alarm package
struct LidarMsgHdr
{
    char sign[4];           /// must be "LMSG"
    uint32_t proto_version; /// Protocol version, currently 0x101
    char dev_sn[20];        /// Device number
    uint32_t dev_id;        /// Device serial number (What dev_sn for ??? )
    uint32_t timestamp;     /// Timestamp
    uint32_t flags;         /// Message type
    uint32_t events;        /// Bit combination of message content
    uint16_t id;            /// Message serial number
    uint16_t extra;         /// Length of (current active zone + device functional states + reserved)
    uint32_t zone_actived;  /// Current active zone (range 0~F)
    uint8_t all_states[32]; /// Device functional states
    uint32_t reserved[11];  /// Reserved
};

//E100系列过滤点云数据
struct ShadowsFilterParam
{
    int32_t enable;
    double max_range;
    double min_angle, max_angle;
    int32_t window;
};

//E100系列过滤点云数据
struct MedianFilterParam
{
    int32_t enable;
    int32_t window;
};

//E330系列过滤离异点算法
struct SeparationFilterParam
{
    int32_t filter_open;
    float max_range;
    float min_range;
    float max_range_difference;
    int32_t filter_window;
};

/*
* RawDataHdr
* CN:代表不同的数据存储格式，每个设备仅使用一种格式，但是不同的设备可能使用同一种格式
* EN:Represents different data storage formats, each device uses only one format, but different devices may use the same format
*/

struct RawDataHdr
{
    uint16_t code; //CN:帧头              EN:data frame header
    uint16_t N;    //CN:扇区内测距点数    EN:The number of ranging points in the sector
    uint16_t angle;//CN:扇区起始角度      EN:Corresponding ranging angle
};

struct RawDataHdr2
{
    uint16_t code;  //CN:帧头             EN:data frame header
    uint16_t N;     //CN:扇区内测距点数   EN:The number of ranging points in the sector
    uint16_t angle; //CN:扇区起始角度     EN:Sector starting angle
    uint16_t span;  //CN:扇区大小         EN:sector size
};

struct RawDataHdr3
{
    uint16_t code;  //CN:帧头             EN:data frame header
    uint16_t N;     //CN:扇区内测距点数   EN:The number of ranging points in the sector
    uint16_t angle; //CN:扇区起始角度     EN:Corresponding ranging angle
    uint16_t span;  //CN:扇区大小         EN:sectors size
    uint16_t fbase; //CN:扇区起始偏差     EN:Sector start offset
    uint16_t first; //CN:第一个点角度     EN:first point32_t angle
    uint16_t last;  //CN:最后一个点角度   EN:last point32_t angle
    uint16_t fend;  //CN:扇区终止偏差     EN:Sector end offset
};

struct RawDataHdr7 {
    uint16_t code;      //CN:帧头                     EN:data frame header
    uint16_t N;         //CN:扇区内这个分包的测距点数 EN:The number of ranging points for this subpacket in the buffer
    uint16_t whole_fan; //CN:缓冲区内总测距点数       EN:The total number of ranging points in the sector
    uint16_t ofset;     //CN:扇区偏移量               EN:sector offset
    uint32_t beg_ang;   //CN:扇区起始角度             EN:Sector start angle
    uint32_t end_ang;   //CN:扇区终止角度             EN:Sector end angle
    uint32_t flags;     //CN:状态开关标识             EN:status switch flag
    uint32_t timestamp; //CN:时间戳                   EN:timestamp        (other:The data timestamps of different sectors in a frame are consistent)
    uint32_t dev_id;    //CN:设备号                   EN:device ID
};

struct FanSegment_C7
{
    RawDataHdr7 hdr;            //CN：Hdr7结构体      EN：Hdr7structure 

    uint16_t dist[MAX_POINTS];  //CN:距离             EN:distance
    uint16_t angle[MAX_POINTS]; //CN:角度             EN:angle
    uint8_t energy[MAX_POINTS]; //CN:能量强度         EN:energy intensity

    struct FanSegment_C7* next; // CN:下个扇区指针    EN:next sector pointer 
};
struct RawDataHdrAA {
    uint16_t code; // 0xFAAA
    uint16_t N;
    uint16_t whole_fan;
    uint16_t ofset;
    uint32_t beg_ang;
    uint32_t end_ang;
    uint32_t flags;
    uint32_t second;
    uint32_t nano_sec;
    uint32_t dev_id;
    uint32_t reserved[4];
};

struct FanSegment_AA
{
    RawDataHdrAA hdr;           //CN：HdrAA结构体     EN：Hdr7structure

    uint16_t dist[MAX_POINTS];  //CN:距离             EN:distance
    uint16_t angle[MAX_POINTS]; //CN:角度             EN:angle
    uint8_t energy[MAX_POINTS]; //CN:能量强度         EN:energy intensity

    struct FanSegment_AA* next; // CN:下个扇区指针    EN:next sector pointer
};

struct RawDataHdr99 {
    uint16_t code;          //CN:帧头，固定为0x99FA         EN:Frame header, fixed at 0x99FA
    uint16_t N;             //CN:扇区内这个分包的测距点数   EN:The number of ranging points for this subpacket in the sector
    uint16_t from;          //CN:扇区起始角度               EN:Sector start angle
    uint16_t total;         //CN:一整圈数据点个数           EN:The number of data points in a full circle
    uint32_t flags;         //CN:状态开关标识               EN:status switch flag
    uint32_t timestamp;     //CN:时间戳                     EN:timestamp 
    uint32_t dev_no;        //CN:设备编号                   EN:device ID
    uint32_t reserved[3];   //CN:保留位                     EN:reserved
};

//客户设置的雷达设备配置信息
struct DevData
{
    int32_t RPM;    /// Rotational speed Range 0450-1200 For example: LSRPM:0450H
    int32_t ERR;    /// Deviation Range -99-+99 For example: LSERR:-23H LSERR:+23H
    char UDP[64];   /// UDP combination information Set radar IP address Subnet mask Gateway Service port number, for example LSUDP:192.168.158.091 255.255.255.000 192.168.158.001 05000H
    char DST[64];   /// IP address and port number for receiving radar information LSDST:192.168.158.043 12300H
    char NSP[32];   /// Machine type LSNSP:LDS-50C-S-UH
    char UID[32];   /// Machine serial number, e.g., LSUID:201812030001H
    int32_t FIR;    /// Filtering cycles (range 01~99), e.g., LSFIR:03H
    int32_t PUL;    /// Set motor start pulse count (range 0500~4000), e.g., LSPUL:2000H
    int32_t VER;    /// Set version number, e.g., LSPUL:2000H
    int32_t PNP;    /// Set IO type, e.g., setting LSPPN:1H to PNP output IO type
    int32_t SMT;    /// Data smoothing, LSSMT:1H on, LSSMT:0H off
    int32_t DSW;    /// Remove drag points, LSDSW:1H on, LSDSW:0H off
    int32_t DID;    /// Device ID, LSDID:xxxH
    int32_t ATS;    /// Automatic upload on startup LSATS:xH 1/0 On/Off
    int32_t TFX;    /// Fixed upload address LSTFX:xH 1/0 On/Off
    //In addition
    int32_t PST;    /// Data/alarm upload type LSPST:xH 0: None 1: Data 2: Alarm 3: Data + Alarm  
    int32_t AF;     /// De-trapping coefficient
    char set[18];   /// Parameter to be set, 1 to be set 0 not set 
    char result[35];/// Whether the setting was successful 1 successful 0 failed
};

struct  DevTimestamp
{
    char ip[256];
    uint16_t port;
    uint32_t timestamp;
    uint32_t delay;
};

struct EEpromV101
{
    char     label[4];      /// "EPRM"
    uint16_t pp_ver;        /// 协议版本
    uint16_t size;          /// 结构大小

    // device
    uint8_t  dev_sn[20];     /// sn号
    uint8_t  dev_type[16];   /// 型号
    uint32_t dev_id;        /// 编号

    // network
    uint8_t  IPv4[4];
    uint8_t  mask[4];
    uint8_t  gateway[4];
    uint8_t  srv_ip[4];     /// 上传IP
    uint16_t srv_port;      /// 上传端口
    uint16_t local_port;    /// 本地端口

    uint16_t RPM;
    uint16_t RPM_pulse;     /// 电机启动参数
    uint8_t  fir_filter;    /// 防区报警点数标准
    uint8_t  cir;           /// 防区报警圈数过滤标准
    uint16_t with_resample;

    uint8_t auto_start;     /// 开机自动旋转
    uint8_t target_fixed;   /// 固定上传
    uint8_t with_smooth;    /// 数据平滑
    uint8_t with_filter;    /// 去拖点

    uint8_t  ranger_bias[8];/// 零偏角错误修正
    uint32_t net_watchdog;  /// 看门狗

    uint32_t pnp_flags;     /// PNP/NPN
    uint16_t deshadow;      /// 平滑系数
    uint8_t  zone_acted;    /// 激活防区
    uint8_t  should_post;   /// 上传方式, 0 无数据 1仅数据 2报警 3报警+数据

    uint8_t  functions_map[16]; /// I/O输入输出口
    uint8_t  reserved[36];
};

// UDP uses the header when setting lidar parameters
struct CmdHeader
{
    uint16_t sign;  /// Flags consistent with hardware
    uint16_t cmd;   /// command
    uint16_t sn;    /// Random numbers, verify consistency when sending and receiving messages
    uint16_t len;   /// command length
};

//串口雷达状态包(STXXXXED) 中间四个字节
struct UartState
{
    //byte1
    bool unit_mm;       /// 0 cm 1 mm
    bool with_conf;     /// 0 close 1 open
    bool with_smooth;
    bool with_fitter;
    bool span_9;
    bool span_18;
    bool span_other;
    bool resampele;     /// 重采样
    //byte2
    bool moter_turn;    /// 0正向 1反向
    bool span_8;
    bool span_16;
    //byte3
    bool byte3_error0;
    bool byte3_error1;
    bool byte3_error2;
    bool byte3_error3;
    //byte4
    bool byte4_error0;
    bool byte4_error1;
    bool byte4_error2;
    bool byte4_error3;
    bool byte4_error4;
    bool byte4_error5;
    bool byte4_error6;
    bool byte4_error7;
};

struct DevInfo
{
    //转速    2个字节
    uint16_t rpm;
    //启动脉冲个数    2个字节
    uint16_t pulse;
    //保留    4个字节
    char     sign[4];
    //版本号   2个字节
    uint16_t version;
    //ip地址  4个 字节
    uint8_t  ip[4];
    //子网掩码  4个字节
    uint8_t  mask[4];
    //网关地址  4个字节
    uint8_t  gateway[4];
    //默认目标IP    4个字节
    uint8_t  remote_ip[4];
    //默认目标udp端口号    2个字节
    uint16_t remote_udp;
    //默认UDP对外服务端口号  2个字节
    uint16_t port;
    //物体分辨率 1个字节
    uint8_t  fir;
    //偏置    6个字节
    char     zero_offset[6];
    //机器序号  20个字节
    char     dev_sn[20];
    //机器类型  11个字节
    char     dev_type[11];
    //IO类型选择    1个字节
    char     io_type;
    //响应圈数  1个字节 
    uint8_t  cir;
    //IO功能引脚配置  10个字节
    uint8_t  io_mux[10];
};

struct DevInfo2
{
    // 标签  4个字节
    char     sign[4];
    // 机器序号 20个字节
    char     dev_sn[20];
    // 机器类型 11个字节
    char     dev_type[12];
    //版本号   2个字节
    uint16_t version;
    // ip地址 4个 字节
    uint8_t  ip[4];
    // 子网掩码 4个字节
    uint8_t  mask[4];
    // 网关地址 4个字节
    uint8_t  gateway[4];
    // 默认目标IP   4个字节
    uint8_t  remote_ip[4];
    //默认目标udp端口号    2个字节
    uint16_t remote_udp;
    // 默认UDP对外服务端口号 2个字节
    uint16_t port;
    //保留    2个字节
    char     reserved[2];
};

struct DevInfoV101
{
    char     sign[4];  // must be "LiDA"
    uint32_t proto_version; // Ð­Òé°æ±¾ V101
    uint32_t timestamp[2];// Ê±¼ä´Á
    char     dev_sn[20];
    char     dev_type[16];
    uint32_t version;
    uint32_t dev_id; //Éè±¸±àºÅ
    uint8_t  ip[4]; //Éè±¸µØÖ·
    uint8_t  mask[4]; //
    uint8_t  gateway[4];
    uint8_t  remote_ip[4]; // ·þÎñÆ÷µØÖ·

    uint16_t remote_udp; //·þÎñÆ÷¶Ë¿Ú
    uint16_t port; // lidar service udp port
    uint16_t status;// Éè±¸×´Ì¬
    uint16_t rpm; // µ±Ç°×ªËÙ

    uint16_t freq; // µ±Ç°×ªËÙ
    uint8_t  ranger_version[2];
    uint16_t CpuTemp;
    uint16_t InputVolt;
    uint8_t  alarm[16]; // ±¨¾¯ÐÅÏ¢
    uint32_t crc;
};

//#pragma pack(pop)

enum ConnType
{
    TYPE_COM,
    TYPE_UDP_V1,
    TYPE_UDP_V2,
    TYPE_UDP_V101
};

struct DevConnInfo
{
    ConnType type;
    char     com_port[128];
    int32_t  com_speed;
    char     conn_ip[32];
    int32_t  conn_port;
    char     timeStr[16];  //HH-MM-SS
    union {
        DevInfo v1;// info;
        DevInfoV101 v101;// info101;
        DevInfo2 v2;
    } info;
};

#endif /// of __LIDAR_DATA_H__
