#ifndef _LIDAR_DATA
#define  _LIDAR_DATA

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

#define CHECKSERVICE    6789            /// 检测本地雷达列表端口号
#define getbit(x,y)     ((x) >> (y)&1)
#define setbit(x,y)     x|=(1<<y)       /// 将X的第Y位置1
#define clrbit(x,y)     x&=~(1<<y)      /// 将X的第Y位清0
#define UNUSED(x)       (void)(x)

//CN：心跳检测包 EN：Heartbeat detection package
struct KeepAlive {
    uint32_t world_clock;   /// 时间戳
    uint32_t mcu_hz;        /// mcu频率(内部使用)
    uint32_t arrive;        /// 包数据主机到雷达的时间(内部使用)
    uint32_t delay;         /// 延迟(内部使用)
    uint32_t reserved[4];   /// 预留位
};

struct DataPoint
{
    float   angle;          /// CN:弧度      EN:radian
    float   distance;       /// CN:距离(米)  EN:distance(Meter)
    uint8_t confidence;     /// CN:强度      EN:strength
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
    pthread_mutex_t        datalock = PTHREAD_MUTEX_INITIALIZER;
    std::vector<DataPoint> data;
};
//最终返回的客户使用的雷达实时数据
struct UserData
{
    DataType type;
    int32_t idx;            /// 0表示扇区序号  1表示帧序号  超过10000000(1千万)帧回拨
    char connectArg1[16];   /// ip/com
    int32_t connectArg2;    /// port /baud
    //只有对应模式才有数据
    SpanData spandata;
    FrameData framedata;    
};
//报警包
struct LidarMsgHdr
{
    char sign[4];           /// must be "LMSG"
    uint32_t proto_version; /// 协议版本，当前为0x101
    char dev_sn[20];        /// 设备编号
    uint32_t dev_id;        /// 设备序号
    uint32_t timestamp;     /// 时间戳
    uint32_t flags;         /// 消息类型
    uint32_t events;        /// 消息内容的位组合
    uint16_t id;            /// 消息序号
    uint16_t extra;         ///（当前激活防区 + 设备各功能状态 + 保留）长度
    uint32_t zone_actived;  /// 当前激活防区（范围0~F）
    uint8_t all_states[32]; /// 设备各功能状态
    uint32_t reserved[11];  /// 保留
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
    int32_t RPM;    /// 转速    范围0450-1200   例如：LSRPM:0450H
    int32_t ERR;    /// 偏差      范围-99-+99 例如:LSERR:-23H  LSERR:+23H
    char UDP[64];   /// udp组合信息  设置雷达IP地址 子网掩码 网关 服务端口号，例如LSUDP:192.168.158.091 255.255.255.000 192.168.158.001 05000H
    char DST[64];   /// 接收雷达信息的ip地址和端口号  LSDST:192.168.158.043 12300H
    char NSP[32];   /// 机器类型    LSNSP:LDS-50C-S-UH
    char UID[32];   /// 机器序号     例如LSUID:201812030001H
    int32_t FIR;    /// 滤波圈数    (范围01~99) 例如：LSFIR:03H
    int32_t PUL;    /// 设置电机启动脉冲数   (范围0500~4000) 例如：LSPUL:2000H
    int32_t VER;    /// 设置版本号 例如：LSPUL:2000H
    int32_t PNP;    /// 设置IO类型  比如设置LSNPN:1H 输出IO类型为PNP
    int32_t SMT;    /// 数据平滑  LSSMT:1H  打开 LSSMT:0H  关闭
    int32_t DSW;    /// 去拖点  LSDSW:1H 打开 LSDSW:0H  关闭
    int32_t DID;    /// 设备ID   LSDID:xxxH
    int32_t ATS;    /// 开机自动上传  LSATS:xH    1/0   打开/关闭   
    int32_t TFX;    /// 固定上传地址  LSTFX:xH    1/0   打开/关闭   
    //另外
    int32_t PST;    /// 数据/报警上传类型  LSPST:xH    0:无 1:数据 2报警  3 数据+报警  
    int32_t AF;     /// 去拖点系数
    char set[18];   /// 需要设置的参数，1需要设置  0不设置  
    char result[35];/// 设置是否成功  1成功   0失败
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
    char label[4];          // "EPRM"
    uint16_t pp_ver; // 协议版本
    uint16_t size;          // 结构大小

    //uint32_t version;     // firmware version

    // device
    uint8_t dev_sn[20];//sn号
    uint8_t dev_type[16];//型号
    uint32_t dev_id;        // 编号

    // network
    uint8_t IPv4[4];
    uint8_t mask[4];
    uint8_t gateway[4];
    uint8_t srv_ip[4];//上传IP
    uint16_t srv_port;//上传端口
    uint16_t local_port;//本地端口

    uint16_t RPM;
    uint16_t RPM_pulse;//电机启动参数
    uint8_t fir_filter;//防区报警点数标准
    uint8_t cir;//防区报警圈数过滤标准
    uint16_t with_resample;

    uint8_t auto_start;//开机自动旋转
    uint8_t target_fixed;//固定上传
    uint8_t with_smooth;//数据平滑
    uint8_t with_filter;//去拖点

    //
    uint8_t ranger_bias[8];//零偏角错误修正
    uint32_t net_watchdog;//看门狗

    uint32_t pnp_flags;//PNP/NPN
    uint16_t deshadow;//平滑系数
    uint8_t zone_acted;//激活防区
    uint8_t should_post;//上传方式   0 无数据 1仅数据 2报警 3报警+数据

    uint8_t functions_map[16];//I/O输入输出口
    uint8_t reserved[36];
};

//CN:UDP设置雷达参数时接收使用报文头  EN:UDP uses the header when setting lidar parameters
struct CmdHeader
{
    uint16_t sign; //CN:与硬件约定的标志位                       EN:Flags consistent with hardware
    uint16_t cmd;    //CN:命令                                        EN：command
    uint16_t sn;    //CN:随机数，发送报文和接收报文时验证是否一致   EN:Random numbers, verify consistency when sending and receiving messages
    uint16_t len;   //CN:命令长度                                   EN:command length
};

//串口雷达状态包(STXXXXED) 中间四个字节
struct UartState
{
    //byte1
    bool unit_mm;//0 cm 1 mm
    bool with_conf;//0 close 1 open
    bool with_smooth;
    bool with_fitter;
    bool span_9;
    bool span_18;
    bool span_other;
    bool resampele;//重采样
    //byte2

    bool moter_turn;//0正向 1反向
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
    char sign[4];
    //版本号   2个字节
    uint16_t version;
    //ip地址  4个 字节
    uint8_t ip[4];
    //子网掩码  4个字节
    uint8_t mask[4];
    //网关地址  4个字节
    uint8_t gateway[4];
    //默认目标IP    4个字节
    uint8_t remote_ip[4];
    //默认目标udp端口号    2个字节
    uint16_t remote_udp;
    //默认UDP对外服务端口号  2个字节
    uint16_t port;
    //物体分辨率 1个字节
    uint8_t fir;
    //偏置    6个字节
    char zero_offset[6];
    //机器序号  20个字节
    char dev_sn[20];
    //机器类型  11个字节
    char dev_type[11];
    //IO类型选择    1个字节
    char io_type;
    //响应圈数  1个字节 
    uint8_t cir;
    //IO功能引脚配置  10个字节
    uint8_t io_mux[10];
};

struct DevInfo2
{
    // 标签   4个字节
    char sign[4];
    // 机器序号 20个字节
    char dev_sn[20];
    // 机器类型 11个字节
    char dev_type[12];
    //版本号   2个字节
    uint16_t version;
    // ip地址 4个 字节
    uint8_t ip[4];
    // 子网掩码 4个字节
    uint8_t mask[4];
    // 网关地址 4个字节
    uint8_t gateway[4];
    // 默认目标IP   4个字节
    uint8_t remote_ip[4];
    //默认目标udp端口号    2个字节
    uint16_t remote_udp;
    // 默认UDP对外服务端口号 2个字节
    uint16_t port;
    //保留    2个字节
    char reserver[2];
};

struct DevInfoV101
{
    char sign[4];  // must be "LiDA"
    uint32_t proto_version; // Ð­Òé°æ±¾ V101
    uint32_t timestamp[2];// Ê±¼ä´Á
    char dev_sn[20];
    char dev_type[16];
    uint32_t version;
    uint32_t dev_id; //Éè±¸±àºÅ
    uint8_t ip[4]; //Éè±¸µØÖ·
    uint8_t mask[4]; //
    uint8_t gateway[4];
    uint8_t remote_ip[4]; // ·þÎñÆ÷µØÖ·

    uint16_t remote_udp; //·þÎñÆ÷¶Ë¿Ú
    uint16_t port; // lidar service udp port
    uint16_t status;// Éè±¸×´Ì¬
    uint16_t rpm; // µ±Ç°×ªËÙ

    uint16_t freq; // µ±Ç°×ªËÙ
    uint8_t ranger_version[2];
    uint16_t CpuTemp;
    uint16_t InputVolt;
    uint8_t alarm[16]; // ±¨¾¯ÐÅÏ¢
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
    char com_port[128];
    int32_t com_speed;
    char conn_ip[32];
    int32_t conn_port;
    char timeStr[16];  //HH-MM-SS
    union {
        DevInfo v1;// info;
        DevInfoV101 v101;// info101;
        DevInfo2 v2;
    } info;
};

#endif
