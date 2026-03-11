#ifndef __STANDARD_INTERFACE_H__
#define __STANDARD_INTERFACE_H__

#include <LidarDataProcess.h>
#include <LidarService.h>

// SDK version numbers
#define SDKVERSION "3.2.2" 
#define SDKEXTRA "posix"

class BlueSeaLidarSDK
{
    public:
        static BlueSeaLidarSDK* getInstance();
        static void deleteInstance();

    public:
        /************************************************
        * @functionName:  getLidar
        * @date:          2022-08-21
        * @description:   Get current lidar info
        * @Parameter:
        *                 1.ID[int,IN]          lidar ID
        *
        * @return:        RunConfig  struct
        * @others:        Null
        *************************************************/
        RunConfig * GetLidarConfig(int ID);
        
        /************************************************
        * @functionName:  addLidarByPath
        * @date:          2022-07-14
        * @description:   CN:通过配置文件的路径添加ID  EN:add lidar by path
        * @Parameter:
        *                 1.cfg_file_name[const char*,IN]    config file path
        *
        * @return:        LIDAR ID
        * @others:        Null
        *************************************************/
        int AddLidarByPath(const char *cfg_file_name);
        
        /************************************************
        * @functionName:  delLidarByID
        * @date:          2022-07-14
        * @description:   Delete the specified lidar by the ID returned by addLidarByPath
        * @Parameter:
        *                 1.ID[int,IN]          Lidar ID
        *
        * @return:        true/false
        * @others:        Null
        *************************************************/
        bool DeleteLidarByID(int ID);
        
        /************************************************
        * @functionName:  setCallBackPtr
        * @date:          2022-07-14
        * @description:   Setting up a callback print function for the radar based on an ID
        * @Parameter:
        *                 1.ID[int,IN]          Lidar ID
        *                 1.ptr[printfMsg,IN]   Callback print function
        *
        * @return:        NUll
        * @others:        Null
        *************************************************/
        void SetCallBackPtr(int ID,printfMsg ptr);
        
        /************************************************
        * @functionName:  openDev
        * @date:          2022-03-28
        * @description:   Start lidar
        * @Parameter:
        *                 1.ID[int,IN]          Lidar ID
        *
        * @return:        true/false
        * @others:        Null
        *************************************************/
        bool OpenDev(int ID);
        
        /************************************************
        * @functionName:  StopDrv
        * @date:          2022-03-28
        * @description:   Stop lidar
        * @Parameter:
        *                 1.ID[int,IN]          Lidar ID
        *
        * @return:        Null
        * @others:        Null
        *************************************************/
        void StopDev(int ID);
        
        /************************************************
        * @functionName:  GetDevState
        * @date:          2026-03-02
        * @description:   Get device state
        * @Parameter:
        *                 1.ID[int,IN]          Lidar ID
        *
        * @return:        state value, ONLINE | OFFLINE
        * @others:        Null
        *************************************************/
        int  GetDevState(int ID);
        
        /************************************************
        * @functionName:  GetDevInfo
        * @date:          2023-08-21
        * @description:   Get lidar config arg
        * @Parameter:
        *                 1.ID[int,IN]                  Lidar ID
        *                 2.eepromv101[EEpromV101,OUT]  Lidar arg info
        *
        * @return:        true/false
        * @others:        Null
        *************************************************/
        bool GetDevInfo(int ID,EEpromV101 *eepromv101);

        /************************************************
         * @functionName:  ControlDrv
         * @date:          2023-08-21
         * @description:   Set the lidar on or off
         * @Parameter:    
         *                  1.ID[int,IN]       Lidar ID
         *                  2.num   [int,IN]   cmd length
         *                  3.cmd   [int,IN]   cmd  data
         * @return:         true/false
         * @others:        Null
         *************************************************/
        bool ControlDrv(int ID, int num, char *cmd);

        /************************************************
         * @functionName:  getVersion
         * @date:          2022-05-09
         * @description:   Get version
         * @Parameter:     Null
         * @return:        const char*
         * @others:        Null
         *************************************************/
        const char* GetVersion();
        
        /************************************************
         * @functionName:  ZoneSection
         * @date:          2022-10-18
         * @description:   Change  section
         * @Parameter:    1.ID [long,IN]        Lidar ID
         *                2.section [char,IN]   Active zone serial number
         * @return:        true/false
         * @others:        防区取值范围 0-9 a-f
         *************************************************/
        bool ZoneSection(int ID, char section);
        
        /************************************************
         * @functionName:  SetUDP
         * @date:          2023-07-31
         * @description:   Set lidar ip
         * @Parameter:      1.ID    [long,IN]   Lidar ID
         *                  2.ip    [char*,IN]  ip
         *                  3.mask  [char*,IN]  mask
         *                  4.gateway   [char*,IN]   gateway
         *                  5.port  [int,IN]    port
         * @return:        true/false
         * @others:        Null
         *************************************************/
        bool SetUDP(int ID, char* ip, char* mask, char* gateway, int port);
        
        /************************************************
         * @functionName:  SetDST
         * @date:          2023-07-31
         * @description:   Set lidar post ip  address
         * @Parameter:      1.ID [long,IN]   Lidar ID
         *                  2.ip[char*,IN]   ip
         *                  3.port[int,IN]   port
         * @return:        true/false
         * @others:        Null
         *************************************************/
        bool SetDST(int ID, char* ip, int port);
        
        /************************************************
         * @functionName:  SetRPM
         * @date:          2023-08-21
         * @description:   Set lidar rpm
         * @Parameter:      1.ID [long,IN]  Lidar ID
         *                  2.RPM[int,IN]   rpm
         * @return:        true/false
         * @others:        rpm   (300-3000)
         *************************************************/
        bool SetRPM(int ID,int RPM);
        
        /************************************************
         * @functionName:  SetTFX
         * @date:          2023-08-21
         * @description:   Set lidar fixed upload
         * @Parameter:      1.ID [long,IN]   Lidar ID
         *                  2.tfx[bool,IN]   true/false
         * @return:        true/false
         * @others:        Null
         *************************************************/
        bool SetTFX(int ID,bool tfx);
        
        /************************************************
         * @functionName:  SetDSW
         * @date:          2023-08-21
         * @description:   Set lidar filtering
         * @Parameter:      1.ID [long,IN]   Lidar ID
         *                  2.tfx[bool,IN]   true/false
         * @return:        true/false
         * @others:        Null
         *************************************************/
        bool SetDSW(int ID,bool dsw);
        
        /************************************************
         * @functionName:  SetSMT
         * @date:          2023-08-21
         * @description:   Set lidar smooth
         * @Parameter:      1.ID [long,IN]   Lidar ID
         *                  2.smt[bool,IN]   true/false
         * @return:        true/false
         * @others:        Null
         *************************************************/
        bool SetSMT(int ID,bool smt);
        
        /************************************************
         * @functionName:  SetPST
         * @date:          2023-08-21
         * @description:   Set lidar upload way
         * @Parameter:      1.ID [long,IN]  Lidar ID
         *                  2.mode[int,IN]  0 no data 1 only data 2 only alarm 3 data+alarm
         * @return:        true/false
         * @others:        Null
         *************************************************/
        bool SetPST(int ID,int mode);
        
        /************************************************
         * @functionName:  SetDID
         * @date:          2023-08-21
         * @description:   Set lidar number
         * @Parameter:      1.ID [long,IN]          Lidar ID
         *                  2.number[uint32_t,IN]   lidar number     
         * @return:        true/false
         * @others:        Repeatable, different from ID
         *************************************************/
        bool SetDID(int ID,uint32_t number);
        
        /************************************************
         * @functionName:  SetNTP
         * @date:          2025-01-17
         * @description:   Set lidar ntp property
         * @Parameter:      1.ID [long,IN]       Lidar ID
         *                  2.ntp_ip[char*,IN]   lidar ntp upload ip   
         *                  3.ntp_port[uint32_t,IN]   lidar ntp upload port     
         *                  4.enable[bool,IN]    enable    
         * @return:        true/false
         * @others:        Repeatable, different from ID
         *************************************************/
        bool SetNTP(int ID,char*ntp_ip,uint16_t port,bool enable);

        /************************************************
         * @functionName:  getLidarsList
         * @date:          2023-12-13
         * @description:   Get heartbeat packet data for networked Lidars and currently available serial Lidars
         * @return:        vector
         * @others:        The network heartbeat packet contains the received timestamp, which needs to be compared with the current timestamp to see if it is offline.
         *************************************************/
        std::vector<DevConnInfo> GetLidarsList();
            
    public:
        bool OpenHeartService();
        bool CloseHeartService();

    protected:
        bool closeService(int ID);
        // Configure radar parameters (stored after power failure)
        bool SetDevInfo(RunConfig *lidar,int length,char*cmd,int mode);

    private:
        static BlueSeaLidarSDK *m_sdk;
        BlueSeaLidarSDK();
        ~BlueSeaLidarSDK();

        int m_idx;
        std::vector<RunConfig*> m_lidars;

        LidarService*           m_service;
        bool m_checkflag;   /// Control heartbeat thread to shut down
};

#endif /// of __STANDARD_INTERFACE_H__
