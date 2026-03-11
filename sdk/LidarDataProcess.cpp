#include "LidarDataProcess.h"
#include "error.h"
#include <pthread.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <iostream>

#ifdef __unix__
#include <sys/time.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <arpa/inet.h>
#include <stdarg.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <errno.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#endif

#define CAPI_UDPT_SPACK     CommunicationAPI::udp_talk_S_PACK
#define CAPI_UDPT_CPACK     CommunicationAPI::udp_talk_C_PACK
#define CAPI_VCPT           CommunicationAPI::vpc_talk
#define CAPI_SCMD_VPC       CommunicationAPI::send_cmd_vpc
#define CAPI_UARTT          CommunicationAPI::uart_talk
#define CAPI_SNDC_UDP       CommunicationAPI::send_cmd_udp
#define CAPI_UDPT_GSPACK    CommunicationAPI::udp_talk_GS_PACK

// these two functions was declared in Global.cpp.
extern int32_t _write( int fd, const char* d, size_t l );
extern int32_t _read( int fd, char* d, size_t l );

bool setup_lidar_udp( int fd_udp, RunScript *arg )
{
	char buf[32] = {0};
    
	if (arg->ats >= 0)
	{
		char tmp[3] = {0};
		if (CAPI_UDPT_SPACK(fd_udp, arg->connectArg, arg->connectArg2, 10, "LSATS:001H", tmp))
		{
#ifdef DEBUG
			printf("set LSATS:1H ,result:%s\n", tmp);
#endif /// of DEBUG
		}
	}
	// Initialize default start rotation
	if (arg->with_start >= 0)
	{
		if (CAPI_UDPT_CPACK(fd_udp, arg->connectArg, arg->connectArg2, 6, "LSTARH", 2, "OK", 0, NULL))
		{
#ifdef DEBUG
			printf("set LiDAR LSTARH  OK \n");
#endif /// of DEBUG
		}
	}
	// Hardware version number
	if (arg->version >= 0)
	{
		if (CAPI_UDPT_CPACK(fd_udp, arg->connectArg, arg->connectArg2, 6, "LVERSH", 14, "MOTOR VERSION:", 15, buf))
		{
#ifdef DEBUG
			printf("set LiDAR LXVERH  OK %.12s\n", buf);
#endif /// of DEBUG
		}
	}
    
	char result[3] = {0};
	if (arg->with_deshadow >= 0)
	{
		char cmd[12] = {0};
		snprintf(cmd, 12, "LSDSW:%dH", arg->with_deshadow);
		if (CAPI_UDPT_SPACK(fd_udp, arg->connectArg, arg->connectArg2, sizeof(cmd), cmd, result))
		{
#ifdef DEBUG
			printf("set LiDAR deshadow %s %s\n", cmd, result);
#endif  /// of DEBUG
		}
		else
		{
			PRTSTDERR("set LiDAR deshadow %s failure.\n", cmd);
		}
	}
    
	if (arg->with_smooth >= 0)
	{
		char cmd[12] = {0};
		snprintf(cmd, 12, "LSSMT:%dH", arg->with_smooth);
		if (CAPI_UDPT_SPACK(fd_udp, arg->connectArg, arg->connectArg2, 6, cmd, result))
		{
#ifdef DEBUG
			printf("set LiDAR with_smooth %s\n", result);
#endif /// of DEBUG
		}
		else
		{
			PRTSTDERR( "set LiDAR with_smooth, failure.\n");
		}
	}

	if (arg->ntp_enable >= 0)
	{
		bool ret = judgepcIPAddrIsValid(arg->ntp_ip);
		if (!ret)
		{
			PRTSTDERR( "ntp ip set failure!\n" );
		}
		else
		{
			char cmd[64];
			char ip_1[4];
			char ip_2[4];
			char ip_3[4];
			char ip_4[4];
			ip_1[3] = '\0';
			ip_2[3] = '\0';
			ip_3[3] = '\0';
			ip_4[3] = '\0';

			size_t idx[3];
			size_t index = 0;
			size_t ip_len = strlen(arg->ntp_ip);
			for (size_t i = 0; i < ip_len; i++)
			{
				if (arg->ntp_ip[i] == '.')
				{
					idx[index] = i;
					index++;
				}
			}
			memcpy(ip_1, &arg->ntp_ip[0], idx[0]);
			memcpy(ip_2, &arg->ntp_ip[idx[0] + 1], idx[1] - idx[0] - 1);
			memcpy(ip_3, &arg->ntp_ip[idx[1] + 1], idx[2] - idx[1] - 1);
			memcpy(ip_4, &arg->ntp_ip[idx[2] + 1], ip_len - idx[2]);
			snprintf(cmd, 64, \
                     "LSNTP:%d,%03d.%03d.%03d.%03d,%05dH",  \
                     arg->ntp_enable, \
                     atoi(ip_1), atoi(ip_2), atoi(ip_3), atoi(ip_4), \
                     arg->ntp_port);

			if (CAPI_UDPT_SPACK(fd_udp, arg->connectArg, arg->connectArg2, strlen(cmd), cmd, result))
			{
#ifdef DEBUG
				printf("set LiDAR ntp %s\n", result);
#endif /// of DEBUG
			}
			else
			{
				PRTSTDERR( "set LiDAR ntp failure.\n" );
			}
		}
	}

	if (arg->resample_res > 0)
	{
		// resample == 0  
        // Non-fixed angular resolution is not suitable for network packet calculation.
		if (arg->resample_res == 1 || (arg->resample_res > 100 && arg->resample_res <= 1500))
			snprintf(buf, 32, "LSRES:%03dH", arg->resample_res);
		else
			buf[0] = 0;

		if (buf[0])
		{
			if (CAPI_UDPT_SPACK(fd_udp, arg->connectArg, arg->connectArg2, strlen(buf), buf, result))
			{
#ifdef DEBUG
				printf("%s set LiDAR resample %d %s\n", buf, arg->resample_res, result);
#endif /// of DEBUG
			}
			else
			{
				PRTSTDERR( "%s set LiDAR resample %d %s\n", \
                           buf, arg->resample_res, result);
			}
		}
	}
    
	if (arg->rpm >= 0)
	{
		char cmd[16] = {0};
		snprintf(cmd, 16, "LSRPM:%dH", arg->rpm);
		if (CAPI_UDPT_SPACK(fd_udp, arg->connectArg, arg->connectArg2, strlen(cmd), cmd, result))
		{
#ifdef DEBUG
			printf("set RPM to %d  %s\n", arg->rpm, result);
#endif /// of DEBUG
		}
		else
		{
			PRTSTDERR( "set RPM to %d failure.\n", arg->rpm );
		}
	}
    
	if (arg->alarm_msg >= 0)
	{
		char cmd[12] = {0};
		snprintf(cmd, 12, "LSPST:%dH", arg->alarm_msg == 1 ? 3 : 1);
		if (CAPI_UDPT_SPACK(fd_udp, arg->connectArg, arg->connectArg2, sizeof(cmd), cmd, result))
		{
#ifdef DEBUG
			printf("set LiDAR %s %s\n", cmd, result);
#endif /// of DEBUG
		}
		else
		{
			PRTSTDERR( "set LiDAR should_post failure.\n" );
		}
	}
    
	if (arg->direction >= 0)
	{
		char cmd[12] = {0};
		snprintf(cmd, 12, "LSCCW:%dH", arg->direction);
		if (CAPI_UDPT_SPACK(fd_udp, arg->connectArg, arg->connectArg2, sizeof(cmd), cmd, result))
		{
#ifdef DEBUG            
			printf("set LiDAR %s %s\n", cmd, result);
#endif /// of DEBUG
		}
		else
		{
			PRTSTDERR( "set LiDAR Rotation direction failure.\n" );
		}
	}
    
	return true;
}

size_t strip( const char *s, char *buf )
{
	size_t len = 0;
	for (size_t i = 0; s[i] != 0; i++)
	{
		if (s[i] >= 'a' && s[i] <= 'z')
			buf[len++] = s[i];
		else if (s[i] >= 'A' && s[i] <= 'Z')
			buf[len++] = s[i];
		else if (s[i] >= '0' && s[i] <= '9')
			buf[len++] = s[i];
		else if (len > 0)
			break;
	}
	buf[len] = 0;
	return len;
}

void *lidar_thread_proc_uart(void *param)
{
	RunConfig *cfg = (RunConfig *)param;
    
    if ( cfg == nullptr )
    {
        PRTSTDERR( "Error at void *lidar_thread_proc_uart( %p );\n", 
                 param );
        pthread_exit( nullptr );
        return nullptr;
    }
    
	if (cfg->runscript.output_360)
		cfg->userdata.type = FRAMEDATA;
	else
		cfg->userdata.type = SPANDATA;
    
	strcpy(cfg->userdata.connectArg1, cfg->runscript.connectArg);
	cfg->userdata.connectArg2 = cfg->runscript.connectArg2;
	FanSegment_AA **fan_segs = new FanSegment_AA *;
	*fan_segs = nullptr;
	std::vector<RawData> whole_datas;
	int error_num = 0;
	int is_pack;
	int data_bytes = 3;
	int collect_angle = -1;
    int state = -1;
	std::string error;
	UartState uartstate;
	CmdHeader cmdheader;
	char result[512] = {0};
	char info[512] = {0};

	if (strcmp(cfg->runscript.type, "uart") == 0)
		setup_lidar_uart(cfg->fd, &cfg->runscript, &cfg->eepromv101, cfg->hardwareVersion);
	else
		setup_lidar_vpc(cfg->fd, &cfg->runscript);

	/*
	 * 4, read and parser data
	 */
	uint8_t *buf = new uint8_t[BUF_SIZE];
    
    if ( buf == nullptr )
    {
        PRTSTDERR( "read buffer allocation failure.\n" );
        return nullptr;
    }
    
	int buf_len = 0;
	int ret;
	struct timeval start_tv;
	gettimeofday(&start_tv, nullptr);

	while ( cfg->state != STOP_ALL )
	{
		fd_set fds;
		FD_ZERO(&fds);
		FD_SET(cfg->fd, &fds);
		struct timeval to = {1, 1};
		ret = select(cfg->fd + 1, &fds, nullptr, nullptr, &to);
		if (ret < 0)
		{
			PRTSTDERR( "select(%d) failure.\n", cfg->fd );
			return nullptr;
		}
        
		if (cfg->fd > 0 && FD_ISSET(cfg->fd, &fds))
		{
			int nr = _read(cfg->fd, buf + buf_len, BUF_SIZE - buf_len);
			if (nr < 0)
			{
				snprintf(info, 512, "read port %d error %d", buf_len, nr);
				cfg->callback(9, info, strlen(info) + 1);
				break;
			}
            
			if (nr == 0)
			{
				msleep(10);
			}
            
			if (nr > 0)
			{
				buf_len += nr;
			}
		}

		if (buf_len > 0)
		{
            // in order to compute the rest of data after every parser process
			int consume = 0; 
			RawData dat;
			if (data_bytes == 3)
			{
				is_pack = ParseAPI::parse_data_x(buf_len, buf, &uartstate, dat, \
                                                 consume, true, data_bytes, \
                                                 result, 512, &cmdheader, \
                                                 (void **)fan_segs);
			}
			else
			{
				is_pack = ParseAPI::parse_data(buf_len, buf, &uartstate, dat, consume, true);
			}
            
			switch (is_pack)
			{
                case 1:
                {
                    if (collect_angle == -1)
                    {
                        // Obtain the current radar's initial statistical angle.
                        if (state < 0)
                        {
                            state = UserAPI::autoGetFirstAngle(dat, cfg->runscript.from_zero, whole_datas, error);
                        }
                        if (state >= 0)
                        {
                            collect_angle = state;
                            cfg->action = ONLINE;
                            snprintf(info, 512, \
                                     "Lidar start work,first span angle is %d",\
                                     collect_angle / 10);
                            cfg->callback(8, info, strlen(info) + 1);
                        }
                    }
                    else
                    if (cfg->runscript.output_360)
                    {
                        ret = UserAPI::whole_data_process(dat, collect_angle, whole_datas, error);
                                                
                        if (cfg->action >= RUN && ret == -1)
                        {
                            timeval tv;
                            gettimeofday(&tv, NULL);
                            if (tv.tv_sec - start_tv.tv_sec > 2)
                            {
                                snprintf(info, 512, \
                                         "%ld %ld span err  code:%d value:%s ",\
                                         tv.tv_sec, tv.tv_usec, ret, \
                                         error.c_str());
                                cfg->callback(9, info, strlen(info) + 1);
                                
                                if ( error.size() > 0 )
                                    error.clear();
                            }
                        }
                        
                        if (ret == 1)
                        {
                            cfg->userdata.framedata.data.clear();
                            for (std::size_t i = 0; i < whole_datas.size(); i++)
                            {
                                for (int j = 0; j < whole_datas.at(i).N; j++)
                                {
                                    cfg->userdata.framedata.data.push_back(whole_datas.at(i).points[j]);
                                }
                            }
                            // 执行回调函数
                            if (cfg->userdata.framedata.data.size() > 0)
                            {
                                if (cfg->runscript.shadows_filter.enable)
                                {
                                    AlgorithmAPI::ShadowsFilter(&cfg->userdata, cfg->runscript.shadows_filter);
                                }
                                if (cfg->runscript.median_filter.enable)
                                {
                                    AlgorithmAPI::MedianFilter(&cfg->userdata, cfg->runscript.median_filter);
                                }
                                if (checkPointsLengthZero(&cfg->userdata, cfg->runscript.error_scale))
                                    error_num++;
                                else
                                    error_num = 0;
                                if (cfg->runscript.error_circle <= error_num)
                                {
                                    snprintf(info, 512,\
                                             "%s %d There are many points with a distance of 0 in the current lidar operation", \
                                             cfg->runscript.connectArg, cfg->runscript.connectArg2);
                                    cfg->callback(3, info, strlen(info) + 1);
                                    error_num = 0;
                                }
                            }
                            if (cfg->runscript.separation_filter.filter_open)
                            {
                                double angle_increment = 2 * PI / cfg->userdata.framedata.data.size();
                                AlgorithmAPI::filter(cfg->userdata.framedata.data,
                                                     cfg->runscript.separation_filter.max_range,
                                                     cfg->runscript.separation_filter.min_range,
                                                     cfg->runscript.separation_filter.max_range_difference,
                                                     cfg->runscript.separation_filter.filter_window, angle_increment);
                            }
                            cfg->userdata.idx++;
                            if (strcmp(cfg->runscript.type, "uart") == 0)
                            {
                                struct timeval tv;
                                gettimeofday(&tv, NULL);
                                cfg->userdata.framedata.ts[0] = tv.tv_sec;
                                cfg->userdata.framedata.ts[1] = tv.tv_usec;
                            }
                            else if (strcmp(cfg->runscript.type, "vpc") == 0)
                            {
                                RawData data = whole_datas.at(whole_datas.size() - 1);
                                cfg->userdata.framedata.ts[0] = data.ts[0];
                                cfg->userdata.framedata.ts[1] = data.ts[1];
                            }
                            whole_datas.clear();
                            cfg->action = RUN;
                        }
                        else
                            break;
                    }
                    // 单独扇区输出
                    else
                    {
                        cfg->userdata.idx++;
                        memcpy(&cfg->userdata.spandata.data, &dat, sizeof(RawData));
                        if (strcmp(cfg->runscript.type, "uart") == 0)
                        {
                            struct timeval tv;
                            gettimeofday(&tv, NULL);
                            cfg->userdata.spandata.data.ts[0] = tv.tv_sec;
                            cfg->userdata.spandata.data.ts[1] = tv.tv_usec;
                        }
                        else if (strcmp(cfg->runscript.type, "vpc") == 0)
                        {
                            // Network-enabled sectors have built-in timestamps
                        }
                        cfg->action = RUN;
                    }

                    ((void (*)(int, void *))cfg->callback)(1, &cfg->userdata);
                    // Avoid cumulative out-of-bounds
                    if (cfg->userdata.idx >= MAX_FRAMEIDX)
                        cfg->userdata.idx = 0;
                }break;
                
                case 2:
                {
                    // Alarm information
                    memcpy(&cfg->zonemsg, &result, sizeof(LidarMsgHdr));
                    ((void (*)(int, void *))cfg->callback)(2, &result);
                    cfg->action = FINISH;
                }break;
                
                case 3:
                {
                    // global parameters
                    memcpy(&cfg->eepromv101, &result, sizeof(EEpromV101));
                    ((void (*)(int, void *))cfg->callback)(3, &result);
                    cfg->action = FINISH;
                }break;
                
                case 4: /// Response returned by time synchronization
                    break;
                
                case 5: /// C_PACK
                    cfg->action = FINISH;
                    break;
                
                case 6: /// S_PACK
                    cfg->action = FINISH;
                    break;
                
                case 7: /// readzone
                    break;
                
                case 8: /// writezone
                    break;
                
                case 9:
                {
                    // 串口每圈头发送的状态信息
                    // uartstate.with_fitter,uartstate.with_smooth
                }break;
                
			} /// of switch() ..
            
			if (consume > 0)
			{
				// data is not whole fan,drop it
				if (!is_pack)
				{
#ifdef DEBUG
					printf("drop %d bytes: %02x %02x %02x %02x %02x %02x",
						   consume,
						   buf[0], buf[1], buf[2],
						   buf[3], buf[4], buf[5]);
#endif /// of DEBUG
				}

				for (int i = consume; i < buf_len; i++)
					buf[i - consume] = buf[i];
				buf_len -= consume;
			}
		}
        
		// There are instructions that need to be performed.
		switch (cfg->action)
		{
            case CONTROL:
            {
                if (strcmp(cfg->runscript.type, "uart") == 0)
                {
                    _write(cfg->fd, cfg->send_cmd, cfg->send_len);
                }
                else if (strcmp(cfg->runscript.type, "vpc") == 0)
                {
                    CAPI_SCMD_VPC(cfg->fd, 0x0043, rand(), cfg->send_len, cfg->send_cmd);
                }

                cfg->action = FINISH;
            }break;
            
            case GETALLPARAMS:
            {
                char buf[20] = {0};
                if (strcmp(cfg->runscript.type, "uart") == 0)
                {
                    if (CAPI_UARTT(cfg->fd, 6, "LUUIDH", 11, "PRODUCT SN:", 16, buf))
                    {
#ifdef DEBUG
                        printf("get LiDAR uuid info:  %s\n", buf);
#endif /// of DEBUG
                        std::string str = BaseAPI::stringfilter(buf, 16);
                        memcpy(cfg->eepromv101.dev_sn, str.c_str(), str.length());
                    }
                }
                else if (strcmp(cfg->runscript.type, "vpc") == 0)
                {
                    if (!CAPI_VCPT(cfg->fd, 0x4753, rand(), 6, "LUUIDH", sizeof(EEpromV101), &cfg->eepromv101))
                    {
                        PRTSTDERR( "vpc GetDevInfo_MSG failure.\n" );
                        strcpy(cfg->recv_cmd, "NG");
                    }
                    else
                        strcpy(cfg->recv_cmd, "OK");
                }
                cfg->action = FINISH;                
            }break;

            case SETPARAM:
            {
                if (strcmp(cfg->runscript.type, "vpc") == 0)
                {
                    cfg->recv_len = 2;
                    if (CAPI_VCPT(cfg->fd, 0x0053, rand(), cfg->send_len, cfg->send_cmd, cfg->recv_len, cfg->recv_cmd))
                    {
#ifdef DEBUG
                        printf("cmd:%s  recv: %d %s \n", cfg->send_cmd, cfg->recv_len, cfg->recv_cmd);
#endif /// of DEBUG
                        strcpy(cfg->recv_cmd, "OK");
                    }
                    else
                        strcpy(cfg->recv_cmd, "NG");
                }
                else if (strcmp(cfg->runscript.type, "uart") == 0)
                {
                    if (CAPI_UARTT(cfg->fd, cfg->send_len, cfg->send_cmd, 2, "OK", 0, NULL))
                    {
#ifdef DEBUG
                        printf("set %s OK\n", cfg->send_cmd);
#endif /// of DEBUG
                        strcpy(cfg->recv_cmd, "OK");
                    }
                    else
                        strcpy(cfg->recv_cmd, "NG");
                }

                cfg->action = FINISH;
            }break;

            case READZONE:
                break;

            case WRITEZONE:
                break;
            
            case ONLINE: /// This indicates that it is running normally.
                break;
            
            default:
                break;
        } /// of switch()
	} 
	
    if ( buf != nullptr )
    {
        delete[] buf;
        buf = nullptr;
    }
    
    SystemAPI::closefd(cfg->fd, false);
    
    pthread_exit( NULL );
	return NULL;
}

int setup_lidar_vpc(int hCom, RunScript *arg)
{
	char buf[64] = {0};
	int nr = 0;

	if (arg->ats >= 0)
	{
		char tmp[3] = {0};
		if (CAPI_VCPT(hCom, 0x0053, rand(), 10, "LSATS:002H", 3, tmp))
		{
#ifdef DEBUG
			printf("set LSATS:2H ,result:%s\n", tmp);
#endif /// of DEBUG
		}
	}
    
	if (arg->with_start >= 0)
	{
		if (CAPI_VCPT(hCom, 0x0043, rand(), 6, "LSTARH", 3, buf))
		{
#ifdef DEBUG
			printf("set LSTARH ,result:%s\n", buf);
#endif /// of DEBUG
		}
	}

	for (size_t i = 0; i < 300 && nr <= 0; i++)
	{
		msleep(10);
		nr = _read(hCom, buf, sizeof(buf));
	}
    
	if (nr <= 0)
	{
#ifdef DEBUG
		printf("serial port is not availed.\n");
#endif /// of DEBUG
		return -1;
	}
    
	// 硬件版本号
	if (arg->version >= 0)
	{
		if (CAPI_VCPT(hCom, 0x0043, rand(), 6, "LXVERH", 64, buf))
		{
#ifdef DEBUG
			printf("set LiDAR LXVERH  OK  %.16s\n", buf);
#endif /// of DEBUG
		}
	}

	if (arg->uuid >= 0)
	{
		if (CAPI_VCPT(hCom, 0x0043, rand(), 6, "LUUIDH", 32, buf))
		{
#ifdef DEBUG
			printf("set LiDAR LUUIDH  OK  %.16s\n", buf + 10);
#endif /// of DEBUG
		}
	}
    
	if (arg->with_deshadow >= 0)
	{
		if (CAPI_VCPT(hCom, 0x0043, rand(), 6, arg->with_deshadow == 0 ? "LFFF0H" : "LFFF1H", 3, buf))
		{
#ifdef DEBUG            
			printf("set deshadow to %d,result:%s\n", arg->with_deshadow, buf);
#endif /// of DEBUG
		}
	}
    
	if (arg->with_smooth >= 0)
	{
		if (CAPI_VCPT(hCom, 0x0043, rand(), 6, arg->with_smooth == 0 ? "LSSS0H" : "LSSS1H", 3, buf))
		{
#ifdef DEBUG
			printf("set smooth to %d,result:%s\n", arg->with_smooth, buf);
#endif /// of DEBUG
		}
	}

	if (arg->resample_res >= 0)
	{
		char cmd[32];
		snprintf(cmd, 32, "LSRES:%03dH", arg->resample_res);
		if (CAPI_VCPT(hCom, 0x0053, rand(), sizeof(cmd), cmd, 3, buf))
		{
#ifdef DEBUG
			printf("set LiDAR resample to %d,result:%s\n", arg->resample_res, buf);
#endif /// of DEBUG
		}
	}
    
	if (arg->rpm >= 0)
	{
		if (arg->rpm > 300 && arg->rpm <= 3000)
		{
			for (int i = 0; i < 5; i++)
			{
				char cmd[32];
				snprintf(cmd, 32, "LSRPM:%dH", arg->rpm);
				if (CAPI_VCPT(hCom, 0x0043, rand(), strlen(cmd), cmd, 3, buf))
				{
#ifdef DEBUG
					printf("set RPM to %s,,result:%s\n", cmd, buf);
#endif /// of DEBUG
					break;
				}
			}
		}
	}
    
	return 0;
}

int setup_lidar_uart(int fd_uart, RunScript *arg, EEpromV101 *eepromv101, char *version)
{
	char buf[32];
	int index = 3;
	int nr = 0;

	if (arg->with_start >= 0)
	{
		_write(fd_uart, "LSTARH", 6);
	}
	for (int i = 0; i < 300 && nr <= 0; i++)
	{
		msleep(10);
		nr = _read(fd_uart, buf, sizeof(buf));
	}
	if (nr <= 0)
	{
		PRTSTDERR( "serial port seem not working\n" );
		SystemAPI::closefd(fd_uart, false);
		return READ_UART_FAILED;
	}
	if (arg->uuid >= 0)
	{
		for (int i = 0; i < index; i++)
		{
			if (CAPI_UARTT(fd_uart, 6, "LUUIDH", 11, "PRODUCT SN:", 12, buf))
			{

				strcpy((char *)eepromv101->dev_sn, buf);
#ifdef DEBUG
				printf("get LiDAR uuid info:  %s\n", eepromv101->dev_sn);
#endif /// of DEBUG
				break;
			}
		}
	}
	// 硬件版本号
	if (arg->version >= 0)
	{
		for (int i = 0; i < index; i++)
		{
			if (CAPI_UARTT(fd_uart, 6, "LXVERH", 14, "MOTOR VERSION:", 12, buf))
			{
#ifdef DEBUG
				printf("get LiDAR version info:  %.12s\n", buf);
#endif /// of DEBUG
				strcpy(version, buf);
				break;
			}
		}
	}
	if (arg->model >= 0)
	{
		for (int i = 0; i < index; i++)
		{
			if (CAPI_UARTT(fd_uart, 6, "LTYPEH", 8, "TYPE ID:", 16, buf))
			{
				std::string tmp = BaseAPI::stringfilter(buf, 16);
				memcpy((char *)eepromv101->dev_type, tmp.c_str(), tmp.length());
#ifdef DEBUG
				printf("set LiDAR LTYPEH2  %s \n", eepromv101->dev_type);
#endif /// of DEBUG
				break;
			}
		}
	}
	// Set the lidar returned data unit   CM or MM
	if (arg->unit_is_mm >= 0)
	{
		for (int i = 0; i < index; i++)
		{
			if (CAPI_UARTT(fd_uart, 6, arg->unit_is_mm == 0 ? "LMDCMH" : "LMDMMH", 6, "LiDAR ", 12, buf))
			{
#ifdef DEBUG
				printf("set LiDAR unit %s\n", buf);
#endif /// of DEBUG
				break;
			}
		}
	}
	// set lidar confidence state   LNCONH close   LOCONH open
	if (arg->with_confidence >= 0)
	{
		for (int i = 0; i < index; i++)
		{
			if (CAPI_UARTT(fd_uart, 6, arg->with_confidence == 0 ? "LNCONH" : "LOCONH", 6, "LiDAR ", 12, buf))
			{
#ifdef DEBUG
				printf("set LiDAR confidence %.02s\n", buf);
#endif /// of DEBUG
				break;
			}
		}
	}
	// set  de-deshadow state    LFFF0H:close  LFFF1H:open
	if (arg->with_deshadow >= 0)
	{
		for (int i = 0; i < index; i++)
		{
			if (CAPI_UARTT(fd_uart, 6, arg->with_deshadow == 0 ? "LFFF0H" : "LFFF1H", 6, "LiDAR ", 12, buf))
			{
#ifdef DEBUG
				printf("set deshadow %.02s\n", buf);
#endif /// of DEBUG
				break;
			}
		}
	}
	// set  de-smooth     LSSS0H:close   LSSS1H:open
	if (arg->with_smooth >= 0)
	{
		for (int i = 0; i < index; i++)
		{
			if (CAPI_UARTT(fd_uart, 6, arg->with_smooth == 0 ? "LSSS0H" : "LSSS1H", 6, "LiDAR ", 12, buf))
			{
#ifdef DEBUG
				printf("set smooth %.02s\n", buf);
#endif /// of DEBUG
				break;
			}
		}
	}
	// LSRES:000H :set default Angular resolution  LSRES:001H :fix Angular resolution
	if (arg->resample_res >= 0)
	{
		char cmd[16];
		snprintf(cmd, 16, "LSRES:%dH", arg->resample_res);
		for (int i = 0; i < index; i++)
		{
			if (CAPI_UARTT(fd_uart, strlen(cmd), cmd, 15, "set resolution ", 12, buf))
			{
#ifdef DEBUG
				printf("set LiDAR resample %.02s\n", buf);
#endif /// of DEBUG
				break;
			}
		}
	}

	// setup rpm  (The specific model range is different)
	if (arg->rpm >= 0)
	{
		char cmd[16] = {0};
		snprintf(cmd, 16, "LSRPM:%dH", arg->rpm);
		for (int i = 0; i < index; i++)
		{
			if (CAPI_UARTT(fd_uart, strlen(cmd), cmd, 8, "Set RPM:", 12, buf))
			{
#ifdef DEBUG
				printf("set RPM to %d  %.02s\n", arg->rpm, buf);
#endif /// of DEBUG
				break;
			}
		}
	}
	return 0;
}

void* lidar_thread_proc_udp(void *param)
{
	RunConfig *cfg = (RunConfig *)param;
    
	if (cfg->runscript.output_360)
		cfg->userdata.type = FRAMEDATA;
	else
		cfg->userdata.type = SPANDATA;

	strcpy(cfg->userdata.connectArg1, cfg->runscript.connectArg);
	cfg->userdata.connectArg2 = cfg->runscript.connectArg2;

	FanSegment_AA **fan_segs = new FanSegment_AA *;
	*fan_segs = nullptr;
	std::vector<RawData> whole_datas;
	int error_num       = 0;
	int is_pack         = 0;
	int collect_angle   = -1;
    int state           = -1;
	int data_bytes      = 3;
	std::string error;
	UartState uartstate;
	CmdHeader cmdheader;
	char result[512] = {0};
	char info[512] = {0};
	if (cfg->runscript.is_group_listener == 1)
	{
		ip_mreq group;
		memset(&group, 0, sizeof(group));
		group.imr_multiaddr.s_addr = inet_addr(cfg->runscript.group_ip);
		group.imr_interface.s_addr = INADDR_ANY;

		int rt = setsockopt(cfg->fd, IPPROTO_IP,
							IP_ADD_MEMBERSHIP, (char *)&group,
							sizeof(group));

		if (rt < 0)
		{
			snprintf( info, 512, \
                      "Adding to multicast group %s %s", \
                      cfg->runscript.group_ip, rt < 0 ? "fail!" : "ok");
			cfg->callback(9, info, strlen(info) + 1);
            
            if ( fan_segs != nullptr )
            {
                delete fan_segs;
                fan_segs = nullptr;
            }
            
			return NULL;
		}
        
		snprintf(info, 512, "Adding to multicast group success");
		cfg->callback(8, info, strlen(info) + 1);
	}
	else
	{
		setup_lidar_udp(cfg->fd, &cfg->runscript);
	}

	uint8_t *buf = new uint8_t[BUF_SIZE];
    
    if ( buf == nullptr )
    {
        PRTSTDERR( "read buffer allocation failure.\n" );
        
        if ( fan_segs != nullptr )
        {
            delete fan_segs;
            fan_segs = nullptr;
        }
        return nullptr;
    }
        
	struct timeval tv;
    struct timeval start_tv;
	gettimeofday(&tv, NULL);
	gettimeofday(&start_tv, NULL);
	time_t tto = tv.tv_sec + 1;
	uint32_t delay = 0;

    fd_set fds;    
    FD_ZERO(&fds);
    FD_SET(cfg->fd, &fds);
 
    struct timeval to = {1, 0};
 
	while ( cfg->state != STOP_ALL )
	{
        int ret_sel = -1;
        
		if (cfg->runscript.is_group_listener != 1)
		{
			ret_sel = select(cfg->fd + 1, &fds, NULL, NULL, &to);
            
			gettimeofday(&tv, NULL);
			if (tv.tv_sec > tto)
			{
				KeepAlive alive;
				gettimeofday(&tv, NULL);
				alive.world_clock = (tv.tv_sec % 3600) * 1000 + tv.tv_usec / 1000;
				alive.delay = delay;

				// acknowlege device ?
				CAPI_SNDC_UDP( cfg->fd, \
                               cfg->runscript.connectArg, \
                               cfg->runscript.connectArg2, \
                               0x4b41, 
                               rand(), 
                               sizeof(alive), 
                               &alive );

				tto = tv.tv_sec + 1;
				DevTimestamp devtimestamp;
				memcpy(devtimestamp.ip, cfg->runscript.connectArg, sizeof(cfg->runscript.connectArg));
				devtimestamp.port = cfg->runscript.connectArg2;
				devtimestamp.timestamp = alive.world_clock;
				devtimestamp.delay = delay;
                
				cfg->callback(4, &devtimestamp, sizeof(DevTimestamp));
			}
            
			if (ret_sel < 0)
			{
				snprintf(info, 512, "socket select(); failure.\n");
				cfg->callback(9, info, strlen(info) + 1);
				break;
			}
            
		} /// of if (cfg->runscript.is_group_listener != 1)
            
		// read UDP data
        if ( ret_sel > 0 )
		{
			sockaddr_in addr;
			socklen_t sz = sizeof(addr);
            
			int buf_len = recvfrom( cfg->fd, \
                                    (char *)buf, BUF_SIZE, \
                                    0, (struct sockaddr *)&addr, &sz );
                                    
            bool ipcompared = false;
            if (strcmp(cfg->runscript.connectArg, (char*)inet_ntoa(addr.sin_addr)) == 0)
            {
                ipcompared = true;
            }

            if ( buf_len > 0 && ipcompared == true )
            {
                // in order to compute the rest of data after every parser process
                int consume = 0; 
                
                RawData dat;
                                
                if ( data_bytes == 3 )
                {
                    is_pack = \
                    ParseAPI::parse_data_x( buf_len, buf, \
                                            &uartstate, dat, consume, \
                                            true, data_bytes, result, 512, \
                                            &cmdheader, (void **)fan_segs);
                }
                else 
                if ( data_bytes == 2 )
                {
                    is_pack = \
                    ParseAPI::parse_data( buf_len, buf, \
                                          &uartstate, dat, consume, true );
                }

                switch ( is_pack )
                {
                    case 1:
                    {
                        if (collect_angle == -1)
                        {
                            // Obtain the current radar's initial statistical angle.
                            if (state < 0)
                            {
                                state = \
                                UserAPI::autoGetFirstAngle( dat, \
                                                            cfg->runscript.from_zero, \
                                                            whole_datas, error );
                            }
                            
                            if (state >= 0)
                            {
                                whole_datas.clear();
                                collect_angle = state;
                                cfg->action = ONLINE;
                                
                                snprintf( info, 512, \
                                          "Lidar start work,first span angle is %d", \
                                          collect_angle / 10 );
                                          
                                cfg->callback(8, info, strlen(info) + 1);
                            }
                            break;
                        }
                        
                        if (cfg->runscript.output_360)
                        {
                            int ret = UserAPI::whole_data_process(dat, collect_angle, whole_datas, error);
                                                        
                            if (cfg->action >= RUN && ret == -1)
                            {
                                timeval tmp_tv;
                                gettimeofday(&tmp_tv, NULL);
                                if (tmp_tv.tv_sec - start_tv.tv_sec > 2)
                                {
                                    snprintf(info, 512, \
                                             "%ld %ld span err  code:%d value:%s ", \
                                             tv.tv_sec, tv.tv_usec, ret, error.c_str());
                                    cfg->callback(9, info, strlen(info) + 1);
                                }
                                
                                if ( error.size() > 0 )
                                    error.clear();
                            }
                            
                            if (ret == 1)
                            {
                                cfg->userdata.framedata.data.clear();
                                
                                for (size_t i = 0; i < whole_datas.size(); i++)
                                {
                                    for (int j = 0; j < whole_datas.at(i).N; j++)
                                    {
                                        cfg->userdata.framedata.data.push_back(whole_datas.at(i).points[j]);
                                    }
                                }
                                
                                if (cfg->userdata.framedata.data.size() > 0)
                                {
                                    if (checkPointsLengthZero(&cfg->userdata, cfg->runscript.error_scale))
                                        error_num++;
                                    else
                                        error_num = 0;
                                    if (cfg->runscript.error_circle <= error_num)
                                    {
                                        snprintf(info, 512, \
                                                 "%s %d Too many points with a distance of 0", \
                                                 cfg->runscript.connectArg, 
                                                 cfg->runscript.connectArg2);
                                        cfg->callback(9, info, strlen(info) + 1);
                                        error_num = 0;
                                    }
                                }
                                
                                if (cfg->runscript.separation_filter.filter_open)
                                {
                                    double angle_increment = 2 * PI / cfg->userdata.framedata.data.size();
                                    AlgorithmAPI::filter(cfg->userdata.framedata.data,
                                                         cfg->runscript.separation_filter.max_range,
                                                         cfg->runscript.separation_filter.min_range,
                                                         cfg->runscript.separation_filter.max_range_difference,
                                                         cfg->runscript.separation_filter.filter_window, angle_increment);
                                }
                                
                                cfg->userdata.idx++;
                                RawData data = whole_datas[0];
                                cfg->userdata.framedata.ts[0] = data.ts[0];
                                cfg->userdata.framedata.ts[1] = data.ts[1];
                                whole_datas.clear();
                                
                                cfg->action = RUN;
                                cfg->callback(1, &cfg->userdata, sizeof(UserData));
                            }
                        }
                        else /// Individual sector output
                        {
                            cfg->userdata.idx++;
                            memcpy(&cfg->userdata.spandata.data, &dat, sizeof(RawData));
                            cfg->action = RUN;
                            cfg->callback(1, &cfg->userdata, sizeof(UserData));
                        }

                        // Avoid cumulative out-of-bounds
                        if (cfg->userdata.idx >= MAX_FRAMEIDX)
                            cfg->userdata.idx = 0;
                    }break;
                    
                    case 2:
                    {
                        // Alarm information
                        if ( result[0] != 0 )
                        {
                            memcpy(&cfg->zonemsg, &result, sizeof(LidarMsgHdr));
                            cfg->callback(2, &result, sizeof(LidarMsgHdr));
                        }
                        cfg->action = FINISH;
                        
                    }break;
                    
                    case 3:
                    {
                        // global parameters
                        // memcpy(&cfg->eepromv101, &result, sizeof(EEpromV101));
                        //((void (*)(int, void *))cfg->callback)(3, &result);
                        // cfg->action = FINISH;
                    }break;
                    
                    case 4:
                        // Response returned by time synchronization
                        break;

                    case 5:
                        // C_PACK
                        cfg->action = FINISH;
                        break;

                    case 6:
                        // S_PACK
                        cfg->action = FINISH;
                        break;
                    
                    case 7:
                        // readzone
                        break;
                        
                    case 8:
                        // writezone
                        break;

                    case 9:
                        // Status information sent per revolution of the serial port
                        break;

                } /// of switch (is_pack)
            } /// of if (buf_len > 0)
		} /// of if (FD_ISSET(cfg->fd, &fds))
        
		switch (cfg->action)
		{
            case CONTROL:
            {
                CAPI_UDPT_CPACK( cfg->fd, cfg->runscript.connectArg, \
                                 cfg->runscript.connectArg2, \
                                 cfg->send_len, cfg->send_cmd, 2, \
                                 "OK", 0, nullptr );
                cfg->action = FINISH;                
            } break;
            
            case GETALLPARAMS:
            {
                if (!CAPI_UDPT_GSPACK( cfg->fd, cfg->runscript.connectArg, \
                                       cfg->runscript.connectArg2, \
                                       cfg->send_len, cfg->send_cmd, \
                                       &cfg->eepromv101) )
                {
#ifdef DEBUG
                    printf("GetDevInfo_MSG failed\n");
#endif /// of DEBUG
                    strcpy(cfg->recv_cmd, "NG");
                }
                else
                    strcpy(cfg->recv_cmd, "OK");
                cfg->action = FINISH;
                
            } break;
            
            case SETPARAM:
            {
                if (cfg->mode == S_PACK)
                {
                    if (CAPI_UDPT_SPACK(cfg->fd, cfg->runscript.connectArg, cfg->runscript.connectArg2, cfg->send_len, cfg->send_cmd, result))
                    {
#ifdef DEBUG
                        printf("set LiDAR  %s %s\n", cfg->send_cmd, result);
#endif /// of DEBUG
                        strcpy(cfg->recv_cmd, result);
                    }
                    else
                        strcpy(cfg->recv_cmd, "NG");
                }
                else if (cfg->mode == C_PACK)
                {
                    if (CAPI_UDPT_CPACK(cfg->fd, cfg->runscript.connectArg, cfg->runscript.connectArg2, cfg->send_len, cfg->send_cmd, 2, "OK", 0, NULL))
                    {
#ifdef DEBUG
                        printf("set LiDAR  %s OK\n", cfg->send_cmd);
#endif /// of DEBUG
                        strcpy(cfg->recv_cmd, "OK");
                    }
                    else
                        strcpy(cfg->recv_cmd, "NG");
                }
                cfg->action = FINISH;
                
            } break;
            
            case READZONE:
                break;

            case WRITEZONE:
                break;

            default:
                break;
        } /// of switch (cfg->action)
        
#ifdef PTHREAD_NEED_YIELD
        msleep( 10 );
#endif /// of PTHREAD_NEED_YIELD
	}
    
    if ( buf != nullptr )
    {
        delete[] buf;
        buf = nullptr;
    }
    
    if ( fan_segs != nullptr )
    {
        delete fan_segs;
        fan_segs = nullptr;
    }
    
#ifdef DEBUG
	printf("cfg->state = %d\n", cfg->state);
#endif /// of DEBUG
	SystemAPI::closefd(cfg->fd, true);
    
    pthread_exit( NULL );
	return NULL;
}

#include <cctype>
#include <algorithm>
bool readConfig(const char *cfg_file_name, RunScript &cfg)
{
	std::ifstream infile;
	infile.open(cfg_file_name);
	if (!infile.is_open())
		return false;

	std::string str, key, value;
	while (getline(infile, str))
	{
		std::stringstream linestream(str);
		getline(linestream, key, ':');
		if (key.find("#") != std::string::npos || key.find(" ") != std::string::npos)
			continue;
		getline(linestream, value);
		if (!key.empty())
			value.erase(std::remove_if(value.begin(), value.end(), ::isspace), value.end());

		if (key == "type")
			strcpy(cfg.type, value.c_str());
		else if (key == "connectArg")
			strcpy(cfg.connectArg, value.c_str());
		else if (key == "connectArg2")
			cfg.connectArg2 = atoi(value.c_str());
		else if (key == "from_zero")
			cfg.from_zero = atoi(value.c_str());
		else if (key == "error_circle")
			cfg.error_circle = atoi(value.c_str());
		else if (key == "error_scale")
			cfg.error_scale = atof(value.c_str());
		else if (key == "output_360")
			cfg.output_360 = atoi(value.c_str());
		else if (key == "is_open_service")
			cfg.is_open_service = atoi(value.c_str());
		else if (key == "is_group_listener")
			cfg.is_group_listener = atoi(value.c_str());
		else if (key == "group_ip")
			strcpy(cfg.group_ip, value.c_str());
		else if (key == "shadow_filter.enable")
			cfg.shadows_filter.enable = atoi(value.c_str());
		else if (key == "shadow_filter.max_range")
			cfg.shadows_filter.max_range = atoi(value.c_str());
		else if (key == "shadow_filter.min_angle")
			cfg.shadows_filter.min_angle = atoi(value.c_str());
		else if (key == "shadow_filter.max_angle")
			cfg.shadows_filter.max_angle = atoi(value.c_str());
		else if (key == "shadow_filter.window")
			cfg.shadows_filter.window = atoi(value.c_str());
		else if (key == "median_filter.enable")
			cfg.median_filter.enable = atoi(value.c_str());
		else if (key == "median_filter.window")
			cfg.median_filter.window = atoi(value.c_str());
		else if (key == "uuid")
			cfg.uuid = atoi(value.c_str());
		else if (key == "model")
			cfg.model = atoi(value.c_str());
		else if (key == "version")
			cfg.version = atoi(value.c_str());
		else if (key == "rpm")
			cfg.rpm = atoi(value.c_str());
		else if (key == "resample_res")
			cfg.resample_res = atoi(value.c_str());
		else if (key == "with_smooth")
			cfg.with_smooth = atoi(value.c_str());
		else if (key == "with_deshadow")
			cfg.with_deshadow = atoi(value.c_str());
		else if (key == "with_start")
			cfg.with_start = atoi(value.c_str());
		else if (key == "with_confidence")
			cfg.with_confidence = atoi(value.c_str());
		else if (key == "unit_is_mm")
			cfg.unit_is_mm = atoi(value.c_str());
		else if (key == "alarm_msg")
			cfg.alarm_msg = atoi(value.c_str());
		else if (key == "direction")
			cfg.direction = atoi(value.c_str());
		else if (key == "ats")
			cfg.ats = atoi(value.c_str());
		else if (key == "local_port")
			cfg.local_port = atoi(value.c_str());
		else if (key == "filter_open")
			cfg.separation_filter.filter_open = atoi(value.c_str());
		else if (key == "max_range")
			cfg.separation_filter.max_range = atof(value.c_str());
		else if (key == "min_range")
			cfg.separation_filter.min_range = atof(value.c_str());
		else if (key == "max_range_difference")
			cfg.separation_filter.max_range_difference = atof(value.c_str());
		else if (key == "filter_window")
			cfg.separation_filter.filter_window = atoi(value.c_str());
		else if (key == "ntp_ip")
			strcpy(cfg.ntp_ip, value.c_str());
		else if (key == "ntp_port")
			cfg.ntp_port = atoi(value.c_str());
		else if (key == "ntp_enable")
			cfg.ntp_enable = atoi(value.c_str());
	}
	if (cfg.error_scale == 0)
		cfg.error_scale = 0.9;

	if (cfg.error_circle == 0)
		cfg.error_circle = 3;

	return true;
}

bool checkPointsLengthZero(UserData *tmp, float scale)
{
	size_t lengthZeroNum = 0;
	for (size_t i = 0; i < tmp->framedata.data.size(); i++)
	{
		if (tmp->framedata.data[i].distance == 0)
		{
			lengthZeroNum++;
		}
	}

	if (tmp->framedata.data.size() * scale < lengthZeroNum)
		return true;
    
	return false;
}
