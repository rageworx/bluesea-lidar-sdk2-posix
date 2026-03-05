#include "standard_interface.h"
#include "error.h"

// singletone instance --
BlueSeaLidarSDK *BlueSeaLidarSDK::m_sdk = \
    new (std::nothrow)BlueSeaLidarSDK();

BlueSeaLidarSDK *BlueSeaLidarSDK ::getInstance()
{
	return m_sdk;
}

void BlueSeaLidarSDK::deleteInstance()
{
	if ( m_sdk != nullptr )
	{
		delete m_sdk;
		m_sdk = nullptr;
	}
}

BlueSeaLidarSDK::BlueSeaLidarSDK()
 : m_idx( 0 ),
   m_service( nullptr )
{
}

BlueSeaLidarSDK::~BlueSeaLidarSDK()
{
}

RunConfig *BlueSeaLidarSDK::GetLidarConfig(int ID)
{
	for (size_t i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars[i]->ID == ID)
			return m_lidars[i];
	}

	return NULL;
}

int BlueSeaLidarSDK::AddLidarByPath(const char *cfg_file_name)
{
	RunConfig *cfg = new RunConfig;
    
    if (cfg == NULL)
    {
        return -1;
    }
    
	memset((void*)cfg, 0, sizeof(RunConfig));
    
	if (readConfig(cfg_file_name, cfg->runscript))
	{
		m_idx++;
		cfg->ID = m_idx;
		m_lidars.push_back(cfg);
		return m_idx;
	}

    return 0;
}

bool BlueSeaLidarSDK::DeleteLidarByID(int ID)
{
	for (size_t i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars[i]->ID == ID)
		{
			if ( m_lidars[i]->state == WORK )
			{
				m_lidars[i]->state = STOP_ALL;
				sleep(1);
			}
			m_lidars.erase(m_lidars.begin() + i);
			return true;
		}
	}
	return false;
}

void BlueSeaLidarSDK::SetCallBackPtr(int ID, printfMsg ptr)
{
	for (size_t i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars[i]->ID == ID)
		{
			m_lidars[i]->callback = ptr;
		}
	}
}

bool BlueSeaLidarSDK::OpenDev(int ID)
{
	RunConfig *lidar = NULL;
	for (size_t i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars[i]->ID == ID)
		{
			lidar = m_lidars[i];
		}
	}
    
	if (lidar == NULL)
    {
		return false;
    }
    
	if (strcmp(lidar->runscript.type, "uart") == 0 \
        || strcmp(lidar->runscript.type, "vpc") == 0)
	{
		int fd = SystemAPI::open_serial_port(lidar->runscript.connectArg, \
                                             lidar->runscript.connectArg2);
		if (fd <= 0)
		{
			return false;
		}
        
		lidar->fd = fd;

		if (pthread_create(&lidar->thread_data, \
                           NULL, lidar_thread_proc_uart, lidar) != 0)
        {
			return false;
        }
	}
	else
    if (strcmp(lidar->runscript.type, "udp") == 0)
	{
		int fd = SystemAPI::open_socket_port(lidar->runscript.local_port);
		if (fd <= 0)
		{
			return false;
		}
		lidar->fd = fd;

		if ( pthread_create(&lidar->thread_data, \
                            NULL, lidar_thread_proc_udp, lidar) != 0 )
			return false;
	}
	else
	{
		return false;
	}
    
	lidar->state = WORK;

	// Starting a local web service is primarily for visually checking 
    // whether the radar point cloud is functioning correctly; 
    // -- it is not necessary if the SDK is integrated separately.
	if (lidar->runscript.is_open_service)
	{
		// Start the radar heartbeat packet thread 
        // and detect the number of currently online radars.
		OpenHeartService();
	}
	// Determine if the data thread is running normally
	size_t index = 100;
	while (lidar->action < ONLINE && index > 0)
	{
		msleep(100);
		index--;
	}
    
	if (lidar->action >= ONLINE)
		return true;

	return false;
}

bool BlueSeaLidarSDK::GetDevInfo(int ID, EEpromV101* eepromv101)
{
	RunConfig *lidar = NULL;
	for (size_t i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars[i]->ID == ID)
		{
			lidar = m_lidars[i];
		}
	}
    
	if (lidar == NULL)
    {
		return false;
    }

	lidar->action = GETALLPARAMS;
	size_t index = 30;
	while (lidar->action != FINISH && index > 0)
	{
		msleep(200);
		index--;
	}

	if (lidar->action == FINISH)
	{
		if (strcmp(lidar->recv_cmd, "OK") == 0)
		{
			memcpy(eepromv101, &lidar->eepromv101, sizeof(EEpromV101));
			return true;
		}
	}
    
	return false;
}

bool BlueSeaLidarSDK::SetDevInfo(RunConfig *lidar, int num, char *cmd, int mode)
{
	lidar->mode         = mode;
	lidar->send_len     = num;
	memset(lidar->recv_cmd, 0, sizeof(lidar->recv_cmd));
    
	lidar->recv_len     = 0;
	strcpy(lidar->send_cmd, cmd);
	lidar->action = SETPARAM;
    
	size_t index = 20;
    
	while (lidar->action != FINISH && index > 0)
	{
		msleep(100);
		index--;
	}
    
	if (lidar->action == FINISH)
	{
		if (strcmp(lidar->recv_cmd, "OK") == 0)
		{
#ifdef DEBUG
			printf("%s OK\n", lidar->send_cmd);
#endif /// of DEBUG
			return true;
		}
		else 
        if (strcmp(lidar->recv_cmd, "NG") == 0)
		{
#ifdef DEBUG
			printf("%s NG\n", lidar->send_cmd);
#endif /// of DEBUG
			return false;
		}
	}
    
	return false;
}

// 释放连接
void BlueSeaLidarSDK::StopDev(int ID)
{
	for (size_t i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars[i]->ID == ID)
		{
			m_lidars[i]->action = OFFLINE;
		}
	}
}

int BlueSeaLidarSDK::GetDevState(int ID)
{
	for (size_t i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars[i]->ID == ID)
		{
			return m_lidars[i]->action;
		}
	}
    
    return OFFLINE;
}

const char *BlueSeaLidarSDK::GetVersion()
{
	return SDKVERSION;
}

// 关闭本地服务
bool BlueSeaLidarSDK::closeService(int ID)
{
	RunConfig *lidar = nullptr;
    
	for (size_t i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars[i]->ID == ID)
		{
			lidar = m_lidars[i];
		}
	}
    
	if (lidar == nullptr)
    {
		return false;
    }

	if ( m_lidars[ID]->state == WORK )
	{
		return true;
	}

	return false;
}

bool BlueSeaLidarSDK::OpenHeartService()
{
	if ( m_service == nullptr )
	{
		m_service = new LidarService();
	}
    
    if ( m_service != nullptr )
	{
        return m_service->Run();
    }
    
    return false;
}

bool BlueSeaLidarSDK::CloseHeartService()
{
    if ( m_service != nullptr )
	{
        m_service->Stop();
    }
    
	return true;
}

// Start-stop radar ranging
bool BlueSeaLidarSDK::ControlDrv(int ID, int num, char *cmd)
{
	RunConfig *lidar = NULL;
    
	for (size_t i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars[i]->ID == ID)
		{
			lidar = m_lidars[i];
		}
	}
    
	if (lidar == NULL)
    {
		return false;
    }

	lidar->mode     = C_PACK;
	lidar->send_len = num;
	strcpy(lidar->send_cmd, cmd);
    
	lidar->action = CONTROL;
	size_t index  = 30;
    
	while (lidar->action != FINISH && index > 0)
	{
		msleep(100);
		index--;
	}
    
	if (lidar->action == FINISH)
	{
#ifdef DEBUG
		printf("%s OK\n", lidar->send_cmd);
#endif /// of DEBUG
		return true;
	}
    
	else if (index == 0 && strcmp(lidar->send_cmd, "LRESTH") == 0)
	{
#ifdef DEBUG
		printf("%s OK\n", lidar->send_cmd);
#endif /// of DEBUG
		return true;
	}

	return false;
}

bool BlueSeaLidarSDK::ZoneSection(int ID, char section)
{
	RunConfig *lidar = NULL;
    
	for (size_t i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars[i]->ID == ID)
		{
			lidar = m_lidars[i];
		}
	}
    
	if (lidar == NULL)
    {
		return false;
    }

	// Determine if the incoming defense zone is legitimate.
	if ((section >= 48 && section <= 57) || (section >= 65 && section <= 70))
	{
		char tmp[12] = {0};
		snprintf(tmp, 12, "LSAZN:%cH", section);
		return SetDevInfo(lidar, strlen(tmp), tmp, S_PACK);
	}

	return false;
}

bool BlueSeaLidarSDK::SetUDP(int ID, char *ip, char *mask, char *gateway, int port)
{
	RunConfig *lidar = NULL;
    
	for (size_t i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars[i]->ID == ID)
		{
			lidar = m_lidars[i];
		}
	}
    
	if (lidar == NULL)
    {
		return false;
    }

	char result[64] = {0};
    
	// Validate the format of the input data.
	if (!BaseAPI::checkAndMerge(1, ip, mask, gateway, port, result, 64))
	{
		return false;
	}
    
	char tmp[128] = {0};
	snprintf(tmp, 128, "LSUDP:%sH", result);
    
	lidar->mode     = S_PACK;
	lidar->send_len = strlen(tmp);
	strcpy(lidar->send_cmd, tmp);
    
	lidar->action = SETPARAM;
	
    // No return value was found after modifying the IP address.
	return true;
}

bool BlueSeaLidarSDK::SetDST(int ID, char *ip, int port)
{
	RunConfig *lidar = NULL;
    
	for (size_t i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars[i]->ID == ID)
		{
			lidar = m_lidars[i];
		}
	}
    
	if (lidar == NULL)
    {
		return false;
    }

	char result[50] = {0};
    char noaddr[2] = "";
	// Validate the format of the input data.
	if (!BaseAPI::checkAndMerge(0, ip, noaddr, noaddr, port, result, 50))
	{
		return false;
	}
    
	char tmp[64] = {0};
	snprintf(tmp, 64, "LSDST:%sH", result);
    
	return SetDevInfo(lidar, strlen(tmp), tmp, S_PACK);
}

bool BlueSeaLidarSDK::SetRPM(int ID, int RPM)
{
	RunConfig *lidar = NULL;
    
	for (size_t i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars[i]->ID == ID)
		{
			lidar = m_lidars[i];
		}
	}
    
	if (lidar == NULL)
    {
		return false;
    }

	// Validate the format of the input data.
	if (RPM < 300 || RPM > 3000)
	{
		return false;
	}
    
	char cmd[16] = {0};
	snprintf(cmd, 16, "LSRPM:%dH", RPM);
    
	return SetDevInfo(lidar, strlen(cmd), cmd, C_PACK);
}

bool BlueSeaLidarSDK::SetTFX(int ID, bool tfx)
{
	RunConfig *lidar = NULL;
    
	for (size_t i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars[i]->ID == ID)
		{
			lidar = m_lidars[i];
		}
	}
    
	if (lidar == NULL)
    {
		return false;
    }

	char cmd[16] = {0};
	snprintf(cmd, 16, "LSTFX:%dH", tfx);
    
	return SetDevInfo(lidar, strlen(cmd), cmd, S_PACK);
}

bool BlueSeaLidarSDK::SetDSW(int ID, bool dsw)
{
	RunConfig *lidar = NULL;
    
	for (size_t i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars[i]->ID == ID)
		{
			lidar = m_lidars[i];
		}
	}
    
	if (lidar == NULL)
    {
		return false;
    }

	char cmd[16] = {0};
	snprintf(cmd, 16, "LFFF%dH", dsw);
    
	return SetDevInfo(lidar, strlen(cmd), cmd, C_PACK);
}

bool BlueSeaLidarSDK::SetSMT(int ID, bool smt)
{
	RunConfig *lidar = NULL;
    
	for (size_t i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars[i]->ID == ID)
		{
			lidar = m_lidars[i];
		}
	}
    
	if (lidar == NULL)
    {
		return false;
    }

	char cmd[16] = {0};
	snprintf(cmd, 16, "LSSS%dH", smt);
    
	return SetDevInfo(lidar, strlen(cmd), cmd, C_PACK);
}

bool BlueSeaLidarSDK::SetPST(int ID, int mode)
{
	RunConfig *lidar = NULL;
    
	for (size_t i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars[i]->ID == ID)
		{
			lidar = m_lidars[i];
		}
	}
    
	if (lidar == NULL)
    {
		return false;
    }

	if (mode <= 0 || mode > 3)
		return false;

	char cmd[16] = {0};
	snprintf(cmd, 16, "LSPST:%dH", mode);
    
	return SetDevInfo(lidar, strlen(cmd), cmd, S_PACK);
}

bool BlueSeaLidarSDK::SetDID(int ID, uint32_t did)
{
	RunConfig *lidar = NULL;
    
	for (size_t i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars[i]->ID == ID)
		{
			lidar = m_lidars[i];
		}
	}
    
	if (lidar == NULL)
    {
		return false;
    }

	char cmd[16] = {0};
	snprintf(cmd, 16, "LSDID:%uH", did);
    
	return SetDevInfo(lidar, strlen(cmd), cmd, S_PACK);
}

bool BlueSeaLidarSDK::SetNTP(int ID, char *ntp_ip, uint16_t port, bool enable)
{
	RunConfig *lidar = NULL;
    
	for (size_t i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars[i]->ID == ID)
		{
			lidar = m_lidars[i];
		}
	}
    
	if (lidar == NULL)
    {
		return false;
    }

	char cmd[64] = {0,};
	char ip_1[4] = {0,};
	char ip_2[4] = {0,};
	char ip_3[4] = {0,};
	char ip_4[4] = {0,};
	ip_1[3] = '\0';
	ip_2[3] = '\0';
	ip_3[3] = '\0';
	ip_4[3] = '\0';

	size_t idx[3] = {0,};
	size_t index = 0;
	size_t ip_len = strlen(ntp_ip);
    
	for (size_t i = 0; i < ip_len; i++)
	{
		if (ntp_ip[i] == '.')
		{
			idx[index] = i;
			index++;
		}
	}
    
	if (index != 3)
	{
		PRTSTDERR( "ntp ip set error!");
		return false;
	}
	else
	{
		memcpy(ip_1, &ntp_ip[0], idx[0]);
		memcpy(ip_2, &ntp_ip[idx[0] + 1], idx[1] - idx[0] - 1);
		memcpy(ip_3, &ntp_ip[idx[1] + 1], idx[2] - idx[1] - 1);
		memcpy(ip_4, &ntp_ip[idx[2] + 1], ip_len - idx[2]);
		snprintf(cmd, 64, "LSNTP:%u,%03d.%03d.%03d.%03d,%05uH", \
                 enable, \
                 atoi(ip_1), atoi(ip_2), atoi(ip_3), atoi(ip_4), \
                 port);
                 
		return SetDevInfo(lidar, strlen(cmd), cmd, S_PACK);
	}
	return false;
}

std::vector<DevConnInfo> BlueSeaLidarSDK::GetLidarsList()
{
	return m_service->LidarList();
}
