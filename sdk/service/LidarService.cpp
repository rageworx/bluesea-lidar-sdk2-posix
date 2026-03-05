#include <pthread.h>
#include "LidarService.h"

std::vector<DevConnInfo> deviceInfos;

void* thread_heart(void* p)
{
    if ( p != nullptr )
    {
        LidarService* pLS = (LidarService*)p;

        int32_t ret = pLS->ThreadCallBack( pLS );

        pthread_exit( &ret );
        return nullptr;
    }

    pthread_exit( nullptr );
    return nullptr;
}

void uptodate(DevConnInfo data)
{
	if (deviceInfos.size() == 0)
	{
		deviceInfos.push_back(data);
		return ;
	}
    
	for ( size_t i = 0; i < deviceInfos.size(); i++ )
	{
		//If it's the same radar, then it covers...
		if ( ( data.type == TYPE_COM && \
               strcmp( data.com_port, deviceInfos[i].com_port ) == 0 ) ||
			 ( data.type != TYPE_COM && \
               strcmp( data.conn_ip, deviceInfos[i].conn_ip ) == 0 ) )
		{
			memcpy( &deviceInfos[i], &data, sizeof(DevConnInfo) );
			return;
		}
	}
    
	deviceInfos.push_back(data);
}

LidarService::LidarService()
 : ptHeart( 0 ), bCloseService( false )
{
}

LidarService::~LidarService()
{
    Stop(); 
    
    deviceInfos.clear();
}

bool LidarService::Run()
{
	if (ptHeart != 0)
		return false;
    
	if ( pthread_create( &ptHeart, NULL, thread_heart, this) == 0 )
    {
        return true;
    }
    else
    {
        fprintf( stderr, "LidarService, thread failure.\n" );
    }
    
    return false;
}

void LidarService::Stop()
{
    if ( bCloseService == false )
    {
        bCloseService = true;
    
        pthread_join( ptHeart, NULL );
    }
}

std::vector<DevConnInfo> LidarService::LidarList()
{
	UpdateUartInfo();
	return deviceInfos;
}

void LidarService::GetTime_HMS(char* data, size_t datalen)
{
	time_t t0 = time(nullptr);
    
	uint32_t hh = t0 % (3600 * 24) / 3600;
	uint32_t mm = t0 % 3600 / 60;
	uint32_t ss = t0 % 60;
    
	snprintf(data, datalen, "%u-%u-%u", hh, mm, ss);
}

void LidarService::Clear()
{
	deviceInfos.clear();
}

void LidarService::UpdateUartInfo()
{
	std::vector<UARTARG>list;
	SystemAPI::GetComList(list);
	for (size_t i = 0; i < list.size(); i++)
	{
		DevConnInfo tmp;
		memset( &tmp, 0, sizeof(DevConnInfo) );
		strcpy( tmp.com_port, list[i].portName );
		tmp.com_speed = list[i].port;
		strcpy( tmp.timeStr, "0" );
		uptodate(tmp);
	}
}

int32_t LidarService::State()
{
    if ( bCloseService == true )
        return 1;
    
    return -1;
}

int32_t LidarService::ThreadCallBack( void* param )
{
    if ( param != this )
        return -1;

    initSocket();
    
	int sock = socket(AF_INET, SOCK_DGRAM, 0);

	int yes = 1;
	if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (char*)&yes, sizeof(yes)) < 0)
	{
		return -1;
	}

	sockaddr_in addr = {0,};
	addr.sin_family      = AF_INET;
	addr.sin_port        = htons(CHECKSERVICE);
	addr.sin_addr.s_addr = htonl(INADDR_ANY);

	int iResult = ::bind(sock, (struct sockaddr*)&addr, sizeof(addr));
	if (iResult != 0)
		return -1;

	struct ip_mreq mreq;
	mreq.imr_multiaddr.s_addr = inet_addr("225.225.225.225");
	mreq.imr_interface.s_addr = htonl(INADDR_ANY);
	
    if (setsockopt(sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char*)&mreq, sizeof(mreq)) < 0)
	{
		return -1;
	}
    
	while( State() != 1 )
	{
		socklen_t sz = sizeof(addr);
		char raw[4096] = {0,};
		int dw = recvfrom(sock, raw, sizeof(raw), 0, (struct sockaddr*)&addr, &sz);
        
		if (dw == sizeof(DevInfoV101))
		{
			DevInfoV101* dvi = (DevInfoV101*)raw;
			if (memcmp(dvi->sign, "LiDA", 4) == 0 && dvi->proto_version == 0x101)
			{
				DevConnInfo conn;
				memset(&conn, 0, sizeof(DevConnInfo));
				conn.type = TYPE_UDP_V101;
				strcpy(conn.conn_ip, inet_ntoa(addr.sin_addr));
				conn.conn_port = ntohs(addr.sin_port);
				memcpy(&conn.info.v101, dvi, sizeof(DevInfoV101));
				LidarService::GetTime_HMS(conn.timeStr, 16);
				uptodate(conn);
			}
		}
        
		if (dw == sizeof(DevInfo2))
		{
			DevInfo2* dvi = (DevInfo2*)raw;
			if (memcmp(dvi->sign, "LiDA", 4) == 0)
			{
				DevConnInfo conn;
				memset(&conn, 0, sizeof(DevConnInfo));
				conn.type = TYPE_UDP_V2;
				strcpy(conn.conn_ip, inet_ntoa(addr.sin_addr));
				conn.conn_port = ntohs(addr.sin_port);

				memcpy(&conn.info.v2, dvi, sizeof(DevInfo2));

				LidarService::GetTime_HMS(conn.timeStr,16);
				uptodate(conn);
			}
		}
        
		if (dw == sizeof(DevInfo))
		{
			DevInfo* dvi = (DevInfo*)raw;
			if (memcmp(dvi->sign, "LiDA", 4) == 0)
			{
				DevConnInfo conn;
				conn.type = TYPE_UDP_V1;
				conn.info.v1 = *dvi;

				strcpy(conn.conn_ip, inet_ntoa(addr.sin_addr));
				conn.conn_port = ntohs(addr.sin_port);

				LidarService::GetTime_HMS(conn.timeStr,16);
				uptodate(conn);
			}
		}
	}
    
	SystemAPI::closefd(sock,true);
    
    finalSocket();
    
	return 0;
}
