#include "LidarCheckService.h"
#include "LidarWebService.h"
#include "standard_interface.h"

static const char *s_root_dir = "web";

static void thread_web(struct mg_connection* c, int ev, void* ev_data, void* fn_data);
static char* jsonValue(const char* result, const char* message, cJSON* array);
static void EEpromV101ToStr(EEpromV101* eepromv101, char* version, char* result, size_t result_len);
static void StringReplace(std::string& strBase, std::string strSrc, std::string strDes);

LidarWebService::LidarWebService(uint16_t port)
 : m_port( port ),
   m_loop( false )
{
}

LidarWebService::~LidarWebService()
{
    stop();
}

void LidarWebService::run(int32_t lidarID)
{	
	char address[64] = {0};
	snprintf(address, 64, "http://0.0.0.0:%d", m_port);
    
	struct mg_mgr mgr;  /// Event manager
	mg_log_set("2");    /// Set to 3 to enable debug
	mg_mgr_init(&mgr);  /// Initialise event manager

	// Create HTTP listener
    mg_http_listen(&mgr, address, thread_web, &lidarID);
    m_loop = true;
    
	while( m_loop == true )
	{
        // Infinite event loop
		mg_mgr_poll(&mgr, 100); 
	}
    
	mg_mgr_free(&mgr);
}

void LidarWebService::stop()
{
    if ( m_loop == true )
        m_loop = false;
}

static char *jsonValue(const char *result, const char *message, cJSON *array)
{
	if (array == NULL)
		array = cJSON_CreateArray();
    
    if ( array == NULL )
    {
        // prevent GCC clause guarding warning.
        return NULL;
    }
    
	cJSON *root = cJSON_CreateObject();
	cJSON *item = cJSON_CreateString(result);
	cJSON_AddItemToObject(root, "result", item);
	item = cJSON_CreateString(message);
	cJSON_AddItemToObject(root, "message", item);

	cJSON_AddItemToObject(root, "data", array);
	char *out = cJSON_Print(root);
	cJSON_Delete(root);
	return out;
}

static void EEpromV101ToStr(EEpromV101 *eepromv101, char *version, char *result, size_t result_len)
{
	cJSON *root = cJSON_CreateObject();
    
    if ( root == NULL )
    {
        // prevent GCC clause guarding warning.
        return;
    }
    
	// 类型，编号，序列号
	char tmp_sn[20] = {0};
	memcpy(tmp_sn, eepromv101->dev_sn, sizeof(eepromv101->dev_sn) - 1);
	char tmp_type[16] = {0};
	memcpy(tmp_type, eepromv101->dev_type, sizeof(eepromv101->dev_type) - 1);
	//printf("qqq %s\n",tmp_type);
	cJSON *item = cJSON_CreateNumber(eepromv101->dev_id);
	cJSON_AddItemToObject(root, "DID", item);
	item = cJSON_CreateString(tmp_sn);
	cJSON_AddItemToObject(root, "UID", item);
	item = cJSON_CreateString(tmp_type);
	cJSON_AddItemToObject(root, "NSP", item);
    
	/*printf("dev info: 设备编号:%d\t 序列号:%s\t 类型:%s\n", eepromv101->dev_id, tmp_sn, tmp_type);*/
	// ip地址 子网掩码 网关地址 默认目标IP  默认目标udp端口号  默认UDP对外服务端口号
	char tmp_IPv4[16] = {0};
	char tmp_mask[16] = {0};
	char tmp_gateway[16] = {0};
	char tmp_srv_ip[16] = {0};

	snprintf(tmp_IPv4, 16, "%d.%d.%d.%d", \
             eepromv101->IPv4[0], eepromv101->IPv4[1], eepromv101->IPv4[2], eepromv101->IPv4[3]);
	snprintf(tmp_mask, 16, "%d.%d.%d.%d", \
             eepromv101->mask[0], eepromv101->mask[1], eepromv101->mask[2], eepromv101->mask[3]);
	snprintf(tmp_gateway, 16, "%d.%d.%d.%d", \
             eepromv101->gateway[0], eepromv101->gateway[1], eepromv101->gateway[2], eepromv101->gateway[3]);
	snprintf(tmp_srv_ip, 16, "%d.%d.%d.%d", \
             eepromv101->srv_ip[0], eepromv101->srv_ip[1], eepromv101->srv_ip[2], eepromv101->srv_ip[3]);

	// printf("dev info: ip地址:%s 子网掩码:%s 网关地址:%s 默认目标IP:%s  默认目标udp端口号:%d   默认UDP对外服务端口号:%d\n",
	//	tmp_IPv4, tmp_mask, tmp_gateway, tmp_srv_ip, eepromv101->srv_port, eepromv101->local_port);
	item = cJSON_CreateString(tmp_IPv4);
	cJSON_AddItemToObject(root, "IPv4", item);
	item = cJSON_CreateString(tmp_mask);
	cJSON_AddItemToObject(root, "mask", item);
	item = cJSON_CreateString(tmp_gateway);
	cJSON_AddItemToObject(root, "gateway", item);
	item = cJSON_CreateString(tmp_srv_ip);
	cJSON_AddItemToObject(root, "srv_ip", item);
	item = cJSON_CreateNumber(eepromv101->srv_port);
	cJSON_AddItemToObject(root, "srv_port", item);
	item = cJSON_CreateNumber(eepromv101->local_port);
	cJSON_AddItemToObject(root, "local_port", item);

	/*char tmp_ranger_bias[8] = { 0 };
	memcpy(tmp_ranger_bias, eepromv101->ranger_bias, sizeof(eepromv101->ranger_bias) - 1);*/

	int tmp_ranger_bias[6] = {0};
	for (int i = 0; i < 4; i++)
	{
		tmp_ranger_bias[i] = eepromv101->ranger_bias[i * 2 + 1];
		if (eepromv101->ranger_bias[i * 2])
			tmp_ranger_bias[i] *= -1;
	}
    
	// 转速 ,电机启动参数,FIR滤波阶数，圈数，分辨率，开机自动上传，固定上传，数据点平滑，去拖点，记录校正系数，网络心跳，记录IO口极性
	//  sprintf(result,"dev info: 转速:%d 电机启动参数:%d FIR滤波阶数:%d 圈数:%d  分辨率:%d   开机自动上传:%d 固定上传:%d  数据点平滑:%d 去拖点:%d  记录校正系数:%s  网络心跳:%d  记录IO口极性:%d\n",
	//	eepromv101->RPM, eepromv101->RPM_pulse, eepromv101->fir_filter, eepromv101->cir, eepromv101->with_resample, eepromv101->auto_start,
	//	eepromv101->target_fixed, eepromv101->with_smooth, eepromv101->with_filter, tmp_ranger_bias, eepromv101->net_watchdog, eepromv101->pnp_flags);
	item = cJSON_CreateNumber(eepromv101->RPM);
	cJSON_AddItemToObject(root, "RPM", item);

	item = cJSON_CreateNumber(eepromv101->RPM_pulse);
	cJSON_AddItemToObject(root, "PUL", item);

	item = cJSON_CreateNumber(eepromv101->fir_filter);
	cJSON_AddItemToObject(root, "FIR", item);

	item = cJSON_CreateNumber(eepromv101->cir);
	cJSON_AddItemToObject(root, "cir", item);

	item = cJSON_CreateNumber(eepromv101->with_resample);
	cJSON_AddItemToObject(root, "with_resample", item);

	item = cJSON_CreateNumber(eepromv101->auto_start);
	cJSON_AddItemToObject(root, "ATS", item);

	item = cJSON_CreateNumber(eepromv101->target_fixed);
	cJSON_AddItemToObject(root, "TFX", item);

	item = cJSON_CreateNumber(eepromv101->with_smooth);
	cJSON_AddItemToObject(root, "SMT", item);

	item = cJSON_CreateNumber(eepromv101->with_filter);
	cJSON_AddItemToObject(root, "DSW", item);

	item = cJSON_CreateIntArray(tmp_ranger_bias, 6);
	cJSON_AddItemToObject(root, "ERR", item);

	item = cJSON_CreateNumber(eepromv101->net_watchdog);
	cJSON_AddItemToObject(root, "net_watchdog", item);

	item = cJSON_CreateNumber(eepromv101->pnp_flags);
	cJSON_AddItemToObject(root, "PNP", item);

	item = cJSON_CreateNumber(eepromv101->deshadow);
	cJSON_AddItemToObject(root, "AF", item);

	item = cJSON_CreateNumber(eepromv101->should_post);
	cJSON_AddItemToObject(root, "PST", item);

	item = cJSON_CreateString(version);
	cJSON_AddItemToObject(root, "version", item);
	char *out = jsonValue("SUCCESS", "", root);
	snprintf(result, result_len, "%s", out);
	free(out);
}

static void thread_web(struct mg_connection *c, int ev, void *ev_data, void *fn_data)
{
	int id = *(int *)fn_data;
	RunConfig *runcfg = BlueSeaLidarSDK::getInstance()->getLidar(id);
	if (ev == MG_EV_HTTP_MSG)
	{
		struct mg_http_message *hm = (struct mg_http_message *)ev_data;
		if (mg_http_match_uri(hm, "/api/stats"))
		{
			cJSON *points = cJSON_CreateArray();
			cJSON *user;
			cJSON *item = cJSON_CreateNumber(1);
			for (struct mg_connection *t = c->mgr->conns; t != NULL; t = t->next)
			{
				char loc[40], rem[40];
				user = cJSON_CreateObject();
				item = cJSON_CreateNumber(t->id);
				cJSON_AddItemToObject(user, "ID", item);

				item = cJSON_CreateString(t->is_udp ? "UDP" : "TCP");
				cJSON_AddItemToObject(user, "type", item);

				item = cJSON_CreateString(t->is_listening ? "LISTENING" : t->is_accepted ? "ACCEPTED "
																						 : "CONNECTED");
				cJSON_AddItemToObject(user, "state", item);
				mg_straddr(&t->loc, loc, sizeof(loc));
				item = cJSON_CreateString(loc);
				cJSON_AddItemToObject(user, "loc", item);
				mg_straddr(&t->rem, rem, sizeof(rem));
				item = cJSON_CreateString(rem);
				cJSON_AddItemToObject(user, "rem", item);
				cJSON_AddItemToArray(points, user);
			}
			char *out = jsonValue("SUCCESS", "", points);
			mg_http_reply(c, 200, "", "%s", out);
			free(out);
			return;
		}

		else if (mg_http_match_uri(hm, "/init"))
		{
			cJSON *arr = cJSON_CreateArray();
			cJSON *point = cJSON_CreateObject();
			cJSON *item = cJSON_CreateString(runcfg->runscript.type);
			cJSON_AddItemToObject(point, "type", item);
			std::string SYSType;
#ifdef _WIN32
			SYSType = "WIN32";
#elif __linux__
			SYSType = "LINUX";
#elif __APPLE__
			SYSType = "APPLE";
#endif
			item = cJSON_CreateString(SYSType.c_str());
			cJSON_AddItemToObject(point, "SDKENV", item);
			cJSON_AddItemToObject(arr, "", point);
			char *out = jsonValue("SUCCESS", "", arr);
			mg_http_reply(c, 200, "", "%s", out);
			free(out);
			return;
		}
		// 雷达的动作控制
		else if (mg_http_match_uri(hm, "/action"))
		{
			//cJSON *arr = cJSON_CreateArray();
			char query[256] = {0};
			memcpy(query, hm->query.ptr, hm->query.len);
			char *ret = strstr(query, "cmd=");
			if (!ret || strlen(ret + 4) != 6)
			{
				char *out = jsonValue("ERROR", "url is not current!", NULL);
				mg_http_reply(c, 200, "", "%s", out);
				free(out);
				return;
			}

			char cmd[7] = {0};
			sprintf(cmd, "%s", ret + 4);
			runcfg->mode = S_PACK;
			runcfg->send_len = 6;
			strcpy(runcfg->send_cmd, cmd);
			runcfg->action = CONTROL;

			char *out = jsonValue("SUCCESS", "", NULL);
			mg_http_reply(c, 200, "", "%s", out);
			free(out);
			return;
		}
		// 雷达点云数据/报警信息
		else if (mg_http_match_uri(hm, "/data"))
		{
			size_t pointsNum = runcfg->userdata.framedata.data.size();
            
			if (pointsNum == 0 || pointsNum > MAX_FRAMEPOINTS)
			{
				char message[64] = {0};
				snprintf(message, 64, "get point number unusual %zu", pointsNum);
				char *out = jsonValue("ERROR", message, NULL);
				mg_http_reply(c, 200, "", "%s", out);
				free(out);
				return;
			}
                        
			cJSON *root = cJSON_CreateObject();
            if ( root == NULL )
                return;
			cJSON *points = cJSON_CreateArray();
            if ( points == NULL )
            {
                cJSON_Delete(root);
                cJSON_Delete(points);
                return;
            }
			cJSON *point = NULL;
			cJSON *item = cJSON_CreateNumber(pointsNum);
            if ( item == NULL )
            {
                cJSON_Delete(root);
                return;
            }
			cJSON_AddItemToObject(root, "N", item);

            FrameData *framedata = &runcfg->userdata.framedata;

            int tr = pthread_mutex_trylock( &runcfg->userdata.framedata.datalock );
            if ( tr != 0 )
            {
                fprintf( stderr, 
                         "310: pthread_mutex_trylock( &framedata->datalock ) failure (%d)\n",
                         tr );
                fflush( stderr );
            }
            else
            {
                item = cJSON_CreateNumber(runcfg->userdata.framedata.ts[0]);
                cJSON_AddItemToObject(root, "timestamp_s", item);
                item = cJSON_CreateNumber(runcfg->userdata.framedata.ts[1]);
                cJSON_AddItemToObject(root, "timestamp_us", item);
                pthread_mutex_unlock( &runcfg->userdata.framedata.datalock );
            }
            
            tr = pthread_mutex_trylock( &framedata->datalock );
            if ( tr != 0 )
            {
                fprintf( stderr, 
                         "330: pthread_mutex_trylock( &framedata->datalock ) failure (%d)\n",
                         tr );
                fflush( stderr );                
            }
            else
            {
                for (size_t i = 0; i < framedata->data.size(); i++)
                {
                    point = cJSON_CreateObject();
                    if ( point != NULL )
                    {
                        item = cJSON_CreateNumber(framedata->data[i].angle * 180.0/PI);
                        cJSON_AddItemToObject(point, "angle", item);
                        if ( item != NULL )
                            item = cJSON_CreateNumber(framedata->data[i].distance);
                        cJSON_AddItemToObject(point, "distance", item);
                        if ( item != NULL )
                            item = cJSON_CreateNumber(framedata->data[i].confidence);
                        
                        if ( item != NULL )
                            cJSON_AddItemToObject(point, "confidence", item);
                        
                        if ( point != NULL && points != NULL )
                            cJSON_AddItemToArray(points, point);
                    }
                }
                pthread_mutex_unlock( &framedata->datalock );
            }

			cJSON_AddItemToObject(root, "data", points);
			item = cJSON_CreateString("SUCCESS");
			cJSON_AddItemToObject(root, "result", item);
			item = cJSON_CreateString("");
			cJSON_AddItemToObject(root, "message", item);

			// 添加防区相关的数据
			LidarMsgHdr zone;
			memcpy(&zone, &runcfg->zonemsg, sizeof(LidarMsgHdr));

			item = cJSON_CreateNumber(zone.flags);
			cJSON_AddItemToObject(root, "zone_flag", item);
			item = cJSON_CreateNumber(zone.events);
			cJSON_AddItemToObject(root, "zone_events", item);
			item = cJSON_CreateNumber(zone.zone_actived);
			cJSON_AddItemToObject(root, "zone_actived", item);

			char *out = cJSON_Print(root);
			mg_http_reply(c, 200, "", "%s", out);
			cJSON_Delete(root);
			free(out);
			msleep(10); /// 1000/10 = 100Hz
            
            static size_t progress_q = 0;
            printf( "\rProgress : %zu ...    ", progress_q ++ );
		}
		// 获取雷达设备信息
		else if (mg_http_match_uri(hm, "/getDevinfo"))
		{
			EEpromV101 *eeprom = new  EEpromV101;
			char info[1024]={0};
			BlueSeaLidarSDK::getInstance()->GetDevInfo(id,eeprom);
			EEpromV101ToStr(eeprom,runcfg->hardwareVersion, info, 1024);
			mg_http_reply(c, 200, "", "%s", info);
			return;
		}
		else if (mg_http_match_uri(hm, "/getLidarList"))
		{
			char query[256] = {0};
			memcpy(query, hm->query.ptr, hm->query.len);
			std::string str = query;
			StringReplace(str, "%20", " ");
			int ret1 = str.find("mode=");
			if (ret1 < 0)
			{
				char *out = jsonValue("ERROR", "url is not current!", NULL);
				mg_http_reply(c, 200, "", "%s", out);
				free(out);
				return;
			}
			// 这里0为打开服务,并获取当前的雷达列表  1为关闭服务
			int index = atoi(str.substr(ret1 + 5, 1).c_str());
			if (index == 0)
			{
				std::vector<DevConnInfo> data = BlueSeaLidarSDK::getInstance()->getLidarsList();
#ifdef DEBUG
				printf("num:%zu\n",data.size()); fflush( stdout );
#endif /// of DEBUG
				cJSON *arr = cJSON_CreateArray();
                if ( arr == NULL )
                    return;
				cJSON *point = NULL;
				cJSON *item = NULL;
				char conn_ip[16] = {0};
				uint16_t conn_port = 0;
				for (size_t i = 0; i < data.size(); i++)
				{
					point = cJSON_CreateObject();
					int type = data[i].type;
					item = cJSON_CreateNumber(data[i].type);
					cJSON_AddItemToObject(point, "type", item);

					item = cJSON_CreateString(data[i].com_port);
					cJSON_AddItemToObject(point, "com_port", item);

					item = cJSON_CreateNumber(data[i].com_speed);
					cJSON_AddItemToObject(point, "com_speed", item);

					if (type == 0)
					{
						strcpy(conn_ip, " ");
						conn_port = data[i].com_speed;
					}
					if (type == 1)
					{
						DevInfo v1;
						memcpy(&v1, &data[i].info.v1, sizeof(DevInfo));
						sprintf(conn_ip, "%d.%d.%d.%d", v1.ip[0], v1.ip[1], v1.ip[2], v1.ip[3]);
						conn_port = v1.port;
					}
					else if (type == 2)
					{
						DevInfo2 v1;
						memcpy(&v1, &data[i].info.v1, sizeof(DevInfo2));
						sprintf(conn_ip, "%d.%d.%d.%d", v1.ip[0], v1.ip[1], v1.ip[2], v1.ip[3]);
						conn_port = v1.remote_udp;
					}
					else if (type == 3)
					{
						DevInfoV101 v1;
						memcpy(&v1, &data[i].info.v1, sizeof(DevInfoV101));
						sprintf(conn_ip, "%d.%d.%d.%d", v1.ip[0], v1.ip[1], v1.ip[2], v1.ip[3]);
						conn_port = v1.remote_udp;
					}
					item = cJSON_CreateString(conn_ip);
					cJSON_AddItemToObject(point, "conn_ip", item);
					item = cJSON_CreateNumber(conn_port);
					cJSON_AddItemToObject(point, "conn_port", item);

					char tmp[16] = {0};
					snprintf(tmp, 16, "%s", data[i].timeStr);
					item = cJSON_CreateString(tmp);
					cJSON_AddItemToObject(point, "time", item);

					cJSON_AddItemToObject(arr, "", point);
				}
				char *out = jsonValue("SUCCESS", "", arr);
				mg_http_reply(c, 200, "", "%s", out);
				free(out);
				return;
			}
			else if (index == 1)
			{
				BlueSeaLidarSDK::getInstance()->CloseHeartService();
				cJSON *item = cJSON_CreateArray();
				char *out = jsonValue("SUCCESS", "", item);
				mg_http_reply(c, 200, "", "%s", out);
				free(out);
				return;
			}
			else
			{
				cJSON *item = cJSON_CreateArray();
				char *out = jsonValue("ERROR", "arg is not current!", item);
				mg_http_reply(c, 200, "", "%s", out);
				free(out);
				return;
			}
		}
		else
		{
			mg_http_serve_opts opts = {0};
			opts.root_dir = s_root_dir;
			mg_http_serve_dir(c, hm, &opts);
		}
	}
	else if (ev == MG_EV_WS_MSG)
	{
		mg_ws_send(c, "OK", 2, WEBSOCKET_OP_TEXT);
	}
}

void StringReplace(std::string &strBase, std::string strSrc, std::string strDes)
{
	std::string::size_type pos = 0;
	std::string::size_type srcLen = strSrc.size();
	std::string::size_type desLen = strDes.size();
	pos = strBase.find(strSrc, pos);
	while ((pos != std::string::npos))
	{
		strBase.replace(pos, srcLen, strDes);
		pos = strBase.find(strSrc, (pos + desLen));
	}
}
