#include <unistd.h>
#include <cstdio>
#include <cstring>

#include <standard_interface.h>

void CallBackMsg( int msgtype, void *param, int length )
{
    if ( param == nullptr || length == 0 )
    {
        return;
    }
    
	switch (msgtype)
	{
        // Real-time radar point cloud data
        case 1:
        {
            UserData *pointdata = (UserData *)param;
            
            if ( pointdata->type == FRAMEDATA )
            {
                 printf( "FRAME: idx:%3d %16s %5d num:%5zu timestamp:%d.%d \r", 
                         pointdata->idx, 
                         pointdata->connectArg1, 
                         pointdata->connectArg2, 
                         pointdata->framedata.data.size(), 
                         pointdata->framedata.ts[0], 
                         pointdata->framedata.ts[1] );
#if 0
                 for ( size_t i = 0; i <pointdata->framedata.data.size(); i++ )
                 {
                    printf( "%s\t%d \t%.5f\t%.3f\t%d\n", 
                            pointdata->connectArg1, 
                            pointdata->connectArg2,  
                            pointdata->framedata.data[i].angle, 
                            pointdata->framedata.data[i].distance, 
                            pointdata->framedata.data[i].confidence );
                 }
#endif /// of 0
            }
            else
            {
                printf( "SPAN: idx:%3d %16s %5d num:%5d timestamp:%d.%d \r", 
                        pointdata->idx, 
                        pointdata->connectArg1, 
                        pointdata->connectArg2, 
                        pointdata->spandata.data.N, 
                        pointdata->spandata.data.ts[0], 
                        pointdata->spandata.data.ts[1]);
#if 0
                for ( size_t i = 0; i <pointdata->spandata.data.N; i++ )
                {
                    printf( "%s\t%d \t%.5f\t%.3f\t%d\n", 
                            pointdata->connectArg1, 
                            pointdata->connectArg2,  
                            pointdata->spandata.data.points[i].angle, 
                            pointdata->spandata.data.points[i].distance, 
                            pointdata->spandata.data.points[i].confidence );
                }
#endif /// of 0
            }
        }break;
    
        // Real-time alarm data
        case 2:
        {
            LidarMsgHdr *zone = (LidarMsgHdr *)param;
            uint32_t event = zone->events;
            std::string text;
            
            if (zone->flags % 2 == 1)
            {
                // Hardware alarm information
                if (getbit(event, 0) == 1)
                    text += "Insufficient power supply";
                if (getbit(event, 1) == 1)
                    text += "Motor stall";
                if (getbit(event, 2) == 1)
                    text += "The ranging module is overheating.";
                if (getbit(event, 3) == 1)
                    text += "Network error";
                if (getbit(event, 4) == 1)
                    text += "The ranging module has no output.";
            }
            
            if (zone->flags >= 0x100)
            {
                // Zone alarm information
                if (getbit(event, 12) == 1)
                    text += "Observe！！！";
                if (getbit(event, 13) == 1)
                    text += "Warning！！！";
                if (getbit(event, 14) == 1)
                    text += "Alarm！！！";
                if (getbit(event, 15) == 1)
                    text += "Obstruction！";
                if (getbit(event, 16) == 1)
                    text += "No data";
                if (getbit(event, 17) == 1)
                    text += "No defense zone setting";
                if (getbit(event, 18) == 1)
                    text += "Internal system error";
                if (getbit(event, 19) == 1)
                    text += "System malfunction";
                if (getbit(event, 20) == 1)
                    // This is a duplicate of the fourth item above, 
                    // so it's hidden here.
                    // text+='网络错误\n'
                    if (getbit(event, 21) == 1)
                        text += "Equipment update in progress";
                if (getbit(event, 22) == 1)
                    text += "Zero-position output";
            }
            
            if ( text.size() == 0 )
            {
                text = "unknown";
            }
            
            printf( "\nALARM: 0x%04X, [%s]\n", zone->flags, text.c_str() );
            fflush( stdout );
        }break;
        
        // Obtain global parameters of the network-enabled radar
        case 3:
        {
            EEpromV101 *eepromv101 = (EEpromV101 *)param;
            // Type, Number, Serial Number
            printf( "dev info: Device ID:%d\t Serial Number:%s\t Type:%s\n", 
                    eepromv101->dev_id, 
                    eepromv101->dev_sn, 
                    eepromv101->dev_type );
            // IP address, subnet mask, gateway address, default target IP, 
            // default target UDP port number, 
            // default UDP outbound service port number.
            char tmp_IPv4[16] = {0};
            char tmp_mask[16] = {0};
            char tmp_gateway[16] = {0};
            char tmp_srv_ip[16] = {0};

            snprintf( tmp_IPv4, 16, "%d.%d.%d.%d", \
                      eepromv101->IPv4[0], 
                      eepromv101->IPv4[1], 
                      eepromv101->IPv4[2], 
                      eepromv101->IPv4[3]);
                     
            snprintf( tmp_mask, 16, "%d.%d.%d.%d", \
                      eepromv101->mask[0], 
                      eepromv101->mask[1], 
                      eepromv101->mask[2], 
                      eepromv101->mask[3] );
                     
            snprintf( tmp_gateway, 16, "%d.%d.%d.%d", \
                      eepromv101->gateway[0], 
                      eepromv101->gateway[1], 
                      eepromv101->gateway[2], 
                      eepromv101->gateway[3] );
                      
            snprintf( tmp_srv_ip, 16, "%d.%d.%d.%d", \
                      eepromv101->srv_ip[0], 
                      eepromv101->srv_ip[1], 
                      eepromv101->srv_ip[2], 
                      eepromv101->srv_ip[3] );

            printf( "dev info: \n"
                    "   - IP address:%s Subnet mask:%s Gateway address:%s\n"
                    "   - Default target IP:%s Default target UDP port:%d\n"
                    "   - Default UDP outbound service port:%d\n",
                    tmp_IPv4, 
                    tmp_mask, 
                    tmp_gateway, 
                    tmp_srv_ip, 
                    eepromv101->srv_port, 
                    eepromv101->local_port );

            /*char tmp_ranger_bias[8] = {0};
            memcpy(tmp_ranger_bias, eepromv101->ranger_bias, sizeof(eepromv101->ranger_bias) - 1);*/
            
            // Speed, motor starting parameters,
            // FIR filter order, revolutions, resolution, 
            // automatic upload upon startup, fixed upload, 
            // data point smoothing, dragging point removal, 
            // recording correction coefficients, network heartbeat, 
            // recording I/O port polarity.
            printf( "dev info \n"
                    "   - Speed (RPM): %d\n"
                    "   - Motor starting parameters: %d \n"
                    "   - FIR filter order: %d \n"
                    "   - Number of revolutions: %d \n"
                    "   - Resolution:%d \n"
                    "   - Automatic upload on boot:%d \n"
                    "   - Fixed upload:%d \n"
                    "   - Data point smoothing:%d \n"
                    "   - Remove dragging points:%d \n"
                    "   - Network heartbeat:%d \n"
                    "   - Record I/O port polarity:%d\n",
                    eepromv101->RPM, 
                    eepromv101->RPM_pulse, 
                    eepromv101->fir_filter, 
                    eepromv101->cir, 
                    eepromv101->with_resample, 
                    eepromv101->auto_start,
                    eepromv101->target_fixed, 
                    eepromv101->with_smooth, 
                    eepromv101->with_filter, 
                    eepromv101->net_watchdog, 
                    eepromv101->pnp_flags);

            printf( "dev info \n"
                    "   - Smoothing coefficient: %d \n"
                    "   - Activated zone: %d \n"
                    "   - Uploaded data type: %d\n", 
                    eepromv101->deshadow, 
                    eepromv101->zone_acted, 
                    eepromv101->should_post);
            fflush( stdout );
        }break;

        // Obtain radar timestamp print information 
        // (for network versions, the radar-returned timestamp; 
        //  for serial versions, the timestamp received by the local machine).
        case 4:
        {
            DevTimestamp *devtimestamp = (DevTimestamp *)param;
            printf( "\n" );
            printf( "\nTIMESTAMP:lidar_ip:%s lidar_port:%d time:%d delay:%d\n", 
                    devtimestamp->ip, 
                    devtimestamp->port, 
                    devtimestamp->timestamp, 
                    devtimestamp->delay);
            fflush( stdout );
        }break;
        
        // Print information (which can also be written as a log).
        case 8:
        {
            std::string istr = (const char*)param;
            printf( "\nINFO: %s\n", istr.c_str() );
            fflush( stdout );
        }break;
        
        case 9:
        {
            std::string estr = (const char*)param;
            printf( "\nERROR: %s\n", estr.c_str() );
            fflush( stdout );
        }break;
	}
}

int main(int argc, char **argv)
{
	if (argc < 2)
	{
		printf( "Incorrect number of parameters  %d\n"
                "\t usage : %s  config/xxx.txt\n"
                " -- At least one txt of Lidar\n", 
                argc, argv[0] );
		return -1;
	}
    
	BlueSeaLidarSDK *lidarSDK =  BlueSeaLidarSDK::getInstance();
    if ( lidarSDK == NULL )
    {
        return -1;
    }
    
	size_t lidar_sum = (size_t)argc - 1;
    int lidarID = -1;
    
	for (size_t i = 0; i < lidar_sum; i++)
	{
		const char *cfg_file_name = argv[i + 1];
		// Add the relevant radar according to the configuration file path.
		lidarID = lidarSDK->AddLidarByPath(cfg_file_name);
		if (!lidarID)
		{
			printf("config file is not exist:%s\n", cfg_file_name);
			return -1;
		}

		// Callback function for passing in data information
		lidarSDK->SetCallBackPtr(lidarID, CallBackMsg);

		// Connect to the specified radar and the associated thread pool.
		if (!lidarSDK->OpenDev(lidarID))
		{
			printf("open lidar failed:%d\n", lidarID);
			return -2;
		}
        
		printf("SDK version:%s\n", lidarSDK->GetVersion());
	}
    
    // Wait for device is closed ...
	while( lidarSDK != nullptr )
	{
        int state = lidarSDK->GetDevState( lidarID );
        
        if ( state == OFFLINE )
        {
            printf( "!!!!!!state changed to OFFLINE.\n" );
            fflush( stdout );
            break;
        }
        
        usleep( 10 );
	}
    
	return 0;
}
