#include <unistd.h>#include <cstdio>
#include <cstring>
#include <cmath>
#include <omp.h>

#include <standard_interface.h>

#include <FL/platform.H>
#include <FL/Fl.H>
#include <FL/Fl_Box.H>
#include <FL/Fl_Double_Window.H>
#include <FL/Fl_RGB_Image.H>

#include <fl_imgtk.h>

#define DEBUG_PRINT_DATA    0
#define RADF_MAX            (6.2831853072)

typedef struct _thread_param
{
    int argc;
    char** argv;
}thread_param;

BlueSeaLidarSDK* lidarSDK = BlueSeaLidarSDK::getInstance();

Fl_Double_Window*   flWindow    = nullptr;
Fl_Box*             flRndrBox   = nullptr;
Fl_RGB_Image*       flRndrImg   = nullptr;

pthread_mutex_t     pmtxWait    = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t      pcondData   = PTHREAD_COND_INITIALIZER;
bool                bPrtCbMsg   = false;

void fl_wcb( Fl_Widget* w, void* p )
{
    if ( w == flWindow )
    {
        w->hide();
    }
}

void clearRndrBack( bool dolock = false )
{
    if ( dolock == true )
    {
        Fl::lock();
    }
    
    Fl_RGB_Image* dstImg = (Fl_RGB_Image*)flRndrBox->image();
    
    // clear background
    fl_imgtk::draw_fillrect( dstImg, 0, 0, 
                             dstImg->w(),
                             dstImg->h(),
                             0x223344FF );
                             
    // -------
    fl_imgtk::draw_smooth_line_ex( dstImg,
                                   0, dstImg->h() / 2,
                                   dstImg->w(), dstImg->h() / 2,
                                   1.5f, 0x44FF44FF );
    // - -|- -   
    fl_imgtk::draw_smooth_line_ex( dstImg,
                                   dstImg->w() / 2, 0,
                                   dstImg->w() / 2, dstImg->h(),
                                   1.5f, 0x44FF44FF );
    #pragma omp parallel for                        
    for( size_t cnt=0; cnt<360; cnt+=4 )
    {
        float distancef = (float)(dstImg->w() / 3 );
        float linedistf = 50.f;
        float radf = (float)cnt * ( PI / 180.f );
        
        float cntrX = dstImg->w() / 2;
        float cntrY = dstImg->h() / 2;
        float recX[2] = {0.f,0.f};
        float recY[2] = {0.f, 0.f};
        recX[0] = cntrX + distancef * cos( radf );
        recX[1] = cntrX + ( distancef + linedistf ) * cos( radf );    
        recY[0] = cntrY + distancef * sin( radf );
        recY[1] = cntrY + ( distancef + linedistf ) * sin( radf );

        unsigned point_col = 0x33CC338F;
        
        fl_imgtk::draw_smooth_line( dstImg,
                                    (unsigned)recX[0], (unsigned)recY[0],
                                    (unsigned)recX[1], (unsigned)recY[1],
                                    point_col );
    }
    
    dstImg->uncache();
    
    if ( dolock == true )
    {
        Fl::unlock();
    }
}

void CallBackMsg( int msgtype, void *param, int length )
{
    if ( param == nullptr || length == 0 )
    {
        return;
    }
    
    if ( bPrtCbMsg == false )
    {
        return;
    }
    
    if ( flWindow->shown() == 0 )
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
#ifdef DEBUG_STATE
                 printf( "FRAME: idx:%3d %16s %5d num:%5zu timestamp:%d.%d \n", 
                         pointdata->idx, 
                         pointdata->connectArg1, 
                         pointdata->connectArg2, 
                         pointdata->framedata.data.size(), 
                         pointdata->framedata.ts[0], 
                         pointdata->framedata.ts[1] );
#endif /// of DEBUG_STATE
                 clearRndrBack( true );
            }
            else
            {
#ifdef DEBUG_STATE                
                printf( "SPAN: idx:%3d %16s %5d num:%5d timestamp:%d.%d \r", 
                        pointdata->idx, 
                        pointdata->connectArg1, 
                        pointdata->connectArg2, 
                        pointdata->spandata.data.N, 
                        pointdata->spandata.data.ts[0], 
                        pointdata->spandata.data.ts[1]);
#endif /// of DEBUG_STATE
                Fl::lock();
                Fl_RGB_Image* dstImg = (Fl_RGB_Image*)flRndrBox->image();
                
                // scaling X, Y .. (acutally meaningless)
                float cntrX = (float)dstImg->w() / 2;
                float cntrY = (float)dstImg->h() / 2;
                float putCX = (float)dstImg->w() * 0.3f;
                float putCY = (float)dstImg->h() * 0.3f;

                // then go !
                #pragma omp parallel for
                for ( size_t i = 0; i <pointdata->spandata.data.N; i++ )
                {
                    // what is PaceCat's zero degree ????
                    float distancef = pointdata->spandata.data.points[i].distance
                                      * putCX;
                    float radf = pointdata->spandata.data.points[i].angle;
                    float recX = cntrX + distancef * cos( radf );
                    float recY = cntrY + distancef * sin( radf );
                                        
                    if ( recX <= 0 ) recX = dstImg->w() / 2;
                    if ( recY <= 0 ) recY = dstImg->h() / 2;
                    
                    unsigned point_col = 0xFF444400;
                    
                    float max_conf = 400.f;
                    float conff = pointdata->spandata.data.points[i].confidence / max_conf;
                    if ( conff > 1.f ) conff = 1.f;
                    
                    uint8_t conf8 = (uint8_t)(conff * 255.f);
                    point_col |= (uint32_t)conf8;

                    fl_imgtk::draw_smooth_line( dstImg,
                                                (unsigned)recX - 2, (unsigned)recY - 2,
                                                (unsigned)recX + 2, (unsigned)recY + 2,
                                                point_col );
                    fl_imgtk::draw_smooth_line( dstImg,
                                                (unsigned)recX + 2, (unsigned)recY - 2,
                                                (unsigned)recX - 2, (unsigned)recY + 2,
                                                point_col );
                }

                dstImg->uncache();
                flRndrBox->redraw();
                flWindow->redraw();
                Fl::unlock();
                Fl::awake();
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
 #if PRT_PROGRESS
            printf( "\n" );
            printf( "\nTIMESTAMP:lidar_ip:%s lidar_port:%d time:%d delay:%d\n", 
                    devtimestamp->ip, 
                    devtimestamp->port, 
                    devtimestamp->timestamp, 
                    devtimestamp->delay);
            fflush( stdout );
#endif /// of PRT_PROGRESS
            clearRndrBack( true );
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

int initGUI()
{
    Fl::lock();
    Fl::scheme( "flat" );
    Fl_Window::default_xclass( "fltklidardemo" );
    Fl::set_color( FL_GRAY, 0x33333300 );

    flWindow = new Fl_Double_Window( 810, 810, 
                                     "PaceCat LiDAR LC50D-E FLTK demo | (C)2026 Raph.K." );
    if ( flWindow != nullptr )
    {
        flWindow->begin();
        flWindow->color( 0x33333300 );
        flWindow->labelcolor( 0xEEEEEE00 );
        
        flRndrBox = new Fl_Box( 3, 3, 804, 804 );
        if ( flRndrBox != nullptr )
        {
            flRndrBox->box( FL_NO_BOX );
            flRndrBox->align( FL_ALIGN_INSIDE );
            flRndrImg = fl_imgtk::makeanempty( 800, 800, 3, 0x111122FF );
            if ( flRndrImg != nullptr )
            {
                flRndrBox->image( flRndrImg );
            }
        }
        
        flWindow->end();
        flWindow->callback( fl_wcb, nullptr );
        flWindow->show();
        
        return 0;
    }
    
    return -1;
}

void* testRun( void* p )
{
    usleep( 1000000 );
    
    if ( lidarSDK == NULL )
    {
        return nullptr;
    }
    
    thread_param* tp = (thread_param*)p;
    
	size_t lidar_sum = (size_t)tp->argc - 1;
    int lidarID = -1;
    
	for (size_t i = 0; i < lidar_sum; i++)
	{
		const char *cfg_file_name = tp->argv[i + 1];
		// Add the relevant radar according to the configuration file path.
		lidarID = lidarSDK->AddLidarByPath(cfg_file_name);
		if (!lidarID)
		{
			printf("config file is not exist:%s\n", cfg_file_name);
			return nullptr;
		}

		// Callback function for passing in data information
		lidarSDK->SetCallBackPtr(lidarID, CallBackMsg);

		// Connect to the specified radar and the associated thread pool.
		if (!lidarSDK->OpenDev(lidarID))
		{
			printf("open lidar failed:%d\n", lidarID);
			return nullptr;
		}
        
		printf("SDK version:%s\n", lidarSDK->GetVersion());
	}
    
    printf( "\n" );
    fflush( stdout );
    
    clearRndrBack();
    
    bPrtCbMsg = true;
    
    // Wait for device is closed ...
	while( lidarSDK != nullptr )
	{
        /*
        pthread_mutex_lock( &pmtxWait );
        timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec += 1;
        int rc = pthread_cond_timedwait( &pcondData, &pmtxWait, &ts );
        if ( rc == 0 )
        {
        }
        pthread_mutex_unlock( &pmtxWait );
        */
        
        // === 
        int state = lidarSDK->GetDevState( lidarID );
        
        if ( state == OFFLINE )
        {
            break;
        }
        
        usleep( 100000 );
        Fl::awake();
	}
        
    pthread_exit( nullptr );
    return nullptr;
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
 
    // init GUI ...
    initGUI();
    
    thread_param tp { argc, argv };
    pthread_t ptGUI;
    if ( pthread_create( &ptGUI, nullptr, testRun, &tp ) == 0 )
    {
        return Fl::run();
    }
    
    pthread_join( ptGUI, nullptr );

	return 0;
}
