#include <unistd.h>
#include <cstdio>
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

#ifdef _WIN32
#include <resource.h>
#endif /// of _WIN32

#define DEBUG_PRINT_DATA    0
#define RADF_1              (0.01745329)
#define RADF_MAX            (6.2831853072)
#define RADF_FIX_90DGR      (1.570796)

typedef struct _thread_param
{
    int argc;
    char** argv;
}thread_param;

BlueSeaLidarSDK* lidarSDK = BlueSeaLidarSDK::getInstance();

Fl_Double_Window*   flWindow    = nullptr;
Fl_Box*             flRndrBox   = nullptr;
Fl_RGB_Image*       flRndrImg   = nullptr;
Fl_Box*             flStatBox   = nullptr;
Fl_Box*             flAlarmBox  = nullptr;

pthread_mutex_t     pmtxWait    = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t      pcondData   = PTHREAD_COND_INITIALIZER;
bool                bPrtCbMsg   = false;
bool                bUWTitle    = false;
uint64_t            lAlarmTm    = 0;

// static 360.0 degree values.
float               data360[3600] = { -1.f };
float               rpmHz       = 0.f;
float               rotCnt      = 0.f;
uint64_t            lRotMsTm    = 0;

uint64_t getMonoTick()
{
    timespec ts;
    clock_gettime( CLOCK_MONOTONIC, &ts );

    return (ts.tv_sec * 1000) + (ts.tv_nsec / 1000000);    
}

void fl_wcb( Fl_Widget* w, void* p )
{
    if ( w == flWindow )
    {
        w->hide();
    }
}

inline void flushData360()
{
    #pragma omp parallel for
    for( size_t cnt=0; cnt<3600; cnt++)
    {
        data360[cnt] = -1.f;
    }
}

void clearRndrBack( bool dolock = false )
{
    if ( dolock == true )
    {
        Fl::lock();
    }
    
    Fl_RGB_Image* dstImg = (Fl_RGB_Image*)flRndrBox->image();
    
    uint32_t col_bg = 0x010203FF;
    uint32_t col_ln = 0x30373FFF;
    uint32_t col_p0 = 0x37303FFF;
    uint32_t col_p1 = 0x3F3037FF;
    
    // clear background
    fl_imgtk::draw_fillrect( dstImg, 0, 0, 
                             dstImg->w(),
                             dstImg->h(),
                             col_bg );
                             
    // -------
    fl_imgtk::draw_smooth_line_ex( dstImg,
                                   0, dstImg->h() / 2,
                                   dstImg->w(), dstImg->h() / 2,
                                   0.5f, col_ln );
    size_t cnts = size_t(dstImg->w()/2)+5;
    #pragma omp parallel for
    for( size_t cnt=cnts; cnt<(size_t)dstImg->w(); cnt+=10 )
    {
        float ltf = 0.5f;
        size_t cntc = cnt - cnts;
        if ( cntc%50 == 0 && cnt != cnts ) ltf = 2.5f;        
        
        fl_imgtk::draw_smooth_line_ex( dstImg,
                                       cnt, dstImg->h() / 2 + 1,
                                       cnt, dstImg->h() / 2 + 11,
                                       ltf, col_ln );  
    }
                                   
    // - -|- -   
    fl_imgtk::draw_smooth_line_ex( dstImg,
                                   dstImg->w() / 2, 0,
                                   dstImg->w() / 2, dstImg->h(),
                                   0.5f, col_ln );
    #pragma omp parallel for
    for( size_t cnt=0; cnt<(size_t)(dstImg->h()/2); cnt+=10 )
    {
        float ltf = 0.5f;
        if ( cnt%50 == 0 && cnt != 0 ) ltf = 2.5f;
        
        fl_imgtk::draw_smooth_line_ex( dstImg,
                                       dstImg->w() / 2 + 1, cnt,
                                       dstImg->w() / 2 + 11, cnt,
                                       ltf, col_ln );  
    }


    #pragma omp parallel for
    for( size_t cnt=0; cnt<360; cnt+=5 )
    {
        float distancef = (float)(dstImg->w() / 3 );
        float linedistf = 50.f;
        float radf = (float)cnt * ( PI / 180.f ) - RADF_FIX_90DGR;
        
        float cntrX = dstImg->w() / 2;
        float cntrY = dstImg->h() / 2;
        float recX[2] = {0.f,0.f};
        float recY[2] = {0.f, 0.f};
        recX[0] = cntrX + distancef * cos( radf );
        recX[1] = cntrX + ( distancef + linedistf ) * cos( radf );    
        recY[0] = cntrY + distancef * sin( radf );
        recY[1] = cntrY + ( distancef + linedistf ) * sin( radf );

        unsigned point_col = col_p0;
        float lthf = 1.f;
        
        if ( cnt % 30 == 0 )
        {
            point_col = col_p1;
            lthf = 5.0f;
        }
        
        fl_imgtk::draw_smooth_line_ex( dstImg,
                                      (unsigned)recX[0], (unsigned)recY[0],
                                      (unsigned)recX[1], (unsigned)recY[1],
                                      lthf,
                                      point_col );
    }
    
    dstImg->uncache();
    
    if ( dolock == true )
    {
        Fl::unlock();
    }
}

/**
 * Maps a float (0.0 to 1.0) to a 24-bit RGB integer (0xRRGGBB00)
 * Uses a standard Hue-based rainbow spectrum.
 */
uint32_t distanceToColor(float distf) 
{
    // filter distf.
    if ( distf > 1.f ) { distf = 1.f; }
    else if ( distf < 0.f ) { distf = 0.f; }
    
    // Convert 0.0-1.0 to 0-360 degrees (Hue)
    float h = distf * 360.0f;
    float s = 1.0f; /// Full Saturation
    float v = 1.0f; /// Full Value/Brightness

    float c = v * s;
    float x = c * (1.0f - std::abs(std::fmod(h / 60.0f, 2.0f) - 1.0f));
    float m = v - c;

    float r, g, b;
    if (h < 60)      { r = c; g = x; b = 0; }
    else if (h < 120) { r = x; g = c; b = 0; }
    else if (h < 180) { r = 0; g = c; b = x; }
    else if (h < 240) { r = 0; g = x; b = c; }
    else if (h < 300) { r = x; g = 0; b = c; }
    else             { r = c; g = 0; b = x; }

    uint8_t R = static_cast<uint8_t>((r + m) * 255);
    uint8_t G = static_cast<uint8_t>((g + m) * 255);
    uint8_t B = static_cast<uint8_t>((b + m) * 255);

    // Combine into 0xRRGGBB00
    return (R << 24) | (G << 16) | ( B << 8 );
}

float getDist( int32_t degIdx )
{
    float retf = 0.f;
    float divf = 0.f;
    
    if ( degIdx < 3600 && degIdx >= 0 )
    {
        for( size_t x=0; x<10; x++ )
        {
            float curf = data360[degIdx+x];
            if ( curf > 0.f )
            {
                retf += curf;
                divf += 1.f;
            }
        }
        
        if ( divf > 0.f )
        {
            retf /= divf;
        }
    }
    
    return retf;
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
            
            float min_radf = pointdata->spandata.data.points[0].angle;
            float max_radf = pointdata->spandata.data.points[pointdata->spandata.data.N-1].angle;

            // merge datas 360.0
            #pragma omp parallel for shared(data360)
            for ( size_t i = 0; i <pointdata->spandata.data.N; i++ )
            {
                float distancef = pointdata->spandata.data.points[i].distance;
                float radf = pointdata->spandata.data.points[i].angle;
                int   degf = ( radf * (180.f / PI) ) * 10.f;
                degf %= 3600;

                // simple but some dangerous merge method.
                // has some error, but good to look.
                if ( data360[degf] > 0.f )
                {
                    data360[degf] += distancef;
                    data360[degf] /= 2.f;
                }
                else
                {
                    data360[degf] = distancef;
                }
            }

            char tmpStr[160] = {0};
            snprintf( tmpStr, 160, 
                      "SPAN [%11d:%5d] @@%11d.%d\n"
                      "Rendering RPM = %.3f (%.3f Hz)", 
                      pointdata->idx, 
                      pointdata->spandata.data.N,
                      pointdata->spandata.data.ts[0], 
                      pointdata->spandata.data.ts[1],
                      rpmHz * 60.f, rpmHz );
                      
            if ( bUWTitle == false )
            {
                char tmpWinTitle[80] = {0};
                snprintf( tmpWinTitle, 80, "Device: %s:%d",
                          pointdata->connectArg1, 
                          pointdata->connectArg2 );
                flWindow->copy_label( tmpWinTitle );
                bUWTitle = true;
            }

            Fl::lock();

            clearRndrBack();

            flStatBox->copy_label( tmpStr );
            flStatBox->redraw();
            
            Fl_RGB_Image* dstImg = (Fl_RGB_Image*)flRndrBox->image();
            
            // scaling X, Y .. (acutally meaningless)
            float cntrX = (float)dstImg->w() / 2;
            float cntrY = (float)dstImg->h() / 2;
            float putCW = (float)dstImg->w() * 0.3f;
            
            // then go with OpenMP.
            #pragma omp parallel for
            for ( size_t i = 0; i <3600; i+=10 )
            {
                float scalef = 1.f; /// it should be adjustabled, current = 80 %.
                float distfc = getDist(i) * putCW * scalef;
                float distfn = getDist((i+10)%3600) * putCW * scalef;
                
                if ( distfc > 0.05f && distfn > 0.05f )
                {
                    float deg10pf = (float)i;
                    float deg10nf = (float)((i+10)%3600);
                    float radfc = deg10pf/10.f * ( PI / 180.f) - RADF_FIX_90DGR;
                    float radfn = deg10nf/10.f * ( PI / 180.f) - RADF_FIX_90DGR;
                    float recX = cntrX + distfc * -cos( radfc );
                    float recY = cntrY + distfc * sin( radfc );
                    float nxtX = cntrX + distfn * -cos( radfn );
                    float nxtY = cntrX + distfn * sin( radfn );
                                        
                    if ( recX <= 0 ) recX = dstImg->w() / 2;
                    if ( recY <= 0 ) recY = dstImg->h() / 2;
                    
                    // Colorful RGB -> B to G to R ...
                    float distaf = ( distfc + distfn ) / 2.f;
                    float distnf = distaf / 1000.f;
                    if ( distnf > 1.0f ) distnf = 1.0f;
                    
                    // convert normalized distance to HSV color spaced RGB.
                    uint32_t point_col = distanceToColor( distnf );
                    uint32_t point_half_col = point_col;
                    
                    float max_conf = 128.f;
                    float conff = pointdata->spandata.data.points[i].confidence / max_conf;
                    if ( conff > 1.f ) conff = 1.f;
                    
                    uint8_t conf8 = (uint8_t)(conff * 128.f);
                    point_col |= (uint32_t)(conf8 + 0x70);
                    point_half_col |= (uint32_t)(conf8/2);
                    
                    // filter out errors...
                    bool bIgnore = false;
                    
                    if ( recX == cntrX || recY == cntrY )
                    {
                        bIgnore = true;
                    }

                    if ( bIgnore == false )
                    {
                        fl_imgtk::draw_smooth_line_ex( dstImg,
                                                      (int)recX,
                                                      (int)recY,
                                                      (int)nxtX, 
                                                      (int)nxtY,
                                                      3.1f,
                                                      point_col );
                    }
                } /// of if ( distancef > 0.0f )
            }  /// of for() with openmp

            // Draw lidar sight triangle.
            float trifarf = (float)(flRndrBox->w()) * 0.4f;
            fl_imgtk::vecpoint sightVectors[] = 
            {
                (int)cntrX, 
                (int)cntrY,
                (int)(cntrX + trifarf * -cos( min_radf )),
                (int)(cntrY + trifarf * sin( min_radf )),
                (int)(cntrX + trifarf * -cos( max_radf )),
                (int)(cntrY + trifarf * sin( max_radf )),
                0, 0,
            };
            fl_imgtk::draw_polygon( dstImg, sightVectors, 3, 0x33663360 );
            
            // un-managed rotattion metering ...
            if ( ( min_radf > 4.537856f && max_radf < 5.585054f )
                 || ( min_radf >= 0.0f && max_radf < 0.7853982f ) )
            {
                rotCnt += 1.0f;
            }
            
            dstImg->uncache();
            flRndrBox->redraw();
            flWindow->redraw();
            Fl::unlock();
            Fl::awake();
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
                // unkown alarm should be ignored.
                break;
            }

            char tmpStr[80] = {0};
            snprintf( tmpStr, 80, "ALARM: 0x%04X, [%s]", zone->flags, text.c_str() );
            Fl::lock();
            flAlarmBox->copy_label( tmpStr );
            flAlarmBox->redraw();
            Fl::unlock();
            lAlarmTm = zone->timestamp;
        }break;
        
        case 4: /// time update ...
        {
            // Update Hz ..
            if ( rotCnt > 0.f )
            {
                uint64_t curTm = getMonoTick();

                if ( lRotMsTm == 0 )
                {
                    lRotMsTm = curTm;
                }
                else
                {
                    uint64_t tmDiff = curTm - lRotMsTm;
                    float    rotf = (float)tmDiff / rotCnt;
#ifdef DEBUG_RPM_EMULATION
                    printf( "(debug) RPM, rotate count = %.2f, time diff = %llu ms --> %.3f\n",
                            rotCnt, tmDiff, rotf );
                    fflush( stdout );
#endif /// of DEBUG_RPM_EMULATION
                    rpmHz = 1000.f / rotf;
                    rotCnt = 0.f;
                    lRotMsTm = curTm;
                }
            }
            if ( lAlarmTm > 0 )
            {
			    uint64_t curTm = getMonoTick();
                
                if ( lAlarmTm < ( curTm - 1000 ) )
                {
                    Fl::lock();
                    flAlarmBox->label( nullptr );
                    Fl::unlock();
                }
            }
        }break;
                    
        // Print information (which can also be written as a log).
        case 8:
        {
            char tmpStr[80] = {0};                        
            snprintf( tmpStr, 80, "INFO: %s", (const char*)param );

            flStatBox->copy_label( tmpStr );
            flStatBox->redraw();
        }break;
        
        case 9:
        {
            char tmpStr[80] = {0};                        
            snprintf( tmpStr, 80, "ERROR: %s", (const char*)param );

            Fl::lock();
            flAlarmBox->copy_label( tmpStr );
            flAlarmBox->redraw();
            Fl::unlock();
            
            LidarMsgHdr *zone = (LidarMsgHdr *)param;
            lAlarmTm = zone->timestamp;
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
                                     "PaceCat LiDAR LDS-50C-E FLTK demo | (C)2026 Raph.K." );
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
                clearRndrBack();
            }
        }
        
        // overlay status text printing box.
        flStatBox = new Fl_Box( flRndrBox->x(), flRndrBox->y(),
                                flRndrBox->w(), 50,
                                "Connecting ... " );
        if ( flStatBox != nullptr )
        {
            flStatBox->box( FL_NO_BOX );
            flStatBox->align( FL_ALIGN_INSIDE | FL_ALIGN_TOP_LEFT );
            flStatBox->labelfont( FL_COURIER );
            flStatBox->labelsize( 20 );
            flStatBox->labelcolor( 0xCCFFCC00 );
        }

        // overlay status text printing box.
        flAlarmBox = new Fl_Box( flRndrBox->x(), flRndrBox->h() - 50,
                                 flRndrBox->w(), 50 );
        if ( flAlarmBox != nullptr )
        {
            flAlarmBox->box( FL_NO_BOX );
            flAlarmBox->align( FL_ALIGN_INSIDE | FL_ALIGN_BOTTOM_RIGHT );
            flAlarmBox->labelfont( FL_COURIER );
            flAlarmBox->labelsize( 20 );
            flAlarmBox->labelcolor( 0xFF333300 );
        }
        
        flWindow->end();
        flWindow->callback( fl_wcb, nullptr );
        flWindow->show();
#ifdef _WIN32 
        // Additional windows dep. resouce handle ...
        HICON hIcoL = (HICON)LoadImage( fl_display,
                                        MAKEINTRESOURCE( IDC_ICON_A ),
                                        IMAGE_ICON,
                                        64, 64, LR_SHARED );
     
        HICON hIcoS = (HICON)LoadImage( fl_display,
                                        MAKEINTRESOURCE( IDC_ICON_A ),
                                        IMAGE_ICON,
                                        16, 16, LR_SHARED );
        flWindow->icons( hIcoL, hIcoS );
#endif /// of _WIN32
        
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
			fprintf( stderr, "config file is not exist:%s\n", cfg_file_name);
			return nullptr;
		}

		// Callback function for passing in data information
		lidarSDK->SetCallBackPtr(lidarID, CallBackMsg);

		// Connect to the specified radar and the associated thread pool.
		if (!lidarSDK->OpenDev(lidarID))
		{
			fprintf( stderr, "open lidar failed:%d\n", lidarID);
			return nullptr;
		}        
	}
    
    printf( "\n" );
    fflush( stdout );
    
    clearRndrBack();
    
    bPrtCbMsg = true;
    
    // Wait for device is closed ...
	while( lidarSDK != nullptr )
	{
        int state = lidarSDK->GetDevState( lidarID );
        
        if ( state == OFFLINE )
        {
            break;
        }
#if ENABLED_HEARTBEAT_DETECT_RPM
        else
        if ( rpmHz == 0.f )
        {
            std::vector<DevConnInfo> devList = lidarSDK->GetLidarsList();
            if ( devList.size() > 0 )
            {
                printf( "(debug)devList = %zu\n", devList.size() );
                fflush( stdout );
                rpmHz = devList[0].info.v101.rpm / 60.f;
            }            
        }
#endif /// of ENABLED_HEARTBEAT_DETECT_RPM

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
