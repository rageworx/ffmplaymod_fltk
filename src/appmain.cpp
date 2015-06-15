#ifdef _WIN32
    #include <windows.h>
    #include "resource.h"
#endif /// of _WIN32

#include "appmain.h"

#include <FL/Fl_PNG_Image.H>

#define APP_LOAD_CHECK_TIMER_F  10.0
#define APP_SKIP_TIMER_F        1.0

static void appLoadCheckTimer( void* p )
{
    appMain* am = (appMain*)p;

    if ( am != NULL )
    {
        if ( ( am->GetMediaLoaded() == true ) && ( am->GetMediaReadched() == false ) )
        {
            am->SetMediaReached( false );
        }
    }

    Fl::remove_timeout( appLoadCheckTimer, p );
}

static void appSkipTimer( void* p )
{
    Fl::remove_timeout( appSkipTimer, p );

    appMain* am = (appMain*)p;

    if ( am != NULL )
    {
        if ( ( am->GetMediaLoaded() == true ) && ( am->GetMediaReadched() == true ) )
        {
#ifdef DEBUG
            printf("#PLAYER.CONTROL#\n");
            printf("\tSkipping frames for sync w/ server.\n");
            fflush(stdout);
#endif
            am->Resync();
        }
    }
}

static void btnCB( Fl_Widget* w, void* p )
{
    if ( ( w != NULL ) && ( p != NULL ) )
    {
        appMain* am = (appMain*)p;
        am->StartingStream();
    }
}

static void inpCB( Fl_Widget* w, void* p )
{
    if ( ( w != NULL ) && ( p != NULL ) )
    {
        appMain* am = (appMain*)p;
        am->Connect();
    }
}

static void winCB( Fl_Widget* w, void* p )
{
    Fl_Window* fdw = (Fl_Window*)w;

    if ( fdw != NULL )
    {
        fdw->hide();
    }
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

rtspWindow::rtspWindow(int W, int H, const char *l = 0)
 : Fl_Double_Window(W,H,l),
   _eventh(NULL)
{

}

rtspWindow::~rtspWindow()
{

}

int rtspWindow::handle(int e)
{
    int reti = Fl_Double_Window::handle(e);

    if ( _eventh != NULL )
    {
        return _eventh->event( this, e );
    }

    return reti;
}

void rtspWindow::resize(int x, int y, int w, int h)
{
    Fl_Double_Window::resize( x, y, w, h );

    if ( _eventh != NULL )
    {
        return _eventh->event( this, FL_FULLSCREEN + 10 );
    }

}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

appMain::appMain( int argc, char** argv )
 : _argc(argc),
   _argv(argv),
   winMe(NULL),
   blMutexer(false),
   bInitFFplay(false),
   bMediaLoaded(false),
   bMediaReached(false),
   bPaused(false),
   bFlushingReady(false),
   framecount(0),
   frameskip(0),
   info_frame(0),
   prev_keytime(0),
   wait_ms(0),
   vid_state(NULL),
   audio_proc(NULL),
   userURL(NULL)
{
    createComponents();

    audio_proc = new AudioProc();
    if ( audio_proc != NULL )
    {
        audio_proc->reqcb( this );
    }

    if ( ffplaymod_init( argc, argv ) == 0 )
    {
        bInitFFplay = true;
    }

    // Check "RTSP" server address ...
    for( int cnt=1; cnt<argc; cnt++ )
    {
        char* strp = strstr( argv[cnt], "rtsp://" );
        if ( strp != NULL )
        {
            userURL = strp;
        }
    }

    wait_ms = 1;
}

appMain::~appMain()
{
    Fl::remove_timeout( appSkipTimer, this );
    Fl::remove_timeout( appLoadCheckTimer, this );

    if ( audio_proc != NULL )
    {
        delete audio_proc;
    }

    if ( bInitFFplay == true )
    {
        ffplaymod_final();
    }
}

int appMain::SetParams(int argc, const char** argv )
{
}

int appMain::Run()
{
    return Fl::run();
}

void appMain::setRenderSize(int w, int h)
{
    int mw = Fl::w();
    int mh = Fl::h();

    int bw = realborder_x * 2;
    int bh = realborder_y + realborder_x;

    if ( ( (w+bw) == mw ) || ( (h+bh) == mh ) )
    {
        winMe->position( realborder_x, realborder_y );
    }
    else
    if ( ( w > mw ) || ( h > mh ) )
    {
        winMe->position(realborder_x, realborder_y );
        winMe->size( mw - bw, mh - bh );
    }
    else
    {
        winMe->size( w, h );

        if ( ( winMe->x() + realborder_x + w > mw ) || ( winMe->y() + realborder_y + h > mh ) )
        {
            winMe->position( realborder_x, realborder_y );
        }
    }

    if ( boxRenderer != NULL )
    {
        boxRenderer->size( w, h );

        int new_w = w;
        int new_h = boxBench->labelsize() * 5;

        boxBench->size( new_w, new_h );
        boxBench->label("Waiting for video signal ...");

        boxInfoBg->size( w, h );
        boxInfo->size( w, h );
        boxLic->position(0, h - boxLic->h() );

        winMe->size_range( w / 3 ,
                           h / 3,
                           0,
                           0,
                           0,
                           0,
                           w / h );

        bFlushingReady = true;
    }

    winMe->redraw();
}

bool appMain::putImage( Fl_RGB_Image* i )
{
#ifdef _WIN32
    static unsigned long fps_frame = 0;
    static unsigned long fps = 0;
    static unsigned long prevtick = 0;
    static unsigned long curtick  = 0;
#endif

    if ( blMutexer == true )
    {
        frameskip++;
        return false;
    }

    if ( boxRenderer != NULL )
    {
        Fl::lock();

        blMutexer = true;

        Fl_RGB_Image* prevImg = boxRenderer->image();

        boxRenderer->image( NULL );

        if ( prevImg != NULL )
        {
            if ( ( prevImg->alloc_array == false ) && ( prevImg->array != NULL ) )
            {
                delete[] prevImg->array;
            }

            delete prevImg;
        }

        boxRenderer->image( i );
        boxRenderer->redraw();

        framecount++;

        if ( info_frame > 0 )
        {
            if ( info_frame <= framecount + 60 )
            {
                boxInfo->hide();
                boxInfoBg->hide();
                info_frame = 0;
            }
        }

#ifdef _WIN32
        if ( ( prevtick == 0 ) && ( curtick == 0 ) )
        {
            prevtick  = GetTickCount();
            curtick   = prevtick;
            fps_frame = framecount;
        }
        else
        {
            curtick = GetTickCount();
            if ( curtick - prevtick >= 1000 )
            {
                fps = framecount - fps_frame;
                prevtick = curtick;
                fps_frame = framecount;
            }
        }
#endif

        static char outp[300] = {0};
        sprintf(outp, "FLTK RTSP player sample, Raphael Kim (rageworx@@gmail.com)\n"
                      "  - drawn              : %llu\n"
#ifdef _WIN32
                      "  - skipped(ignored)   : %llu\n"
                      "  - flushed per second : %ld"
#else
                      "  - skipped(ignored)   : %llu"
#endif
                      , framecount
#ifdef _WIN32
                      , frameskip
                      , fps );
#else
                      , frameskip );
#endif

        boxBench->label( outp );

        blMutexer = false;

        //Fl::wait();
        Fl::flush();
        Fl::unlock();

        return true;
    }

    return false;
}

void appMain::setFullScreen()
{

}

int appMain::event(void *p, int i)
{
    if ( bMediaLoaded == false )
        return;

    if ( i == FL_KEYDOWN )
    {

        if ( bKeyLocked == true )
            return 1;

        int k = Fl::event_key() & FL_KEY_MASK;

        switch ( k )
        {
            case FL_F + 1:
                if ( bMediaReached == true )
                {
                    if ( boxBench->visible_r() > 0 )
                    {
                        boxBench->hide();
                    }
                    else
                    {
                        boxBench->show();
                    }
                }
                prev_keytime = gettimems();
                return 1;

            case FL_F + 5:
                if ( bMediaReached == true )
                {
                    if ( winMe->fullscreen_active() == 0 )
                    {
                        winMe->fullscreen();
                    }
                    else
                    {
                        winMe->fullscreen_off();
                    }
                }
                prev_keytime = gettimems();
                return 1;

            case '1':
                if ( bMediaReached == true )
                {
                    Fl_Image* refimg = boxRenderer->image();

                    if ( refimg != NULL )
                    {
                        int img_w = refimg->w();
                        int img_h = refimg->h();

                        if ( winMe->fullscreen_active() > 0 )
                        {
                            winMe->fullscreen_off();
                        }

                        setRenderSize( img_w, img_h );
                    }
                }


            case FL_Right:
                ffplaymod_playcontrol( 3 );
                prev_keytime = gettimems();
                return 1;

            case FL_Left:
                ffplaymod_playcontrol( 2 );
                prev_keytime = gettimems();
                return 1;

        }
    }

    return 0;
}

void appMain::Connect()
{
    if ( boxRenderer->visible_r() == 0 )
    {
        StartingStream();
    }
}

void appMain::StartingStream()
{
    winMe->color( FL_BLACK );

    boxServerInd->hide();
    inpServerAddress->hide();
    btnConnect->hide();
    boxLic->hide();

    boxRenderer->show();
    boxBench->show();

    const char* ref_uri = inpServerAddress->value();

    if ( strlen( ref_uri ) > 5 )
    {
        if ( ffplaymod_open( ref_uri ) == 0 )
        {
            return;
        }
    }

    boxRenderer->hide();
    boxBench->hide();

    boxServerInd->show();
    inpServerAddress->show();
    btnConnect->show();
    boxLic->show();
}

void appMain::Resync()
{
    if ( audio_proc != NULL )
    {
        audio_proc->Flush();
    }

    ffplaymod_playcontrol( 3 );
}

int appMain::SetupAudio( int c, int r )
{
    if ( audio_proc != NULL )
    {
        if ( audio_proc->Open( c, r ) == true )
            return 0;
    }

    return -1;
}

int appMain::CloseAudio()
{
    if ( audio_proc != NULL )
    {
        audio_proc->Close();
        return 0;
    }

    return -1;
}

int appMain::PauseAudio()
{
    if ( audio_proc != NULL )
    {
        audio_proc->Flush();
        return 0;
    }

    return -1;
}

int appMain::GetAudioBufferSize()
{
    if ( audio_proc != NULL )
    {
        return audio_proc->BufferSize();
    }

    return 0;
}

void appMain::ControlProc(int c)
{
    switch( c )
    {
        case FFPLAYMOD_CTRL_ERR:
            break;

        case FFPLAYMOD_CTRL_LOAD:
            if ( bMediaLoaded == false )
            {
                start( this );
            }
            break;

        case FFPLAYMOD_CTRL_PLAY:
            break;

        case FFPLAYMOD_CTRL_PAUSE:
        case FFPLAYMOD_CTRL_RESUME:
            if ( bKeyLocked == true )
                bKeyLocked = false;
            break;

        case FFPLAYMOD_CTRL_STOP:
            if ( bMediaLoaded == true )
            {
                stop();
            }
            break;

        case FFPLAYMOD_CTRL_UNLOAD:
            break;
    }
}

void appMain::SetMediaReached( bool t )
{
    if ( bMediaLoaded == true )
    {
        if ( t == true )
        {
            bMediaReached = t;

            Fl::add_timeout( APP_SKIP_TIMER_F, appSkipTimer, this );
        }
        else
        {
            // it must be failed to load media !
            ffplaymod_close();

            bMediaLoaded = false;

            Fl::remove_timeout( appSkipTimer, this );

            boxRenderer->hide();
            boxBench->hide();

            boxServerInd->show();
            inpServerAddress->show();
            btnConnect->show();
        }
    }
}

void appMain::createComponents()
{
    winMe = new rtspWindow(400,300,"FLTK.ffmepg.RTSP video stream player");
    if ( winMe != NULL )
    {
#ifdef _WIN32
        extern HINSTANCE fl_display;

        winMe->icon((char *)LoadIcon(fl_display, MAKEINTRESOURCE(IDI_ICON_MAIN)));
#endif

        if ( realborder_x == 0 )
        {
            realborder_x = 5;
        }

        if ( realborder_y == 0 )
        {
            realborder_y = 10;
        }

        winMe->border( 1 );
        winMe->eventhandler( this );
        winMe->color( FL_BLACK );
        winMe->begin();

        boxRenderer = new Fl_MRender(0,0,400,300);
        if ( boxRenderer != NULL )
        {
            boxRenderer->color( FL_BLACK );
            boxRenderer->eventhandler( this );
            boxRenderer->hide();
        }

        boxInfoBg = new Fl_TransBox(0,0,400,300);
        if ( boxInfoBg != NULL )
        {
            boxInfoBg->hide();
        }

        boxLic = new Fl_Box(0,0,400,300);
        if ( boxLic != NULL )
        {
            static char outmsg[512] = {0};
            sprintf(outmsg, "FLTK RTSP(ffmpeg) player\n(C)2015 Rageworx free softwares, rageworx/rage.kim@@gmail.com\n");
            strcat(outmsg, "\n*Opensource*\n");
            strcat(outmsg, "FLTK: This program is a part of FLTK project.\nffmpeg: ffmpeg is an open source project.");

            boxLic->box( FL_NO_BOX );
            boxLic->align( FL_ALIGN_INSIDE | FL_ALIGN_LEFT | FL_ALIGN_BOTTOM );
            boxLic->labelcolor( FL_WHITE );
            boxLic->labelsize(12);
            boxLic->label( outmsg );
        }

        boxInfo = new Fl_Box(0,0,400,300,"this is not be appeared");
        if ( boxInfo != NULL )
        {
            boxInfo->box( FL_NO_BOX );
            boxInfo->labelcolor( FL_WHITE );
            boxInfo->labelsize( 30 );
            boxInfo->hide();
        }

        boxBench = new Fl_Box(0,0,300,50,"reaching to server ...");
        if ( boxBench != NULL )
        {
            boxBench->box( FL_NO_BOX );
            boxBench->align( FL_ALIGN_INSIDE | FL_ALIGN_LEFT | FL_ALIGN_TOP );
            boxBench->labelfont( FL_COURIER );
            boxBench->labelcolor( FL_WHITE );
            boxBench->labelsize( 16 );
            boxBench->hide();
        }

        boxServerInd = new Fl_Box(10,10,380,20,"RTSP server:");
        if ( boxServerInd !=NULL )
        {
            boxServerInd->box( FL_NO_BOX );
            boxServerInd->align( FL_ALIGN_INSIDE | FL_ALIGN_LEFT );
            boxServerInd->labelfont( FL_HELVETICA_BOLD_ITALIC );
            boxServerInd->labelcolor( FL_WHITE );
        }

        inpServerAddress = new Fl_Input(10,35,380,25);
        if ( inpServerAddress != NULL )
        {
            inpServerAddress->box( FL_FLAT_BOX );
            inpServerAddress->color( FL_DARK3 );
            inpServerAddress->textcolor( FL_GREEN );
            inpServerAddress->textfont( FL_COURIER );
            inpServerAddress->value( "rtsp://192.168.0.30/live1.sdp" );
            inpServerAddress->cursor_color( FL_RED );
            inpServerAddress->callback( inpCB, this );

            if ( userURL != NULL )
            {
                inpServerAddress->value( userURL );
            }
        }

        btnConnect = new Fl_Button(10,65,100,30,"&Connect");
        if ( btnConnect != NULL )
        {
            btnConnect->box( FL_FLAT_BOX );
            btnConnect->color( 0x3F3FFF00 );
            btnConnect->labelcolor( FL_WHITE );
            btnConnect->callback( btnCB, this );
        }

        winMe->resizable( boxRenderer );
        winMe->size_range( winMe->w() - 1, winMe->h() - 1,
                           winMe->w(), winMe->h(),
                           0, 0,
                           0 );

        winMe->end();
        winMe->callback( winCB, this );
        winMe->show();

#ifdef _WIN32
        HWND winH = fl_xid( winMe );
        RECT winRect;

        GetWindowRect( winH, &winRect );

        realborder_x = ( winRect.right - winRect.left - winMe->w() ) / 2;
        realborder_y = winRect.bottom - winRect.top - winMe->h() - realborder_x;

#else
        realborder_x = ( winMe->decorated_w() - winMe->w() ) / 2;
        realborder_y = winMe->decorated_h() - winMe->h() - realborder_x;
#endif

    }
}

void  appMain::prepareThread( void* p )
{
    bMediaLoaded = true;
    bMediaReached = false;

    Fl::add_timeout( APP_LOAD_CHECK_TIMER_F, appLoadCheckTimer, this );
}

void* appMain::doThread( void* p )
{
    // Wait for warming up !
    sleepms( 500 );

    int force_skip_frames = 0;

    while( bMediaLoaded == true )
    {
        if ( bFlushingReady == false )
        {
            sleepms( wait_ms );
            continue;
        }

        VideoState* vs = ffplaymod_getVideoState();

        if ( locked() == false )
        {
            lock();

            if ( vs != NULL )
            {
                unsigned char* rgbapixels = NULL;

                if ( ffplaymod_getvideobuffer( &rgbapixels ) > 0 )
                {
                    if ( rgbapixels == NULL )
                    {
                        unlock();
                        continue;
                    }

                    if ( GetMediaReadched() == false )
                    {
                        SetMediaReached( true );
                        Resync();
                    }

                    if ( force_skip_frames < 5 )
                    {
                        delete rgbapixels;
                        force_skip_frames ++;
                        unlock();
                        continue;
                    }

                    Fl_RGB_Image* newimg = new Fl_RGB_Image( rgbapixels, vs->width, vs->height, 4 );
                    if ( ( newimg != NULL ) && ( GetImageReady() == true ) )
                    {
                        if (  putImage( newimg ) == false )
                        {
                            delete newimg;
                        }

                    }
                }
                else
                {
                    sleepms( 1 );
                    frameskip++;
                }

            }

            unlock();
        }
        else
        {
            sleepms( wait_ms );
        }
    }

}

void  appMain::cleanupThread( void* p )
{

}

void appMain::audioBuffreRequired()
{
    if ( bMediaLoaded == false )
        return;

    char* audio_buff = NULL;
    unsigned dur_ms = 0;

    unsigned int audio_sz = ffplaymod_getaudiobuffer( &audio_buff, &dur_ms );

    if ( audio_sz > 0 )
    {
        audio_proc->Write( audio_buff, audio_sz, dur_ms );
    }

}

static void appMain::recv_controlsig(void* p, int a)
{
    if ( p == NULL )
        return;

    appMain* am = (appMain*)p;

    am->ControlProc(a);
}
