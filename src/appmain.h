#ifndef __APPMAIN_H__
#define __APPMAIN_H__

#include <FL/Fl.H>
#include <FL/Fl_Window.H>
#include <FL/Fl_Double_Window.H>
#include <FL/Fl_Group.H>
#include <FL/Fl_Box.H>
#include <FL/Fl_Input.H>
#include <FL/Fl_Button.H>
#include <FL/Fl_Timer.H>
#include <FL/fl_ask.H>

#include "Fl_MRender.h"
#include "Fl_TransBox.h"

#include "ffplaymod.h"
#include "appaudio.h"
#include "cpthread.h"

typedef void (*event_callback)(void *p, int e);

class rtspWindow
 : public Fl_Double_Window
{
    public:
        rtspWindow(int W, int H, const char *l = 0);
        virtual ~rtspWindow();

    // Inherits ...
    public:
        int handle(int e);
        void resize(int x, int y, int w, int h);

    public:
        void  eventhandler( Fl_MRender_Event* p )   { _eventh = p; }
        Fl_MRender_Event* eventhandler()            { return _eventh; }

    protected:
        Fl_MRender_Event*   _eventh;
};

class appMain
 : public Fl_MRender_Event,
   public cpthread,
   public AudioProcReq
{
    public:
        appMain( int argc, char** argv );
        virtual ~appMain();

    public:
        int SetParams(int argc, const char** argv );
        int Run();

    public:
        void setRenderSize(int w, int h);
        bool putImage( Fl_RGB_Image* i );
        void setFullScreen();
        void setevent( event_callback e, void* p );

    public:
        Fl_Double_Window* getwindow()       { return winMe; }

    public:
        int event(void *p, int i); /// Inherit to Fl_MRender_Event

    public:
        void Connect();
        void StartingStream();
        void Resync();
        void SetMediaReached( bool t );
        bool GetMediaReadched()             { return bMediaReached; }
        bool GetMediaLoaded()               { return bMediaLoaded; }
        bool GetImageReady()                { return !blMutexer; }
        int  SetupAudio( int c, int r );
        int  PauseAudio();
        int  CloseAudio();
        int  GetAudioBufferSize();
        void ControlProc(int c);

    public:
        void  prepareThread( void* p );
        void* doThread( void* p );
        void  cleanupThread( void* p );

    public:
        void audioBuffreRequired();

    protected:
        void createComponents();

    protected:
        int         _argc;
        char**      _argv;

    protected:
        rtspWindow*         winMe;
        Fl_Box*             boxLic;
        Fl_Box*             boxServerInd;
        Fl_Input*           inpServerAddress;
        Fl_Button*          btnConnect;
        Fl_MRender*         boxRenderer;
        Fl_Box*             boxBench;
        Fl_TransBox*        boxInfoBg;
        Fl_Box*             boxInfo;
        bool                blMutexer;
        bool                bInitFFplay;
        bool                bMediaLoaded;
        bool                bMediaReached;
        bool                bPaused;
        bool                bKeyLocked;
        bool                bFlushingReady;
        void*               event_param;
        unsigned            wait_ms;
        unsigned long long  framecount;
        unsigned long long  frameskip;
        unsigned long long  info_frame;
        unsigned long       prev_keytime;
        int                 realborder_x;
        int                 realborder_y;
        VideoState*         vid_state;
        AudioProc*          audio_proc;
        char*               userURL;

    public:
        static void recv_controlsig(void* p, int a);
};

#endif /// of __APPMAIN_H__
