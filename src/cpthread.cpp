#ifdef _WIN32
    #include <windows.h>
#endif
#include <signal.h>
#include <cstring>
#include <sys/time.h>
#include <time.h>
#include "cpthread.h"

cpthread::cpthread( bool autostart, bool mutexed )
 : _started( false ),
   _mutexed( mutexed ),
   _needlock( false ),
   _locked( false )
{
    memset( &_pt, 0, sizeof(pthread_t) );
    memset( &_mpt, 0, sizeof(pthread_mutex_t) );
    memset( &_cpt, 0, sizeof(pthread_cond_t) );

    if ( _mutexed == true )
    {
        _mpt = PTHREAD_MUTEX_INITIALIZER;
        _cpt = PTHREAD_COND_INITIALIZER;
    }

    if ( autostart == true )
    {
        start( NULL );
    }
}

cpthread::~cpthread()
{
    if ( _started == true )
    {
        stop();
    }

    if ( _mutexed == true )
    {
        pthread_cond_destroy( &_cpt );
        pthread_mutex_destroy( &_mpt );
    }
}

bool cpthread::start( void* p )
{
    if ( _started == true )
        return false;

    _tparam = p;
    _id = pthread_create( &_pt, NULL, cpthread::threadcall, (void*)this );
    if ( _id >= 0 )
    {
        pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS,NULL);
        _started = true;
    }
}

bool cpthread::stop()
{
    if ( _started == true )
    {
        if ( _mutexed == true )
        {
            if ( _locked == true )
            {
                pthread_mutex_unlock( &_mpt );
                _locked = false;
            }
        }

        pthread_cancel( _pt );
        pthread_kill( _pt, 0  );
        return true;
    }

    return false;
}

bool cpthread::lock()
{
    if ( ( _mutexed == true ) && ( _locked == false ) )
    {
        pthread_mutex_lock( &_mpt );
        _locked = true;
        return true;
    }

    return false;
}

bool cpthread::unlock()
{
    if ( ( _mutexed == true ) && ( _locked == true ) )
    {
        pthread_mutex_unlock( &_mpt );
        _locked = false;
        return true;
    }

    return false;
}

bool cpthread::condsig()
{
    if ( ( _mutexed == true ) && ( _locked == true ) )
    {
        pthread_cond_signal( &_cpt );
        return true;
    }

    return false;
}

bool cpthread::waitforsig( unsigned timedout )
{
    if ( ( _mutexed == true ) && ( _locked == true ) )
    {
        if ( timedout == 0 )
        {
            if ( pthread_cond_wait( &_cpt, &_mpt ) == 0 )
                return true;
        }
        else
        {
            struct timespec ts = {0};

            ts.tv_sec  = timedout / 1000000;
            ts.tv_nsec = timedout % 1000000;

            if ( pthread_cond_timedwait( &_cpt, &_mpt, &ts ) == 0 )
                return true;
        }
    }

    return false;
}

void cpthread::sleepms( unsigned wms )
{
#ifdef _WIN32
    Sleep( wms );
#else
    if ( wms == 0 )
    {
        sleep(0);
    }
    else
    {
        struct timeval sttv = {0};

        sttv.tv_sec  = wms / 1000000;
        sttv.tv_usec = wms % 1000000;

        select(0, 0, 0, 0, &sttv);
    }
#endif
}

unsigned long long cpthread::gettimems()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);

    return (unsigned long)(tv.tv_sec * 1000 + tv.tv_usec / 1000);
}

void* cpthread::outtercall()
{
    prepareThread( _tparam );
    void* retp = doThread( _tparam );
    cleanupThread( _tparam );
    pthread_exit(retp);
    return retp;
}

void* cpthread::threadcall( void* p )
{
    if ( p!= NULL )
    {
        cpthread* cpt = (cpthread*)p;
        void* retp = cpt->outtercall();

        return retp;
    }

    return NULL;
}

bool cpthread::waitforthread( cpthread* cpt )
{
    if ( cpt != NULL )
    {
        pthread_t pid = cpt->handle();
        pthread_join( pid, 0 );

        return true;
    }

    return false;
}
