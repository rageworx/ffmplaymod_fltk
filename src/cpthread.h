#ifndef __CPTHREAD_H__
#define __CPTHREAD_H__

#include <pthread.h>

class cpthread
{
    public:
        cpthread( bool autostart=false, bool mutexed=true );
        virtual ~cpthread();

    public:
        bool            started()       { return _started; }
        pthread_t       handle()        { return _pt; }
        int             id()            { return _id; }
        pthread_mutex_t mutexhandle()   { return _mpt; }
        bool            locked()        { return _locked; }

    public: /// thread controls
        bool        start( void* p );
        bool        stop();
        bool        lock();
        bool        unlock();
        bool        condsig();
        bool        waitforsig( unsigned timedout = 0 );
        void        sleepms( unsigned wms );
        unsigned long long gettimems();

    public: /// do not call these functions in directly.
        static
        void*       threadcall( void* p );
        void*       outtercall();

    public: /// Inherit these functions for threading
        virtual void  prepareThread( void* p ) = 0;
        virtual void* doThread( void* p ) = 0;
        virtual void  cleanupThread( void* p ) = 0;

    protected:
        bool                _started;
        bool                _mutexed;
        bool                _needlock;
        bool                _locked;
        int                 _id;
        pthread_t           _pt;
        pthread_mutex_t     _mpt;
        pthread_cond_t      _cpt;
        int                 _waitms;
        void*               _tparam;

    public:
        static bool waitforthread( cpthread* cpt );
};

#endif /// of __CPTHREAD_H__
