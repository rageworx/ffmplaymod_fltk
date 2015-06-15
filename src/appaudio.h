#ifndef __APPAUDIO_H__
#define __APPAUDIO_H__

#include "audio.h"
#include "cpthread.h"

#define MAX_AUDIO_STREAMS       4

class AudioProcReq
{
    public:
        virtual void audioBuffreRequired() = 0;
};

class AudioProc : public cpthread
{
    public:
        AudioProc();
        ~AudioProc();

    public:
        bool Open( int channels, int freq );
        bool Write( unsigned char* buff, int len, unsigned dur_ms );
        void Flush();
        void Close();
        int  BufferSize();

    public:
        void reqcb( AudioProcReq *cb )      { req_cb = cb; }
        AudioProcReq* reqcb()               { return req_cb; }

    public:
        void  prepareThread( void* p );
        void* doThread( void* p );
        void  cleanupThread( void* p );

    private:
        void resetbuffers();
        unsigned long calc_dur( int blen );

    private:
        int             _channels;
        int             _freq;
        int             stopsig;
        bool            isthreading;
        bool            audiofilled;
        bool            isworking;
        unsigned char*  streams[MAX_AUDIO_STREAMS];
        unsigned long   stream_len[MAX_AUDIO_STREAMS];
        unsigned long   stream_dur[MAX_AUDIO_STREAMS];
        int             current_stream;
        int             next_stream;
        audio_output_t  audio_out;
        AudioProcReq*   req_cb;
};

#endif /// of __APPAUDIO_H__
