#include <sys/time.h>
#include "appaudio.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

AudioProc::AudioProc()
 : _channels(-1),
   _freq(-1),
   current_stream(-1),
   next_stream(-1),
   stopsig(0),
   isthreading(false),
   audiofilled(false),
   isworking(false)
{
    memset( &audio_out, 0, sizeof( audio_output_t ) );
    init_audio( &audio_out );
}

AudioProc::~AudioProc()
{
    Close();

    audio_out.deinit( &audio_out );
}

bool AudioProc::Open( int channels, int freq )
{
    if ( audio_out.is_open == 0 )
    {
        _channels = channels;
        _freq     = freq;

        audio_out.channels = channels;
        audio_out.rate     = freq;

        if( audio_out.open( &audio_out ) == 0 )
        {
            start( NULL );
            return true;
        }
    }

    return false;
}

bool AudioProc::Write( unsigned char* buff, int len, unsigned dur_ms )
{
    if ( ( isworking == true ) || ( isthreading == false ) )
        return false;

    if ( ( _channels < 0 ) || ( _freq < 0 ) )
        return false;

    if ( ( buff != NULL ) && ( len > 0 ) )
    {
        if ( current_stream + 1 < MAX_AUDIO_STREAMS )
        {
            int inp_s = current_stream + 1;
            if ( inp_s >= MAX_AUDIO_STREAMS )
            {
                inp_s = 0;
                if ( inp_s == current_stream )
                    return false;       /// no spare buffer left !!
            }

            if ( streams[inp_s] != NULL )
            {
                delete[] streams[inp_s];
                streams[inp_s] = NULL;
            }

            stream_len[inp_s] = 0;
            stream_dur[inp_s] = -1;

            streams[inp_s] = new unsigned char[ len ];
            if ( streams[inp_s] != NULL )
            {
                lock();

                memcpy( streams[inp_s], buff, len );
                delete[] buff;

                stream_len[inp_s] = len;
                stream_dur[inp_s] = calc_dur( len );

                next_stream = inp_s;

                unlock();

                return true;
            }
        }
    }

    return false;
}

void AudioProc::Flush()
{
    if ( isworking == true )
        return;

    if ( audio_out.is_open > 0 )
    {
        isworking = true;

        audio_out.flush( &audio_out );
        sleepms(10);
        resetbuffers();

        isworking = false;

    }
}

unsigned long AudioProc::calc_dur( int blen )
{
    if ( ( _channels <= 0 ) || ( _freq <= 0 ) )
        return 0;

    return blen * ( ( _freq / 1024 ) / _channels );
}

void AudioProc::Close()
{
    while( isworking == true )
    {
        sleepms(0);
    }

    resetbuffers();
    audio_out.flush( &audio_out );
    audio_out.close( &audio_out );

#ifdef DEBUG
    printf("Audo proc complete!\n");
#endif
}

int  AudioProc::BufferSize()
{
    return audio_out.getbuffersize();
}

void  AudioProc::prepareThread( void* p )
{
    isthreading = true;
}

void* AudioProc::doThread( void* p )
{
    while( stopsig == 0 )
    {
        unsigned long curtick = gettimems();

        if ( req_cb != NULL )
        {
            req_cb->audioBuffreRequired();
        }

        while( locked() == true )
        {
            sleepms(1);
        }

        lock();

        current_stream = next_stream;

        if ( streams[ current_stream ] != NULL )
        {
            if ( stream_len[ current_stream ] > 0 )
            {
                audio_out.write( &audio_out,
                                streams[ current_stream ],
                                stream_len[ current_stream ] );
            }

            //delete[] streams[ current_stream ];
            //streams[ current_stream ] = NULL;
            //stream_len[ current_stream ] = 0;
            //stream_dur[ current_stream ] = -1;

            if ( current_stream >= 0 )
                current_stream--;
        }

        unlock();
        sleepms(0);
    }
}

void  AudioProc::cleanupThread( void* p )
{
    isthreading = false;
}

void AudioProc::resetbuffers()
{
    if ( isthreading == true )
    {
        stopsig = 1;
        pthread_t pid = handle();
        stop();
        pthread_join( pid, NULL );
    }

    for( int cnt=0; cnt<MAX_AUDIO_STREAMS; cnt++ )
    {
        if( streams[cnt] != NULL )
        {
            delete[] streams[cnt];
            streams[cnt] = NULL;
        }
        stream_len[cnt] = 0;
        stream_dur[cnt] = -1;
    }
}
