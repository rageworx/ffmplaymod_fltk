#include "ffplaymod.h"
#include "appmain.h"

extern appMain* aMain;

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// Embody of ffplaymod audio open.
int ffplaymod_port_audio_buffersize()
{
    if ( aMain != NULL )
    {
        return aMain->GetAudioBufferSize();
    }

    return 1024;
}

int ffplaymod_port_audio_open( int channels, int rate )
{
    if ( aMain != NULL )
    {
        return aMain->SetupAudio( channels, rate );
    }

    return -1;
}

int ffplaymod_port_audio_close()
{
    if ( aMain != NULL )
    {
        return aMain->CloseAudio();
    }

    return -1;
}

void ffplaymod_port_content_size( int width, int height )
{
    if ( aMain != NULL )
    {
        aMain->setRenderSize( width, height );
    }
}

void ffplaymod_port_content_title( const char* title )
{
    if ( aMain != NULL )
    {
        Fl_Double_Window* aWin = aMain->getwindow();
        if ( aWin != NULL )
        {
            aWin->label( title );
        }
    }
}

