#include "audio.h"
#include <windows.h>

#define BUFFER_SIZE 0x10000
#define NUM_BUFFERS 8  /* total 512k roughly 2.5 sec of CD quality sound */

static void wait_for_buffer(WAVEHDR* hdr, HANDLE hEvent);
static void drain_audio(struct audio_output_struct *ao);

void ereturn( int eno, const char* estr )
{
#ifdef DEBUG
    printf("Error(%d):%s\n", eno, estr);
#endif
}

/* Buffer ring queue state */
struct queue_state
{
    WAVEHDR buffer_headers[NUM_BUFFERS];
    /* The next buffer to be filled and put in playback */
    int next_buffer;
    /* Buffer playback completion event */
    HANDLE play_done_event;
    HWAVEOUT waveout;
};

static int open_audio( audio_output_t *ao )
{
    struct queue_state* state;
    int i;
    MMRESULT res;
    WAVEFORMATEX out_fmt;
    UINT dev_id;

    if(!ao) return -1;
    if(ao->rate == -1) return 0;

    /* Allocate queue state struct for this device */
    state = calloc(1, sizeof(struct queue_state));
    if(!state) return -1;

    ao->userptr = state;

    state->play_done_event = CreateEvent(0,FALSE,FALSE,0);
    if(state->play_done_event == INVALID_HANDLE_VALUE) return -1;

    /* FIXME: real device enumeration by capabilities? */
    dev_id = WAVE_MAPPER;    /* probably does the same thing */
    ao->device = "WaveMapper";
    /* FIXME: support for smth besides MPG123_ENC_SIGNED_16? */
    out_fmt.wFormatTag = WAVE_FORMAT_PCM;
    out_fmt.wBitsPerSample = 16;
    out_fmt.nChannels = ao->channels;
    out_fmt.nSamplesPerSec = ao->rate;
    out_fmt.nBlockAlign = out_fmt.nChannels*out_fmt.wBitsPerSample/8;
    out_fmt.nAvgBytesPerSec = out_fmt.nBlockAlign*out_fmt.nSamplesPerSec;
    out_fmt.cbSize = 0;

    res = waveOutOpen(&state->waveout, dev_id, &out_fmt,
                      (DWORD_PTR)state->play_done_event, 0, CALLBACK_EVENT);

    switch(res)
    {
        case MMSYSERR_NOERROR:
            break;
        case MMSYSERR_ALLOCATED:
            ereturn(-1, "Audio output device is already allocated.");
        case MMSYSERR_NODRIVER:
            ereturn(-1, "No device driver is present.");
        case MMSYSERR_NOMEM:
            ereturn(-1, "Unable to allocate or lock memory.");
        case WAVERR_BADFORMAT:
            ereturn(-1, "Unsupported waveform-audio format.");
        default:
            ereturn(-1, "Unable to open wave output device.");
    }

    /* Reset event from the "device open" message */
    ResetEvent(state->play_done_event);
    /* Allocate playback buffers */
    for(i = 0; i < NUM_BUFFERS; i++)
    if(!(state->buffer_headers[i].lpData = (LPSTR)malloc(BUFFER_SIZE)))
    {
        ereturn(-1, "Out of memory for playback buffers.");
    }
    else
    {
        /* Tell waveOutPrepareHeader the maximum value of dwBufferLength
        we will ever send */
        state->buffer_headers[i].dwBufferLength = BUFFER_SIZE;
        state->buffer_headers[i].dwFlags = 0;
        res = waveOutPrepareHeader(state->waveout, &state->buffer_headers[i], sizeof(WAVEHDR));
        if(res != MMSYSERR_NOERROR) ereturn(-1, "Can't write to audio output device (prepare).");

        /* set the current size of the buffer to 0 */
        state->buffer_headers[i].dwBufferLength = 0;

        /* set flags to unprepared - must reset this to WHDR_PREPARED before calling write */
        state->buffer_headers[i].dwFlags = 0;
    }

    return 0;
}

static void wait_for_buffer(WAVEHDR* hdr, HANDLE hEvent)
{
    /* At this point there are several possible states:
    1) Empty or partial buffer (unqueued) - dwFlags == 0
    2) Buffer queued or being played - dwFlags == WHDR_PREPARED | WHDR_INQUEUE
    3) Buffer unqueued and finished being played - dwFlags == WHDR_PREPARED | WHDR_DONE
    4) Buffer removed from queue, but not yet marked as done - dwFlags == WHDR_PREPARED
    */

    /* Check buffer header and wait if it's being played. */
    if (hdr->dwFlags & WHDR_PREPARED)
    {
        while(!(hdr->dwFlags & WHDR_DONE))
        {
            /*debug1("waiting for buffer %i...", state->next_buffer);*/
            /* Waits for *a* buffer to finish.  May not be the one we
            want, so check again */
            WaitForSingleObject(hEvent, INFINITE);
        }
        hdr->dwFlags = 0;
        hdr->dwBufferLength = 0;
    }
}

static int get_audio_formats(struct audio_output_struct *ao)
{
    /* FIXME: support for smth besides MPG123_ENC_SIGNED_16? */
    return AUDIO_TYPE_SIGNED_16;
}

/* Stores audio data to the fixed size buffers and pushes them into the playback queue.
   I have one grief with that: The last piece of a track may not reach the output,
   only full buffers sent... But we don't get smooth audio otherwise. */
static int write_audio(struct audio_output_struct *ao, unsigned char *buf, int len)
{
    struct queue_state* state;
    MMRESULT res;
    WAVEHDR* hdr;

    int rest_len; /* Input data bytes left for next recursion. */
    int bufill;   /* Bytes we stuff into buffer now. */

    if(!ao || !ao->userptr) return -1;
    if(!buf || len <= 0) return 0;

    state = (struct queue_state*)ao->userptr;
    hdr = &state->buffer_headers[state->next_buffer];

    wait_for_buffer(hdr, state->play_done_event);

    /* Now see how much we want to stuff in and then stuff it in. */
    bufill = BUFFER_SIZE - hdr->dwBufferLength;
    if(len < bufill) bufill = len;

    rest_len = len - bufill;
    memcpy(hdr->lpData + hdr->dwBufferLength, buf, bufill);
    hdr->dwBufferLength += bufill;
    if(hdr->dwBufferLength == BUFFER_SIZE)
    { /* Send the buffer out when it's full. */
        hdr->dwFlags |= WHDR_PREPARED;

        res = waveOutWrite(state->waveout, hdr, sizeof(WAVEHDR));
        if(res != MMSYSERR_NOERROR) ereturn(-1, "Can't write to audio output device.");

        /* Cycle to the next buffer in the ring queue */
        state->next_buffer = (state->next_buffer + 1) % NUM_BUFFERS;
    }
    /* I'd like to propagate error codes or something... but there are no catchable surprises left.
       Anyhow: Here is the recursion that makes ravenexp happy;-) */
    if(rest_len && write_audio(ao, buf + bufill, rest_len) < 0) /* Write the rest. */
    return -1;
    else
    return len;
}

     /* Flush means abort any pending playback */
static void flush_audio(struct audio_output_struct *ao)
{
    struct queue_state* state;
    WAVEHDR* hdr;

    if(!ao || !ao->userptr) return;
    state = (struct queue_state*)ao->userptr;

    /* Cancel any buffers in queue.  Ignore errors since we are void and
    can't return them anyway */
    waveOutReset(state->waveout);

    /* Discard any partial buffer */
    hdr = &state->buffer_headers[state->next_buffer];

    /* If WHDR_PREPARED is not set, this is (potentially) a partial buffer */
    if (!(hdr->dwFlags & WHDR_PREPARED))
    hdr->dwBufferLength = 0;

    /* Finish processing the buffers */
    drain_audio(ao);
}

/* output final buffer (if any) */
static void write_final_buffer(struct queue_state *state)
{
    WAVEHDR* hdr;
    hdr = &state->buffer_headers[state->next_buffer];
    if((!(hdr->dwFlags & WHDR_PREPARED)) && (hdr->dwBufferLength != 0))
    {
        hdr->dwFlags |= WHDR_PREPARED;
        /* ignore any errors */
        waveOutWrite(state->waveout, hdr, sizeof(WAVEHDR));

        /* Cycle to the next buffer in the ring queue */
        state->next_buffer = (state->next_buffer + 1) % NUM_BUFFERS;
    }
}

/* Note: I tried to fix this stuff without testing.
   There were some obvious errors in the code.
   Someone run this on a win32 machine! -- ThOr */
static void drain_audio(struct audio_output_struct *ao)
{
    int i, z;
    struct queue_state* state;

    if(!ao || !ao->userptr) return;
    state = (struct queue_state*)ao->userptr;

    /* output final buffer (if any) */
    write_final_buffer(state);

    /* I _think_ I understood how this should work. -- ThOr */
    z = state->next_buffer;
    for(i = 0; i < NUM_BUFFERS; i++)
    {
        wait_for_buffer(&state->buffer_headers[z], state->play_done_event);
        z = (z + 1) % NUM_BUFFERS;
    }
}

static int close_audio(struct audio_output_struct *ao)
{
    int i;
    struct queue_state* state;

    if(!ao || !ao->userptr) return -1;
    state = (struct queue_state*)ao->userptr;

    /* wait for all active buffers to complete */
    drain_audio(ao);
    CloseHandle(state->play_done_event);

    for(i = 0; i < NUM_BUFFERS; i++)
    {
        state->buffer_headers[i].dwFlags |= WHDR_PREPARED;
        waveOutUnprepareHeader(state->waveout, &state->buffer_headers[i], sizeof(WAVEHDR));
        free(state->buffer_headers[i].lpData);
    }

    waveOutClose(state->waveout);
    free(ao->userptr);
    ao->userptr = 0;
    return 0;
}

static int getbuffersize()
{
    return BUFFER_SIZE;
}

int init_audio(audio_output_t* ao)
{
    if(!ao) return -1;

    /* Set callbacks */
    ao->open = open_audio;
    ao->flush = flush_audio;
    ao->write = write_audio;
    ao->get_formats = get_audio_formats;
    ao->close = close_audio;
    ao->getbuffersize = getbuffersize;

    /* Success */
    return 0;
}
