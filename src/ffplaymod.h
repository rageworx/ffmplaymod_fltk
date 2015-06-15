#ifndef __FFPLAYMOD_H__
#define __FFPLAYMOD_H__

#include "ffconfig.h"

#include <inttypes.h>
#include <math.h>
#include <limits.h>
#include <signal.h>
#include <stdint.h>

extern "C" {
#include "libavutil/avstring.h"
#include "libavutil/colorspace.h"
#include "libavutil/mathematics.h"
#include "libavutil/pixdesc.h"
#include "libavutil/imgutils.h"
#include "libavutil/dict.h"
#include "libavutil/parseutils.h"
#include "libavutil/samplefmt.h"
#include "libavutil/avassert.h"
#include "libavutil/time.h"
#include "libavformat/avformat.h"
#include "libavdevice/avdevice.h"
#include "libswscale/swscale.h"
#include "libavutil/opt.h"
#include "libavutil/log.h"
#include "libavcodec/avfft.h"
#include "libswresample/swresample.h"

#include "libavfilter/avcodec.h"
#include "libavfilter/avfilter.h"
#include "libavfilter/buffersink.h"
#include "libavfilter/buffersrc.h"

#include "cmdutils.h"
}
#include <pthread.h>

////////////////////////////////////////////////////////////////////////////////

/* no AV sync correction is done if below the minimum AV sync threshold */
#define AV_SYNC_THRESHOLD_MIN 0.01
/* AV sync correction is done if above the maximum AV sync threshold */
#define AV_SYNC_THRESHOLD_MAX 0.1
/* If a frame duration is longer than this, it will not be duplicated to compensate AV sync */
#define AV_SYNC_FRAMEDUP_THRESHOLD 0.1
/* no AV correction is done if too big error */
#define AV_NOSYNC_THRESHOLD 10.0

/* maximum audio speed change to get correct sync */
#define SAMPLE_CORRECTION_PERCENT_MAX 10

/* external clock speed adjustment constants for realtime sources based on buffer fullness */
#define EXTERNAL_CLOCK_SPEED_MIN  0.900
#define EXTERNAL_CLOCK_SPEED_MAX  1.010
#define EXTERNAL_CLOCK_SPEED_STEP 0.001

/* we use about AUDIO_DIFF_AVG_NB A-V differences to make the average */
#define AUDIO_DIFF_AVG_NB   20

/* polls for possible required screen refresh at least this often, should be less than 1/fps */
#define REFRESH_RATE 0.01

/* NOTE: the size must be big enough to compensate the hardware audio buffersize size */
/* TODO: We assume that a decoded and resampled frame fits into this buffer */
#define SAMPLE_ARRAY_SIZE (8 * 65536)

#define AUDIO_BUFFER_SIZE 8 * 1024

#ifndef INT64_MAX
    #define INT64_MAX   0x7fffffffffffffffLL
#endif

#ifndef INT64_MIN
    #define INT64_MIN   (-INT64_MAX - 1LL)
#endif

////////////////////////////////////////////////////////////////////////////////

typedef struct VideoPicture
{
    double pts;             /// presentation timestamp for this picture
    double duration;        /// estimated duration based on frame rate
    int64_t pos;            /// byte position in file
    unsigned char* bmp;
    int bmpsize;
    int width, height; /* source height & width */
    int allocated;
    int serial;
    int ready;
    AVRational sar;
} VideoPicture;

typedef struct SubPicture
{
    double pts; /* presentation time stamp for this picture */
    AVSubtitle sub;
    int serial;
} SubPicture;

typedef struct AudioParams
{
    int freq;
    int channels;
    int64_t channel_layout;
    enum AVSampleFormat fmt;
    int frame_size;
    int bytes_per_sec;
} AudioParams;

typedef struct Clock
{
    double pts;           /* clock base */
    double pts_drift;     /* clock base minus time at which we updated the clock */
    double last_updated;
    double speed;
    int serial;           /* clock is based on a packet with this serial */
    int paused;
    int *queue_serial;    /* pointer to the current packet queue serial, used for obsolete clock detection */
} Clock;

enum
{
    AV_SYNC_AUDIO_MASTER, /* default choice */
    AV_SYNC_VIDEO_MASTER,
    AV_SYNC_EXTERNAL_CLOCK, /* synchronize to an external clock */
};

typedef struct MyAVPacketList
{
    AVPacket pkt;
    struct MyAVPacketList *next;
    int serial;
} MyAVPacketList;

typedef struct PacketQueue
{
    MyAVPacketList *first_pkt, *last_pkt;
    int nb_packets;
    int size;
    int abort_request;
    int serial;
    pthread_mutex_t mutex;
    pthread_cond_t cond;
} PacketQueue;

#define VIDEO_PICTURE_QUEUE_SIZE 3
#define SUBPICTURE_QUEUE_SIZE 4

typedef struct VideoState
{
    pthread_t read_tid;
    pthread_t video_tid;
    pthread_t subtitle_tid;

    AVInputFormat *iformat;
    int no_background;
    int abort_request;
    int force_refresh;
    int paused;
    int last_paused;
    int queue_attachments_req;
    int seek_req;
    int seek_flags;
    int64_t seek_pos;
    int64_t seek_rel;
    int read_pause_return;
    AVFormatContext *ic;
    int realtime;
    int audio_finished;
    int video_finished;

    Clock audclk;
    Clock vidclk;
    Clock extclk;

    int audio_stream;
    int av_sync_type;

    // ---------------------------------------
    double audio_clock;
    int audio_clock_serial;
    double audio_diff_cum; /* used for AV difference average computation */
    double audio_diff_avg_coef;
    double audio_diff_threshold;
    int audio_diff_avg_count;
    AVStream *audio_st;
    PacketQueue audioq;
    int audio_hw_buf_size;
    uint8_t silence_buf[AUDIO_BUFFER_SIZE];
    uint8_t *audio_buf;
    uint8_t *audio_buf1;
    unsigned int audio_buf_size; /* in bytes */
    unsigned int audio_buf1_size;
    int audio_buf_index; /* in bytes */
    int audio_write_buf_size;
    int audio_buf_frames_pending;
    AVPacket audio_pkt_temp;
    AVPacket audio_pkt;
    int audio_pkt_temp_serial;
    int audio_last_serial;
    struct AudioParams audio_src;
    struct AudioParams audio_filter_src;
    struct AudioParams audio_tgt;
    struct SwrContext *swr_ctx;
    int frame_drops_early;
    int frame_drops_late;
    AVFrame* frame;
    AVFrame* frameRGB32;
    int64_t audio_frame_next_pts;

    // ---------------------------------------
    int16_t sample_array[SAMPLE_ARRAY_SIZE];
    int sample_array_index;
    int last_i_start;
    RDFTContext *rdft;
    int rdft_bits;
    FFTSample *rdft_data;
    int xpos;
    double last_vis_time;

    // ---------------------------------------
    int subtitle_stream;
    AVStream *subtitle_st;
    PacketQueue subtitleq;
    SubPicture subpq[SUBPICTURE_QUEUE_SIZE];
    int subpq_size, subpq_rindex, subpq_windex;
    pthread_mutex_t subpq_mutex;
    pthread_cond_t  subpq_cond;

    // ---------------------------------------
    double frame_timer;
    double frame_last_returned_time;
    double frame_last_filter_delay;
    int video_stream;
    AVStream *video_st;
    PacketQueue videoq;
    int64_t video_current_pos;      // current displayed file pos
    double max_frame_duration;      // maximum duration of a frame - above this, we consider the jump a timestamp discontinuity
    VideoPicture pictq[VIDEO_PICTURE_QUEUE_SIZE];
    int pictq_size, pictq_rindex, pictq_windex;
    pthread_mutex_t pictq_mutex;
    pthread_cond_t pictq_cond;
    struct SwsContext *img_convert_ctx;

    // ---------------------------------------
    char filename[1024];
    int width, height, xleft, ytop;
    int step;

    int last_video_stream, last_audio_stream, last_subtitle_stream;

    // ---------------------------------------
    pthread_cond_t  continue_read_thread;
} VideoState;

////////////////////////////////////////////////////////////////////////////////

typedef enum
{
    FFPLAYMOD_CTRL_ERR      = -1,
    FFPLAYMOD_CTRL_NONE     = 0,
    FFPLAYMOD_CTRL_LOAD,
    FFPLAYMOD_CTRL_PLAY,
    FFPLAYMOD_CTRL_PAUSE,
    FFPLAYMOD_CTRL_RESUME,
    FFPLAYMOD_CTRL_SEEK,
    FFPLAYMOD_CTRL_STOP,
    FFPLAYMOD_CTRL_UNLOAD,
    FFPLAYMOD_CTRL_MAX
}ffplaymod_controlsigtype;

////////////////////////////////////////////////////////////////////////////////

typedef void (*ffplaymod_controlsig)(void* p, int a);

////////////////////////////////////////////////////////////////////////////////

int          ffplaymod_init( int argc, char** argv );
int          ffplaymod_final();
int          ffplaymod_open( const char* uri );
int          ffplaymod_close();
int          ffplaymod_playcontrol( int controltype );
int          ffplaymod_pause_state();
unsigned int ffplaymod_getvideobuffer( char** buffer );
unsigned int ffplaymod_getaudiobuffer( char** buffer, unsigned* dur_ms );
VideoState*  ffplaymod_getVideoState();

void        ffplaymod_set_controlsig( ffplaymod_controlsig fcs, void* param );
const char* ffplaymod_get_lasterrstr();

////////////////////////////////////////////////////////////////////////////////

// User side calls
void        ffplaymod_port_content_size( int width, int height );
void        ffplaymod_port_content_title( const char* title );

// User side audio
int         ffplaymod_port_audio_buffersize();
int         ffplaymod_port_audio_open( int channels, int rate );
int         ffplaymod_port_audio_close();

#endif /// of __FFPLAYMOD_H__
