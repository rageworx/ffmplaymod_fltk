#include <Fl/Fl.H>
#include <Fl/Fl_Image.H>
#include <Fl/Fl_RGB_Image.H>
#include <Fl/Fl_Image_Surface.H>

#include "ffplaymod.h"
#include "appmain.h"

const char program_name[] = "ffplaymod";
const int program_birth_year = 2003;

#define MAX_QUEUE_SIZE (15 * 1024 * 1024)
#define MIN_FRAMES 5

static int64_t sws_flags = SWS_BICUBIC;

/* options specified by the user */
static VideoState* input_stream = NULL;
static AVInputFormat *file_iformat;
static char* content_title = NULL;
static char* input_filename = NULL;

typedef struct _ffplaymod_configuration
{
    int default_width;
    int default_height;
    int screen_width;
    int screen_height;
    int audio_disable;
    int video_disable;
    int subtitle_disable;
    int wanted_stream[AVMEDIA_TYPE_NB];
    int seek_by_bytes;
    int av_sync_type;
    int64_t start_time;
    int64_t duration;
    int workaround_bugs;
    int fast;
    int genpts;
    int lowres;
    int error_concealment;
    int decoder_reorder_pts;
    int autoexit;
    int loop;
    int framedrop;
    int infinite_buffer;
}ffplaymod_configuration;

static ffplaymod_configuration ffplaymodconf = {0};
static const char *audio_codec_name;
static const char *subtitle_codec_name;
static const char *video_codec_name;
double rdftspeed                            = 0.02;
static int cursor_hidden                    = 0;

static char *vfilters = NULL;
static char *afilters = NULL;

/* current context */
static int64_t audio_callback_time;
static int     audio_temp_size = 0;

static AVPacket flush_pkt;

static char err_strs[240] = {0};
static ffplaymod_controlsig fctrlsig = NULL;
static void*                fctrlsig_p = NULL;

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

appMain* aMain = NULL;

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

static inline
int cmp_audio_fmts(enum AVSampleFormat fmt1, int64_t channel_count1,
                   enum AVSampleFormat fmt2, int64_t channel_count2)
{
    /* If channel count == 1, planar and non-planar formats are the same */
    if (channel_count1 == 1 && channel_count2 == 1)
        return av_get_packed_sample_fmt(fmt1) != av_get_packed_sample_fmt(fmt2);
    else
        return channel_count1 != channel_count2 || fmt1 != fmt2;
}

static inline
int64_t get_valid_channel_layout(int64_t channel_layout, int channels)
{
    if (channel_layout && av_get_channel_layout_nb_channels(channel_layout) == channels)
        return channel_layout;
    else
        return 0;
}

static void reconfigure_ffplaymod_conf( ffplaymod_configuration* config )
{
    if ( config != NULL )
    {
        config->default_width       = 640;
        config->default_height      = 480;
        config->screen_width        = 0;
        config->screen_height       = 0;
        config->audio_disable       = 0;
        config->video_disable       = 0;
        config->subtitle_disable    = 1;
        config->seek_by_bytes       = -1;
        config->av_sync_type        = AV_SYNC_AUDIO_MASTER;
        config->start_time          = AV_NOPTS_VALUE;
        config->duration            = AV_NOPTS_VALUE;
        config->workaround_bugs     = 1;
        config->fast                = 1;
        config->genpts              = 0;
        config->lowres              = 0;
        config->error_concealment   = 3;
        config->decoder_reorder_pts = -1;
        config->autoexit            = 0;
        config->loop                = 1;
        //config->framedrop           = -1;
        config->framedrop           = 1;
        config->infinite_buffer     = -1;

        for( int cnt=0; cnt<AVMEDIA_TYPE_NB; cnt++ )
        {
            config->wanted_stream[cnt] = -1;
        }
    }
}

static int packet_queue_put_private(PacketQueue *q, AVPacket *pkt)
{
    MyAVPacketList *pkt1;

    if (q->abort_request)
       return -1;

    pkt1 = av_malloc(sizeof(MyAVPacketList));
    if (!pkt1)
        return -1;
    pkt1->pkt = *pkt;
    pkt1->next = NULL;
    if (pkt == &flush_pkt)
        q->serial++;
    pkt1->serial = q->serial;

    if (!q->last_pkt)
        q->first_pkt = pkt1;
    else
        q->last_pkt->next = pkt1;
    q->last_pkt = pkt1;
    q->nb_packets++;
    q->size += pkt1->pkt.size + sizeof(*pkt1);
    /* XXX: should duplicate packet data in DV case */
    pthread_cond_signal(&q->cond);
    return 0;
}

static int packet_queue_put(PacketQueue *q, AVPacket *pkt)
{
    int ret;

    /* duplicate the packet */
    if (pkt != &flush_pkt && av_dup_packet(pkt) < 0)
        return -1;

    pthread_mutex_lock(&q->mutex);
    ret = packet_queue_put_private(q, pkt);
    pthread_mutex_unlock(&q->mutex);

    if (pkt != &flush_pkt && ret < 0)
        av_free_packet(pkt);

    return ret;
}

static int packet_queue_put_nullpacket(PacketQueue *q, int stream_index)
{
    AVPacket pkt1, *pkt = &pkt1;
    av_init_packet(pkt);
    pkt->data = NULL;
    pkt->size = 0;
    pkt->stream_index = stream_index;
    return packet_queue_put(q, pkt);
}

/* packet queue handling */
static void packet_queue_init(PacketQueue *q)
{
    memset(q, 0, sizeof(PacketQueue));
    q->mutex = PTHREAD_MUTEX_INITIALIZER;
    q->cond = PTHREAD_COND_INITIALIZER;
    q->abort_request = 1;
}

static void packet_queue_flush(PacketQueue *q)
{
    MyAVPacketList *pkt, *pkt1;

    pthread_mutex_lock(&q->mutex);

    for (pkt = q->first_pkt; pkt != NULL; pkt = pkt1)
    {
        pkt1 = pkt->next;
        av_free_packet(&pkt->pkt);
        av_freep(&pkt);
    }
    q->last_pkt = NULL;
    q->first_pkt = NULL;
    q->nb_packets = 0;
    q->size = 0;

    pthread_mutex_unlock(&q->mutex);
}

static void packet_queue_destroy(PacketQueue *q)
{
    packet_queue_flush(q);
    pthread_mutex_destroy(&q->mutex);
    pthread_cond_destroy(&q->cond);
}

static void packet_queue_abort(PacketQueue *q)
{
    pthread_mutex_lock(&q->mutex);
    q->abort_request = 1;
    pthread_cond_signal(&q->cond);
    pthread_mutex_unlock(&q->mutex);
}

static void packet_queue_start(PacketQueue *q)
{
    pthread_mutex_lock(&q->mutex);
    q->abort_request = 0;
    packet_queue_put_private(q, &flush_pkt);
    pthread_mutex_unlock(&q->mutex);
}

/* return < 0 if aborted, 0 if no packet and > 0 if packet.  */
static int packet_queue_get(PacketQueue *q, AVPacket *pkt, int block, int *serial)
{
    MyAVPacketList *pkt1;
    int ret;

    pthread_mutex_lock(&q->mutex);

    for (;;) {
        if (q->abort_request) {
            ret = -1;
            break;
        }

        pkt1 = q->first_pkt;
        if (pkt1) {
            q->first_pkt = pkt1->next;
            if (!q->first_pkt)
                q->last_pkt = NULL;
            q->nb_packets--;
            q->size -= pkt1->pkt.size + sizeof(*pkt1);
            *pkt = pkt1->pkt;
            if (serial)
                *serial = pkt1->serial;
            av_free(pkt1);
            ret = 1;
            break;
        } else if (!block) {
            ret = 0;
            break;
        } else {
            pthread_cond_wait(&q->cond, &q->mutex);
        }
    }
    pthread_mutex_unlock(&q->mutex);
    return ret;
}

static void free_picture(VideoPicture *vp)
{
    if ( vp->bmp != NULL )
    {
        delete[] vp->bmp;

        vp->bmp = NULL;
    }

    vp->bmpsize = 0;
}

static void free_subpicture(SubPicture *sp)
{
    avsubtitle_free(&sp->sub);
}

static inline int compute_mod(int a, int b)
{
    return a < 0 ? a%b + b : a%b;
}

static void stream_close(VideoState *is)
{
    int i;
    /* XXX: use a special url_shutdown call to abort parse cleanly */
    is->abort_request = 1;
    pthread_join(is->read_tid, NULL);
    packet_queue_destroy(&is->videoq);
    packet_queue_destroy(&is->audioq);
    packet_queue_destroy(&is->subtitleq);

    if ( fctrlsig != NULL )
    {
        fctrlsig(fctrlsig_p, FFPLAYMOD_CTRL_STOP);
    }

    /* free all pictures */
    for (i = 0; i < VIDEO_PICTURE_QUEUE_SIZE; i++)
    {
        free_picture(&is->pictq[i]);
    }

    for (i = 0; i < SUBPICTURE_QUEUE_SIZE; i++)
        free_subpicture(&is->subpq[i]);

    pthread_mutex_destroy(&is->pictq_mutex);
    pthread_cond_destroy(&is->pictq_cond);
    pthread_mutex_destroy(&is->subpq_mutex);
    pthread_cond_destroy(&is->subpq_cond);
    pthread_cond_destroy(&is->continue_read_thread);

    sws_freeContext(is->img_convert_ctx);

    av_free(is);

    if ( fctrlsig != NULL )
    {
        fctrlsig(fctrlsig_p, FFPLAYMOD_CTRL_UNLOAD);
    }
}

static void do_stopall(VideoState *is)
{
    if (is)
    {
        stream_close(is);
    }

    is = NULL;

    av_log(NULL, AV_LOG_QUIET, "%s", "");

}

static void sigterm_handler(int sig)
{
    exit(123);
}

static int video_open(VideoState *is, int force_set_video_mode, VideoPicture *vp)
{
    is->width  = vp->width;
    is->height = vp->height;

    ffplaymod_port_content_title( content_title );
    ffplaymod_port_content_size( is->width, is->height );

    return 0;
}

static double get_clock(Clock *c)
{
    if (*c->queue_serial != c->serial)
        return NAN;
    if (c->paused) {
        return c->pts;
    } else {
        double time = av_gettime() / 1000000.0;
        return c->pts_drift + time - (time - c->last_updated) * (1.0 - c->speed);
    }
}

static void set_clock_at(Clock *c, double pts, int serial, double time)
{
    c->pts = pts;
    c->last_updated = time;
    c->pts_drift = c->pts - time;
    c->serial = serial;
}

static void set_clock(Clock *c, double pts, int serial)
{
    double time = av_gettime() / 1000000.0;
    set_clock_at(c, pts, serial, time);
}

static void set_clock_speed(Clock *c, double speed)
{
    set_clock(c, get_clock(c), c->serial);
    c->speed = speed;
}

static void init_clock(Clock *c, int *queue_serial)
{
    c->speed = 1.0;
    c->paused = 0;
    c->queue_serial = queue_serial;
    set_clock(c, NAN, -1);
}

static void sync_clock_to_slave(Clock *c, Clock *slave)
{
    double clock = get_clock(c);
    double slave_clock = get_clock(slave);
    if (!isnan(slave_clock) && (isnan(clock) || fabs(clock - slave_clock) > AV_NOSYNC_THRESHOLD))
        set_clock(c, slave_clock, slave->serial);
}

static int get_master_sync_type(VideoState *is)
{
    if (is->av_sync_type == AV_SYNC_VIDEO_MASTER)
    {
        if (is->video_st)
            return AV_SYNC_VIDEO_MASTER;
        else
            return AV_SYNC_AUDIO_MASTER;
    }
    else
    if (is->av_sync_type == AV_SYNC_AUDIO_MASTER)
    {
        if (is->audio_st)
            return AV_SYNC_AUDIO_MASTER;
        else
            return AV_SYNC_EXTERNAL_CLOCK;
    }
    else
    {
        return AV_SYNC_EXTERNAL_CLOCK;
    }
}

/* get the current master clock value */
static double get_master_clock(VideoState *is)
{
    double val;

    switch (get_master_sync_type(is))
    {
        case AV_SYNC_VIDEO_MASTER:
            val = get_clock(&is->vidclk);
            break;
        case AV_SYNC_AUDIO_MASTER:
            val = get_clock(&is->audclk);
            break;
        default:
            val = get_clock(&is->extclk);
            break;
    }
    return val;
}

static void check_external_clock_speed(VideoState *is)
{
   if (is->video_stream >= 0 && is->videoq.nb_packets <= MIN_FRAMES / 2 ||
       is->audio_stream >= 0 && is->audioq.nb_packets <= MIN_FRAMES / 2) {
       set_clock_speed(&is->extclk, FFMAX(EXTERNAL_CLOCK_SPEED_MIN, is->extclk.speed - EXTERNAL_CLOCK_SPEED_STEP));
   } else if ((is->video_stream < 0 || is->videoq.nb_packets > MIN_FRAMES * 2) &&
              (is->audio_stream < 0 || is->audioq.nb_packets > MIN_FRAMES * 2)) {
       set_clock_speed(&is->extclk, FFMIN(EXTERNAL_CLOCK_SPEED_MAX, is->extclk.speed + EXTERNAL_CLOCK_SPEED_STEP));
   } else {
       double speed = is->extclk.speed;
       if (speed != 1.0)
           set_clock_speed(&is->extclk, speed + EXTERNAL_CLOCK_SPEED_STEP * (1.0 - speed) / fabs(1.0 - speed));
   }
}

/* seek in the stream */
static void stream_seek(VideoState *is, int64_t pos, int64_t rel, int seek_by_bytes)
{
    if (!is->seek_req)
    {
        is->seek_pos = pos;
        is->seek_rel = rel;
        is->seek_flags &= ~AVSEEK_FLAG_BYTE;
        if (seek_by_bytes)
            is->seek_flags |= AVSEEK_FLAG_BYTE;
        is->seek_req = 1;
        pthread_cond_signal(&is->continue_read_thread);

        if ( fctrlsig != NULL )
        {
            fctrlsig(fctrlsig_p, FFPLAYMOD_CTRL_SEEK);
        }
    }
}

/* pause or resume the video */
static void stream_toggle_pause(VideoState *is)
{
    if (is->paused)
    {
        is->frame_timer += av_gettime() / 1000000.0 + is->vidclk.pts_drift - is->vidclk.pts;
        if (is->read_pause_return != AVERROR(ENOSYS)) {
            is->vidclk.paused = 0;
        }
        set_clock(&is->vidclk, get_clock(&is->vidclk), is->vidclk.serial);

        if ( fctrlsig != NULL )
        {
            fctrlsig(fctrlsig_p, FFPLAYMOD_CTRL_RESUME);
        }
    }
    else
    {
        if ( fctrlsig != NULL )
        {
            fctrlsig(fctrlsig_p, FFPLAYMOD_CTRL_PAUSE);
        }
    }
    set_clock(&is->extclk, get_clock(&is->extclk), is->extclk.serial);
    is->paused = is->audclk.paused = is->vidclk.paused = is->extclk.paused = !is->paused;
}

static void toggle_pause(VideoState *is)
{
    stream_toggle_pause(is);
    is->step = 0;
}

static void step_to_next_frame(VideoState *is)
{
    /* if the stream is paused unpause it, then step */
    if (is->paused)
        stream_toggle_pause(is);
    is->step = 1;
}

static double compute_target_delay(double delay, VideoState *is)
{
    double sync_threshold, diff;

    /* update delay to follow master synchronisation source */
    if (get_master_sync_type(is) != AV_SYNC_VIDEO_MASTER) {
        /* if video is slave, we try to correct big delays by
           duplicating or deleting a frame */
        diff = get_clock(&is->vidclk) - get_master_clock(is);

        /* skip or repeat frame. We take into account the
           delay to compute the threshold. I still don't know
           if it is the best guess */
        sync_threshold = FFMAX(AV_SYNC_THRESHOLD_MIN, FFMIN(AV_SYNC_THRESHOLD_MAX, delay));
        if (!isnan(diff) && fabs(diff) < is->max_frame_duration) {
            if (diff <= -sync_threshold)
                delay = FFMAX(0, delay + diff);
            else if (diff >= sync_threshold && delay > AV_SYNC_FRAMEDUP_THRESHOLD)
                delay = delay + diff;
            else if (diff >= sync_threshold)
                delay = 2 * delay;
        }
    }

    av_dlog(NULL, "video: delay=%0.3f A-V=%f\n",
            delay, -diff);

    return delay;
}

static double vp_duration(VideoState *is, VideoPicture *vp, VideoPicture *nextvp) {
    if (vp->serial == nextvp->serial) {
        double duration = nextvp->pts - vp->pts;
        if (isnan(duration) || duration <= 0 || duration > is->max_frame_duration)
            return vp->duration;
        else
            return duration;
    } else {
        return 0.0;
    }
}

static void pictq_next_picture(VideoState *is)
{
    /* update queue size and signal for next picture */
    if (++is->pictq_rindex == VIDEO_PICTURE_QUEUE_SIZE)
        is->pictq_rindex = 0;

    pthread_mutex_lock(&is->pictq_mutex);
    is->pictq_size--;
    pthread_cond_signal(&is->pictq_cond);
    pthread_mutex_unlock(&is->pictq_mutex);
}

static int pictq_prev_picture(VideoState *is) {
    VideoPicture *prevvp;
    int ret = 0;
    /* update queue size and signal for the previous picture */
    prevvp = &is->pictq[(is->pictq_rindex + VIDEO_PICTURE_QUEUE_SIZE - 1) % VIDEO_PICTURE_QUEUE_SIZE];
    if (prevvp->allocated && prevvp->serial == is->videoq.serial) {
        pthread_mutex_lock(&is->pictq_mutex);
        if (is->pictq_size < VIDEO_PICTURE_QUEUE_SIZE) {
            if (--is->pictq_rindex == -1)
                is->pictq_rindex = VIDEO_PICTURE_QUEUE_SIZE - 1;
            is->pictq_size++;
            ret = 1;
        }
        pthread_cond_signal(&is->pictq_cond);
        pthread_mutex_unlock(&is->pictq_mutex);
    }
    return ret;
}

static void update_video_pts(VideoState *is, double pts, int64_t pos, int serial)
{
    /* update current video pts */
    set_clock(&is->vidclk, pts, serial);
    sync_clock_to_slave(&is->extclk, &is->vidclk);
    is->video_current_pos = pos;
}

/* called to display each frame */
static void video_refresh(void *opaque, double *remaining_time)
{
    VideoState *is = opaque;
    double time;

    SubPicture *sp, *sp2;

    if (!is->paused && get_master_sync_type(is) == AV_SYNC_EXTERNAL_CLOCK && is->realtime)
        check_external_clock_speed(is);

    if (is->audio_st)
    {
        time = av_gettime() / 1000000.0;
        if (is->force_refresh || is->last_vis_time + rdftspeed < time)
        {
            //video_image_display(is);
            //audio_callback(is);
            is->last_vis_time = time;
        }
        *remaining_time = FFMIN(*remaining_time, is->last_vis_time + rdftspeed - time);
    }

    if (is->video_st)
    {
        int redisplay = 0;
        if (is->force_refresh)
            redisplay = pictq_prev_picture(is);
retry:
        if (is->pictq_size == 0) {
            // nothing to do, no picture to display in the queue
        } else {
            double last_duration, duration, delay;
            VideoPicture *vp, *lastvp;

            /* dequeue the picture */
            vp = &is->pictq[is->pictq_rindex];
            lastvp = &is->pictq[(is->pictq_rindex + VIDEO_PICTURE_QUEUE_SIZE - 1) % VIDEO_PICTURE_QUEUE_SIZE];

            if (vp->serial != is->videoq.serial) {
                pictq_next_picture(is);
                is->video_current_pos = -1;
                redisplay = 0;
                goto retry;
            }

            if (lastvp->serial != vp->serial && !redisplay)
                is->frame_timer = av_gettime() / 1000000.0;

            if (is->paused)
                goto display;

            /* compute nominal last_duration */
            last_duration = vp_duration(is, lastvp, vp);
            if (redisplay)
                delay = 0.0;
            else
                delay = compute_target_delay(last_duration, is);

            time= av_gettime()/1000000.0;
            if (time < is->frame_timer + delay && !redisplay) {
                *remaining_time = FFMIN(is->frame_timer + delay - time, *remaining_time);
                return;
            }

            is->frame_timer += delay;
            if (delay > 0 && time - is->frame_timer > AV_SYNC_THRESHOLD_MAX)
                is->frame_timer = time;

            pthread_mutex_lock(&is->pictq_mutex);
            if (!redisplay && !isnan(vp->pts))
                update_video_pts(is, vp->pts, vp->pos, vp->serial);
            pthread_mutex_unlock(&is->pictq_mutex);

            if (is->pictq_size > 1)
            {
                VideoPicture *nextvp = &is->pictq[(is->pictq_rindex + 1) % VIDEO_PICTURE_QUEUE_SIZE];
                duration = vp_duration(is, vp, nextvp);
                if( !is->step && (redisplay || ffplaymodconf.framedrop>0 ||
                    (ffplaymodconf.framedrop && get_master_sync_type(is) != AV_SYNC_VIDEO_MASTER))
                    && time > is->frame_timer + duration)
                {
                    if (!redisplay)
                        is->frame_drops_late++;
                    pictq_next_picture(is);
                    redisplay = 0;
                    goto retry;
                }
            }

            if (is->subtitle_st)
            {
                    while (is->subpq_size > 0)
                    {
                        sp = &is->subpq[is->subpq_rindex];

                        if (is->subpq_size > 1)
                            sp2 = &is->subpq[(is->subpq_rindex + 1) % SUBPICTURE_QUEUE_SIZE];
                        else
                            sp2 = NULL;

                        if ( sp->serial != is->subtitleq.serial
                                || (is->vidclk.pts > (sp->pts + ((float) sp->sub.end_display_time / 1000)))
                                || (sp2 && is->vidclk.pts > (sp2->pts + ((float) sp2->sub.start_display_time / 1000))) )
                        {
                            free_subpicture(sp);

                            /* update queue size and signal for next picture */
                            if (++is->subpq_rindex == SUBPICTURE_QUEUE_SIZE)
                                is->subpq_rindex = 0;

                            pthread_mutex_lock(&is->subpq_mutex);
                            is->subpq_size--;
                            pthread_cond_signal(&is->subpq_cond);
                            pthread_mutex_unlock(&is->subpq_mutex);
                        }
                        else
                        {
                            break;
                        }
                    }
            }

display:
            pictq_next_picture(is);

            if (is->step && !is->paused)
                stream_toggle_pause(is);
        }
    }

    is->force_refresh = 0;
}

static void render_surface(VideoState* is)
{
    double remaining_time = 0.0;

    if (remaining_time > 0.0)
        av_usleep((int64_t)(remaining_time * 1000000.0));

    remaining_time = REFRESH_RATE;

    if (!is->paused)
    {
        video_refresh(is, &remaining_time);
    }
}

/* allocate a picture (needs to do that in main thread to avoid
   potential locking problems */
static void alloc_picture(VideoState *is)
{
    VideoPicture *vp;
    int64_t bufferdiff;

    vp = &is->pictq[is->pictq_windex];

    free_picture(vp);

    video_open(is, 0, vp);

    int video_conv_size = avpicture_get_size(AV_PIX_FMT_BGR32, is->width, is->height);
    if ( video_conv_size < 0 )
    {
        video_conv_size = is->width * is->height * 4;
    }

    vp->ready = 0;

    if ( vp->bmp != NULL )
    {
        delete[] vp->bmp;
        vp->bmp = NULL;
        vp->bmpsize = 0;
    }

    vp->bmp = new unsigned char[ video_conv_size ];
    if ( vp->bmp == NULL )
    {
        sprintf( err_strs, "Error: the video system does not support an image\n"
                           "size of %dx%d pixels. Try using -lowres or -vf \"scale=w:h\"\n"
                           "to reduce the image size.\n", vp->width, vp->height );

        av_log(NULL, AV_LOG_FATAL, err_strs);

        if ( fctrlsig != NULL )
        {
            fctrlsig(fctrlsig_p, FFPLAYMOD_CTRL_ERR );
        }

        do_stopall(is);
    }
    vp->bmpsize = video_conv_size;

    pthread_mutex_lock(&is->pictq_mutex);
    vp->allocated = 1;
    pthread_cond_signal(&is->pictq_cond);
    pthread_mutex_unlock(&is->pictq_mutex);
}

static int queue_picture(VideoState *is, AVFrame *src_frame, double pts, double duration, int64_t pos, int serial)
{
    VideoPicture *vp;

#if defined(DEBUG_SYNC) && 0
    printf("frame_type=%c pts=%0.3f\n",
           av_get_picture_type_char(src_frame->pict_type), pts);
#endif

    /* wait until we have space to put a new picture */
    pthread_mutex_lock(&is->pictq_mutex);

    /* keep the last already displayed picture in the queue */
    while (is->pictq_size >= VIDEO_PICTURE_QUEUE_SIZE - 1)
    {
        pthread_cond_wait(&is->pictq_cond, &is->pictq_mutex);
    }

    pthread_mutex_unlock(&is->pictq_mutex);

    if (is->videoq.abort_request)
        return -1;

    vp = &is->pictq[is->pictq_windex];
    vp->sar = src_frame->sample_aspect_ratio;

    if (!vp->bmp || !vp->allocated || vp->width  != src_frame->width || vp->height != src_frame->height )
    {
        vp->allocated  = 0;
        vp->width = src_frame->width;
        vp->height = src_frame->height;
        alloc_picture(is);

        if (is->videoq.abort_request)
        {
            return -1;
        }
    }

    /* if the frame is not skipped, then display it */
    if (vp->bmp)
    {
        AVFrame* pict = avcodec_alloc_frame();

        vp->ready = 0;

        avpicture_fill((AVPicture*)pict, vp->bmp, AV_PIX_FMT_BGR32, vp->width, vp->height);

        if (is->img_convert_ctx == NULL)
        {
            is->img_convert_ctx = sws_getCachedContext( is->img_convert_ctx,
                                                        vp->width,
                                                        vp->height,
                                                        src_frame->format,
                                                        vp->width,
                                                        vp->height,
                                                        PIX_FMT_BGR32,
                                                        sws_flags,
                                                        NULL,
                                                        NULL,
                                                        NULL);
        }

        if (is->img_convert_ctx == NULL)
        {
            av_log(NULL, AV_LOG_FATAL, "Cannot initialize the conversion context\n");
            exit(1);
        }

        int reti = sws_scale( is->img_convert_ctx,
                              src_frame->data,
                              src_frame->linesize,
                              0,
                              vp->height,
                              pict->data,
                              pict->linesize );

        if ( reti <= 0 )
        {
            av_log(NULL, AV_LOG_FATAL, "sws_scale() failure !\n");
            return 0;
        }
        else
        {
            static int swscnt = 0;
            swscnt++;
        }

        memcpy( vp->bmp, pict->data[0], vp->bmpsize );

        vp->ready = 1;
        vp->pts = pts;
        vp->duration = duration;
        vp->pos = pos;
        vp->serial = serial;

        /* now we can update the picture count */
        if (++is->pictq_windex == VIDEO_PICTURE_QUEUE_SIZE)
            is->pictq_windex = 0;

        pthread_mutex_lock(&is->pictq_mutex);
        is->pictq_size++;
        pthread_mutex_unlock(&is->pictq_mutex);

    }
    return 0;
}

static int get_video_frame(VideoState *is, AVFrame *frame, AVPacket *pkt, int *serial)
{
    int got_picture;

    if (packet_queue_get(&is->videoq, pkt, 1, serial) < 0)
        return -1;

    if (pkt->data == flush_pkt.data) {
        avcodec_flush_buffers(is->video_st->codec);
        return 0;
    }

    if(avcodec_decode_video2(is->video_st->codec, frame, &got_picture, pkt) < 0)
        return 0;

    if (!got_picture && !pkt->data)
        is->video_finished = *serial;

    if (got_picture)
    {
        int ret = 1;
        double dpts = NAN;

        if (ffplaymodconf.decoder_reorder_pts == -1)
        {
            frame->pts = av_frame_get_best_effort_timestamp(frame);
        }
        else
        if (ffplaymodconf.decoder_reorder_pts)
        {
            frame->pts = frame->pkt_pts;
        }
        else
        {
            frame->pts = frame->pkt_dts;
        }

        if (frame->pts != AV_NOPTS_VALUE)
            dpts = av_q2d(is->video_st->time_base) * frame->pts;

        frame->sample_aspect_ratio = av_guess_sample_aspect_ratio(is->ic, is->video_st, frame);

        if ( ffplaymodconf.framedrop>0 ||
             (ffplaymodconf.framedrop && get_master_sync_type(is) != AV_SYNC_VIDEO_MASTER) )
        {
            if (frame->pts != AV_NOPTS_VALUE)
            {
                double diff = dpts - get_master_clock(is);

                if (!isnan(diff) && fabs(diff) < AV_NOSYNC_THRESHOLD &&
                    diff - is->frame_last_filter_delay < 0 &&
                    *serial == is->vidclk.serial &&
                    is->videoq.nb_packets)
                {
                    is->frame_drops_early++;
                    av_frame_unref(frame);
                    ret = 0;
                }
            }
        }

        return ret;
    }
    return 0;
}

#if CONFIG_AVFILTER
static int configure_filtergraph(AVFilterGraph *graph, const char *filtergraph,
                                 AVFilterContext *source_ctx, AVFilterContext *sink_ctx)
{
    int ret, i;
    int nb_filters = graph->nb_filters;
    AVFilterInOut *outputs = NULL, *inputs = NULL;

    if (filtergraph) {
        outputs = avfilter_inout_alloc();
        inputs  = avfilter_inout_alloc();
        if (!outputs || !inputs)
        {
            ret = AVERROR(ENOMEM);
            goto fail;
        }

        outputs->name       = av_strdup("in");
        outputs->filter_ctx = source_ctx;
        outputs->pad_idx    = 0;
        outputs->next       = NULL;

        inputs->name        = av_strdup("out");
        inputs->filter_ctx  = sink_ctx;
        inputs->pad_idx     = 0;
        inputs->next        = NULL;

        if ((ret = avfilter_graph_parse_ptr(graph, filtergraph, &inputs, &outputs, NULL)) < 0)
            goto fail;
    } else {
        if ((ret = avfilter_link(source_ctx, 0, sink_ctx, 0)) < 0)
            goto fail;
    }

    /* Reorder the filters to ensure that inputs of the custom filters are merged first */
    for (i = 0; i < graph->nb_filters - nb_filters; i++)
        FFSWAP(AVFilterContext*, graph->filters[i], graph->filters[i + nb_filters]);

    ret = avfilter_graph_config(graph, NULL);
fail:
    avfilter_inout_free(&outputs);
    avfilter_inout_free(&inputs);
    return ret;
}

#endif  /* CONFIG_AVFILTER */

static void* video_thread(void *arg)
{
    AVPacket pkt = { 0 };
    VideoState *is = arg;
    AVFrame *frame = av_frame_alloc();
    double pts;
    double duration;
    int ret;
    int serial = 0;
    AVRational tb = is->video_st->time_base;
    AVRational frame_rate = av_guess_frame_rate(is->ic, is->video_st, NULL);

    for (;;)
    {
        av_free_packet(&pkt);

        ret = get_video_frame(is, frame, &pkt, &serial);

        if (ret == 0 )
        {
            continue;
        }

        if (ret > 0)
        {
            duration = (frame_rate.num && frame_rate.den ? av_q2d((AVRational){frame_rate.den, frame_rate.num}) : 0);
            pts = (frame->pts == AV_NOPTS_VALUE) ? NAN : frame->pts * av_q2d(tb);
            ret = queue_picture(is, frame, pts, duration, av_frame_get_pkt_pos(frame), serial);
            av_frame_unref(frame);

            if (ret < 0)
                break;

        }
        else
        {
            break;
        }
    }

    av_log(NULL,AV_LOG_INFO,"video_thread going to end !\n");

    av_free_packet(&pkt);
    av_frame_free(&frame);

    // Skipping
    double incr, pos, frac;
    incr = 60;

    if (ffplaymodconf.seek_by_bytes)
    {
        if (is->video_stream >= 0 && is->video_current_pos >= 0)
        {
            pos = is->video_current_pos;
        }
        else
        if (is->audio_stream >= 0 && is->audio_pkt.pos >= 0)
        {
            pos = is->audio_pkt.pos;
        } else
            pos = avio_tell(is->ic->pb);

        if (is->ic->bit_rate)
            incr *= is->ic->bit_rate / 8.0;
        else
            incr *= 180000.0;

        pos += incr;
        stream_seek(is, pos, incr, 1);
    }
    else
    {
        pos = get_master_clock(is);
        if (isnan(pos))
            pos = (double)is->seek_pos / AV_TIME_BASE;
        pos += incr;
        if (is->ic->start_time != AV_NOPTS_VALUE && pos < is->ic->start_time / (double)AV_TIME_BASE)
            pos = is->ic->start_time / (double)AV_TIME_BASE;
        stream_seek(is, (int64_t)(pos * AV_TIME_BASE), (int64_t)(incr * AV_TIME_BASE), 0);
    }


    return NULL;
}

static void* subtitle_thread(void *arg)
{
    VideoState *is = arg;
    SubPicture *sp;
    AVPacket pkt1, *pkt = &pkt1;
    int got_subtitle;
    int serial;
    double pts;
    int i, j;
    int r, g, b, y, u, v, a;

    for (;;)
    {
        while (is->paused && !is->subtitleq.abort_request)
        {
            Sleep(10);
        }

        if (packet_queue_get(&is->subtitleq, pkt, 1, &serial) < 0)
            break;

        if (pkt->data == flush_pkt.data)
        {
            avcodec_flush_buffers(is->subtitle_st->codec);
            continue;
        }

        pthread_mutex_lock(&is->subpq_mutex);

        while ( is->subpq_size >= SUBPICTURE_QUEUE_SIZE && !is->subtitleq.abort_request )
        {
            pthread_cond_wait(&is->subpq_cond, &is->subpq_mutex);
        }

        pthread_mutex_unlock(&is->subpq_mutex);

        if (is->subtitleq.abort_request)
            return 0;

        sp = &is->subpq[is->subpq_windex];

       /* NOTE: ipts is the PTS of the _first_ picture beginning in
           this packet, if any */
        pts = 0;

        if (pkt->pts != AV_NOPTS_VALUE)
        {
            pts = av_q2d(is->subtitle_st->time_base) * pkt->pts;
        }

        avcodec_decode_subtitle2(is->subtitle_st->codec, &sp->sub, &got_subtitle, pkt);

        if ( got_subtitle && sp->sub.format == 0 )
        {
            if (sp->sub.pts != AV_NOPTS_VALUE)
            {
                pts = sp->sub.pts / (double)AV_TIME_BASE;
            }

            sp->pts = pts;
            sp->serial = serial;

            /* now we can update the picture count */
            if (++is->subpq_windex == SUBPICTURE_QUEUE_SIZE)
            {
                is->subpq_windex = 0;
            }

            pthread_mutex_lock(&is->subpq_mutex);
            is->subpq_size++;
            pthread_mutex_unlock(&is->subpq_mutex);
        }
        else
        if (got_subtitle)
        {
            avsubtitle_free(&sp->sub);
        }

        av_free_packet(pkt);
    }

    return NULL;
}


/* copy samples for viewing in editor window */
static void update_sample_display(VideoState *is, short *samples, int samples_size)
{
    int size, len;

    size = samples_size / sizeof(short);
    while (size > 0)
    {
        len = SAMPLE_ARRAY_SIZE - is->sample_array_index;
        if (len > size)
            len = size;
        memcpy(is->sample_array + is->sample_array_index, samples, len * sizeof(short));
        samples += len;
        is->sample_array_index += len;
        if (is->sample_array_index >= SAMPLE_ARRAY_SIZE)
            is->sample_array_index = 0;
        size -= len;
    }
}

/* return the wanted number of samples to get better sync if sync_type is video
 * or external master clock */
static int synchronize_audio(VideoState *is, int nb_samples)
{
    int wanted_nb_samples = nb_samples;

    /* if not master, then we try to remove or add samples to correct the clock */
    if (get_master_sync_type(is) != AV_SYNC_AUDIO_MASTER)
    {
        double diff, avg_diff;
        int min_nb_samples, max_nb_samples;

        diff = get_clock(&is->audclk) - get_master_clock(is);

        if (!isnan(diff) && fabs(diff) < AV_NOSYNC_THRESHOLD)
        {
            is->audio_diff_cum = diff + is->audio_diff_avg_coef * is->audio_diff_cum;
            if (is->audio_diff_avg_count < AUDIO_DIFF_AVG_NB)
            {
                /* not enough measures to have a correct estimate */
                is->audio_diff_avg_count++;
            } else {
                /* estimate the A-V difference */
                avg_diff = is->audio_diff_cum * (1.0 - is->audio_diff_avg_coef);

                if (fabs(avg_diff) >= is->audio_diff_threshold) {
                    wanted_nb_samples = nb_samples + (int)(diff * is->audio_src.freq);
                    min_nb_samples = ((nb_samples * (100 - SAMPLE_CORRECTION_PERCENT_MAX) / 100));
                    max_nb_samples = ((nb_samples * (100 + SAMPLE_CORRECTION_PERCENT_MAX) / 100));
                    wanted_nb_samples = FFMIN(FFMAX(wanted_nb_samples, min_nb_samples), max_nb_samples);
                }
                av_dlog(NULL, "diff=%f adiff=%f sample_diff=%d apts=%0.3f %f\n",
                        diff, avg_diff, wanted_nb_samples - nb_samples,
                        is->audio_clock, is->audio_diff_threshold);
            }
        } else {
            /* too big difference : may be initial PTS errors, so reset A-V filter */
            is->audio_diff_avg_count = 0;
            is->audio_diff_cum       = 0;
        }
    }

    return wanted_nb_samples;
}

/**
 * Decode one audio frame and return its uncompressed size.
 *
 * The processed audio frame is decoded, converted if required, and
 * stored in is->audio_buf, with size in bytes given by the return
 * value.
 */
static int audio_decode_frame(VideoState *is)
{
    if ( !is->audio_st )
        return 0;

    AVPacket *pkt_temp = &is->audio_pkt_temp;
    AVPacket *pkt = &is->audio_pkt;
    AVCodecContext *dec = is->audio_st->codec;
    int len1, data_size, resampled_data_size;
    int64_t dec_channel_layout;
    int got_frame;
    av_unused double audio_clock0;
    int wanted_nb_samples;
    AVRational tb;
    int ret;
    int reconfigure;

    for (;;)
    {
        /* NOTE: the audio packet can contain several frames */
        while (pkt_temp->stream_index != -1 || is->audio_buf_frames_pending)
        {
            if (!is->frame)
            {
                if (!(is->frame = av_frame_alloc()))
                    return AVERROR(ENOMEM);
            } else {
                av_frame_unref(is->frame);
            }

            if (is->audioq.serial != is->audio_pkt_temp_serial)
                break;

            if (is->paused)
                return -1;

            if (!is->audio_buf_frames_pending) {
                len1 = avcodec_decode_audio4(dec, is->frame, &got_frame, pkt_temp);
                if (len1 < 0) {
                    /* if error, we skip the frame */
                    pkt_temp->size = 0;
                    break;
                }

                pkt_temp->dts =
                pkt_temp->pts = AV_NOPTS_VALUE;
                pkt_temp->data += len1;
                pkt_temp->size -= len1;
                if (pkt_temp->data && pkt_temp->size <= 0 || !pkt_temp->data && !got_frame)
                    pkt_temp->stream_index = -1;
                if (!pkt_temp->data && !got_frame)
                    is->audio_finished = is->audio_pkt_temp_serial;

                if (!got_frame)
                    continue;

                tb = (AVRational){1, is->frame->sample_rate};
                if (is->frame->pts != AV_NOPTS_VALUE)
                    is->frame->pts = av_rescale_q(is->frame->pts, dec->time_base, tb);
                else if (is->frame->pkt_pts != AV_NOPTS_VALUE)
                    is->frame->pts = av_rescale_q(is->frame->pkt_pts, is->audio_st->time_base, tb);
                else if (is->audio_frame_next_pts != AV_NOPTS_VALUE)
                    is->frame->pts = av_rescale_q(is->audio_frame_next_pts, (AVRational){1, is->audio_src.freq}, tb);

                if (is->frame->pts != AV_NOPTS_VALUE)
                    is->audio_frame_next_pts = is->frame->pts + is->frame->nb_samples;

            }

            data_size = av_samples_get_buffer_size(NULL, av_frame_get_channels(is->frame),
                                                   is->frame->nb_samples,
                                                   is->frame->format, 1);

            dec_channel_layout =
                (is->frame->channel_layout && av_frame_get_channels(is->frame) == av_get_channel_layout_nb_channels(is->frame->channel_layout)) ?
                is->frame->channel_layout : av_get_default_channel_layout(av_frame_get_channels(is->frame));
            wanted_nb_samples = synchronize_audio(is, is->frame->nb_samples);

            if (is->frame->format        != is->audio_src.fmt            ||
                dec_channel_layout       != is->audio_src.channel_layout ||
                is->frame->sample_rate   != is->audio_src.freq           ||
                (wanted_nb_samples       != is->frame->nb_samples && !is->swr_ctx)) {
                swr_free(&is->swr_ctx);
                is->swr_ctx = swr_alloc_set_opts(NULL,
                                                 is->audio_tgt.channel_layout, is->audio_tgt.fmt, is->audio_tgt.freq,
                                                 dec_channel_layout,           is->frame->format, is->frame->sample_rate,
                                                 0, NULL);
                if (!is->swr_ctx || swr_init(is->swr_ctx) < 0)
                {

                    av_log(NULL, AV_LOG_ERROR,
                           "Cannot create sample rate converter for conversion of %d Hz %s %d channels to %d Hz %s %d channels!\n",
                            is->frame->sample_rate, av_get_sample_fmt_name(is->frame->format), av_frame_get_channels(is->frame),
                            is->audio_tgt.freq, av_get_sample_fmt_name(is->audio_tgt.fmt), is->audio_tgt.channels);

                    if ( fctrlsig != NULL )
                    {
                        fctrlsig(fctrlsig_p, FFPLAYMOD_CTRL_ERR );
                    }

                    break;
                }
                is->audio_src.channel_layout = dec_channel_layout;
                is->audio_src.channels       = av_frame_get_channels(is->frame);
                is->audio_src.freq = is->frame->sample_rate;
                is->audio_src.fmt = is->frame->format;
            }

            if (is->swr_ctx) {
                const uint8_t **in = (const uint8_t **)is->frame->extended_data;
                uint8_t **out = &is->audio_buf1;
                int out_count = (int64_t)wanted_nb_samples * is->audio_tgt.freq / is->frame->sample_rate + 256;
                int out_size  = av_samples_get_buffer_size(NULL, is->audio_tgt.channels, out_count, is->audio_tgt.fmt, 0);
                int len2;
                if (out_size < 0) {
                    av_log(NULL, AV_LOG_ERROR, "av_samples_get_buffer_size() failed\n");
                    break;
                }
                if (wanted_nb_samples != is->frame->nb_samples) {
                    if (swr_set_compensation(is->swr_ctx, (wanted_nb_samples - is->frame->nb_samples) * is->audio_tgt.freq / is->frame->sample_rate,
                                                wanted_nb_samples * is->audio_tgt.freq / is->frame->sample_rate) < 0) {
                        av_log(NULL, AV_LOG_ERROR, "swr_set_compensation() failed\n");
                        break;
                    }
                }
                av_fast_malloc(&is->audio_buf1, &is->audio_buf1_size, out_size);
                if (!is->audio_buf1)
                    return AVERROR(ENOMEM);
                len2 = swr_convert(is->swr_ctx, out, out_count, in, is->frame->nb_samples);
                if (len2 < 0) {
                    av_log(NULL, AV_LOG_ERROR, "swr_convert() failed\n");
                    break;
                }
                if (len2 == out_count) {
                    av_log(NULL, AV_LOG_WARNING, "audio buffer is probably too small\n");
                    swr_init(is->swr_ctx);
                }
                is->audio_buf = is->audio_buf1;
                resampled_data_size = len2 * is->audio_tgt.channels * av_get_bytes_per_sample(is->audio_tgt.fmt);
            } else {
                is->audio_buf = is->frame->data[0];
                resampled_data_size = data_size;
            }

            audio_clock0 = is->audio_clock;
            /* update the audio clock with the pts */
            if (is->frame->pts != AV_NOPTS_VALUE)
                is->audio_clock = is->frame->pts * av_q2d(tb) + (double) is->frame->nb_samples / is->frame->sample_rate;
            else
                is->audio_clock = NAN;
            is->audio_clock_serial = is->audio_pkt_temp_serial;

            return resampled_data_size;
        }

        /* free the current packet */
        if (pkt->data)
            av_free_packet(pkt);

        memset(pkt_temp, 0, sizeof(*pkt_temp));
        pkt_temp->stream_index = -1;

        if (is->audioq.abort_request)
        {
            return -1;
        }

        if (is->audioq.nb_packets == 0)
            pthread_cond_signal(&is->continue_read_thread);

        /* read next packet */
        if ((packet_queue_get(&is->audioq, pkt, 1, &is->audio_pkt_temp_serial)) < 0)
            return -1;

        if (pkt->data == flush_pkt.data)
        {
            avcodec_flush_buffers(dec);
            is->audio_buf_frames_pending = 0;
            is->audio_frame_next_pts = AV_NOPTS_VALUE;
            if ((is->ic->iformat->flags & (AVFMT_NOBINSEARCH | AVFMT_NOGENSEARCH | AVFMT_NO_BYTE_SEEK)) && !is->ic->iformat->read_seek)
                is->audio_frame_next_pts = is->audio_st->start_time;
        }

        *pkt_temp = *pkt;
    }
}

static int audio_open(void *opaque, int64_t wanted_channel_layout, int wanted_nb_channels, int wanted_sample_rate, struct AudioParams *audio_hw_params)
{
    if ( ffplaymod_port_audio_open( wanted_nb_channels, wanted_sample_rate ) < 0 )
    {
        av_log(NULL,AV_LOG_INFO,"-> Failed to open audio out !\n");
        return -1;
    }

    audio_hw_params->fmt = AV_SAMPLE_FMT_S16;
    audio_hw_params->freq = wanted_sample_rate;
    audio_hw_params->channel_layout = wanted_channel_layout;
    audio_hw_params->channels =  wanted_nb_channels;
    audio_hw_params->frame_size = av_samples_get_buffer_size(NULL, audio_hw_params->channels, 1, audio_hw_params->fmt, 1);
    audio_hw_params->bytes_per_sec = av_samples_get_buffer_size(NULL, audio_hw_params->channels, audio_hw_params->freq, audio_hw_params->fmt, 1);

    //audio_temp_size = audio_hw_params->bytes_per_sec / 60;
    audio_temp_size = 256 * audio_hw_params->channels;

    if (audio_hw_params->bytes_per_sec <= 0 || audio_hw_params->frame_size <= 0)
    {
        av_log(NULL, AV_LOG_ERROR, "av_samples_get_buffer_size failed\n");
        return -1;
    }

    return audio_temp_size;
}

/* open a given stream. Return 0 if OK */
static int stream_component_open(VideoState *is, int stream_index)
{
    AVFormatContext *ic = is->ic;
    AVCodecContext *avctx;
    AVCodec *codec;
    const char *forced_codec_name = NULL;
    AVDictionary *opts;
    AVDictionaryEntry *t = NULL;
    int sample_rate, nb_channels;
    int64_t channel_layout;
    int ret;
    int stream_lowres = ffplaymodconf.lowres;

    if (stream_index < 0 || stream_index >= ic->nb_streams)
        return -1;

    avctx = ic->streams[stream_index]->codec;

    codec = avcodec_find_decoder(avctx->codec_id);

    switch(avctx->codec_type)
    {
        case AVMEDIA_TYPE_AUDIO   : is->last_audio_stream    = stream_index; forced_codec_name =    audio_codec_name; break;
        case AVMEDIA_TYPE_SUBTITLE: is->last_subtitle_stream = stream_index; forced_codec_name = subtitle_codec_name; break;
        case AVMEDIA_TYPE_VIDEO   : is->last_video_stream    = stream_index; forced_codec_name =    video_codec_name; break;
    }

    if (forced_codec_name)
        codec = avcodec_find_decoder_by_name(forced_codec_name);

    if (!codec) {
        if (forced_codec_name) av_log(NULL, AV_LOG_WARNING,
                                      "No codec could be found with name '%s'\n", forced_codec_name);
        else                   av_log(NULL, AV_LOG_WARNING,
                                      "No codec could be found with id %d\n", avctx->codec_id);
        return -1;
    }

    avctx->codec_id = codec->id;
    avctx->workaround_bugs   = ffplaymodconf.workaround_bugs;
    if(stream_lowres > av_codec_get_max_lowres(codec)){
        av_log(avctx, AV_LOG_WARNING, "The maximum value for lowres supported by the decoder is %d\n",
                av_codec_get_max_lowres(codec));
        stream_lowres = av_codec_get_max_lowres(codec);
    }
    av_codec_set_lowres(avctx, stream_lowres);
    avctx->error_concealment = ffplaymodconf.error_concealment;

    if(stream_lowres) avctx->flags |= CODEC_FLAG_EMU_EDGE;
    if (ffplaymodconf.fast)   avctx->flags2 |= CODEC_FLAG2_FAST;
    if(codec->capabilities & CODEC_CAP_DR1)
        avctx->flags |= CODEC_FLAG_EMU_EDGE;

    opts = filter_codec_opts(codec_opts, avctx->codec_id, ic, ic->streams[stream_index], codec);
    if (!av_dict_get(opts, "threads", NULL, 0))
        av_dict_set(&opts, "threads", "auto", 0);

    if (stream_lowres)
        av_dict_set(&opts, "lowres", av_asprintf("%d", stream_lowres), AV_DICT_DONT_STRDUP_VAL);

    if (avctx->codec_type == AVMEDIA_TYPE_VIDEO || avctx->codec_type == AVMEDIA_TYPE_AUDIO)
        av_dict_set(&opts, "refcounted_frames", "1", 0);

    if (avcodec_open2(avctx, codec, &opts) < 0)
        return -1;

    if ((t = av_dict_get(opts, "", NULL, AV_DICT_IGNORE_SUFFIX)))
    {
        av_log(NULL, AV_LOG_ERROR, "Option %s not found.\n", t->key);
        return AVERROR_OPTION_NOT_FOUND;
    }

    ic->streams[stream_index]->discard = AVDISCARD_DEFAULT;

    switch (avctx->codec_type)
    {
        case AVMEDIA_TYPE_AUDIO:
#ifdef DEBUG
            printf("AUDIO INFO:\n");
            printf("\tSampling rate: %d\n", avctx->sample_rate);
            printf("\tChannels     : %d\n", avctx->channels);
            printf("\tChannels lay : %d\n", avctx->channel_layout);
#endif
            sample_rate    = avctx->sample_rate;
            nb_channels    = avctx->channels;
            channel_layout = avctx->channel_layout;

            /* prepare audio output */
            if ((ret = audio_open(is, channel_layout, nb_channels, sample_rate, &is->audio_tgt)) < 0)
            {
#ifdef DEBUG
                printf("Audio open failure !\n");
#endif
                return ret;
            }
            is->audio_hw_buf_size = ret;
            is->audio_src = is->audio_tgt;
            is->audio_buf_size  = 0;
            is->audio_buf_index = 0;

            /* init averaging filter */
            is->audio_diff_avg_coef  = exp(log(0.01) / AUDIO_DIFF_AVG_NB);
            is->audio_diff_avg_count = 0;
            /* since we do not have a precise anough audio fifo fullness,
               we correct audio sync only if larger than this threshold */
            is->audio_diff_threshold = 2.0 * is->audio_hw_buf_size / is->audio_tgt.bytes_per_sec;

            memset(&is->audio_pkt, 0, sizeof(is->audio_pkt));
            memset(&is->audio_pkt_temp, 0, sizeof(is->audio_pkt_temp));
            is->audio_pkt_temp.stream_index = -1;

            is->audio_stream = stream_index;
            is->audio_st = ic->streams[stream_index];

            packet_queue_start(&is->audioq);
            break;

        case AVMEDIA_TYPE_VIDEO:
            is->video_stream = stream_index;
            is->video_st = ic->streams[stream_index];

            packet_queue_start(&is->videoq);

            pthread_create( &is->video_tid, NULL, video_thread, (void*)is );
            is->queue_attachments_req = 1;

            break;

        case AVMEDIA_TYPE_SUBTITLE:
            is->subtitle_stream = stream_index;
            is->subtitle_st = ic->streams[stream_index];
            packet_queue_start(&is->subtitleq);

            pthread_create( &is->subtitle_tid, NULL, subtitle_thread, (void*)is);
            break;

        default:
            break;
    }
    return 0;
}

static void stream_component_close(VideoState *is, int stream_index)
{
    AVFormatContext *ic = is->ic;
    AVCodecContext *avctx;

    if (stream_index < 0 || stream_index >= ic->nb_streams)
        return;
    avctx = ic->streams[stream_index]->codec;

    switch (avctx->codec_type)
    {
        case AVMEDIA_TYPE_AUDIO:
            packet_queue_abort(&is->audioq);

            ffplaymod_port_audio_close();

            packet_queue_flush(&is->audioq);
            av_free_packet(&is->audio_pkt);
            swr_free(&is->swr_ctx);
            av_freep(&is->audio_buf1);
            is->audio_buf1_size = 0;
            is->audio_buf = NULL;
            av_frame_free(&is->frame);

            if (is->rdft) {
                av_rdft_end(is->rdft);
                av_freep(&is->rdft_data);
                is->rdft = NULL;
                is->rdft_bits = 0;
            }
            break;
        case AVMEDIA_TYPE_VIDEO:
            packet_queue_abort(&is->videoq);

            /* note: we also signal this mutex to make sure we deblock the
               video thread in all cases */
            pthread_mutex_lock(&is->pictq_mutex);
            pthread_cond_signal(&is->pictq_cond);
            pthread_mutex_unlock(&is->pictq_mutex);

            pthread_join(is->video_tid, NULL);

            packet_queue_flush(&is->videoq);
            break;

        case AVMEDIA_TYPE_SUBTITLE:
            packet_queue_abort(&is->subtitleq);

            /* note: we also signal this mutex to make sure we deblock the
               video thread in all cases */
            pthread_mutex_lock(&is->subpq_mutex);
            pthread_cond_signal(&is->subpq_cond);
            pthread_mutex_unlock(&is->subpq_mutex);

            pthread_join(is->subtitle_tid, NULL);

            packet_queue_flush(&is->subtitleq);
            break;

        default:
            break;
    }

    ic->streams[stream_index]->discard = AVDISCARD_ALL;
    avcodec_close(avctx);

    switch (avctx->codec_type)
    {
        case AVMEDIA_TYPE_AUDIO:
            is->audio_st = NULL;
            is->audio_stream = -1;
            break;
        case AVMEDIA_TYPE_VIDEO:
            is->video_st = NULL;
            is->video_stream = -1;
            break;
        case AVMEDIA_TYPE_SUBTITLE:
            is->subtitle_st = NULL;
            is->subtitle_stream = -1;
            break;
        default:
            break;
    }
}

static int decode_interrupt_cb(void *ctx)
{
    VideoState *is = ctx;
    return is->abort_request;
}

static int is_realtime(AVFormatContext *s)
{
    if(   !strcmp(s->iformat->name, "rtp")
       || !strcmp(s->iformat->name, "rtsp")
       || !strcmp(s->iformat->name, "sdp")
    )
        return 1;

    if(s->pb && (   !strncmp(s->filename, "rtp:", 4)
                 || !strncmp(s->filename, "udp:", 4)
                )
    )
        return 1;
    return 0;
}

/* this thread gets the stream from the disk or the network */
static void* read_thread(void *arg)
{
    VideoState *is = arg;
    AVFormatContext *ic = NULL;
    int err, i, ret;
    int st_index[AVMEDIA_TYPE_NB] = { -1, -1, -1, -1, };
    AVPacket pkt1, *pkt = &pkt1;
    int eof = 0;
    int64_t stream_start_time;
    int pkt_in_play_range = 0;
    AVDictionaryEntry *t;
    AVDictionary **opts;
    int orig_nb_streams;
    pthread_mutex_t wait_mutex = PTHREAD_MUTEX_INITIALIZER;

    is->last_video_stream = is->video_stream = -1;
    is->last_audio_stream = is->audio_stream = -1;
    is->last_subtitle_stream = is->subtitle_stream = -1;

    ic = avformat_alloc_context();
    ic->interrupt_callback.callback = decode_interrupt_cb;
    ic->interrupt_callback.opaque = is;

    err = avformat_open_input(&ic, is->filename, is->iformat, &format_opts);
    if (err < 0)
    {
        ret = -1;
        goto fail;
    }

    if ( ( t = av_dict_get(format_opts, "", NULL, AV_DICT_IGNORE_SUFFIX) ) )
    {
        av_log(NULL, AV_LOG_ERROR, "Option %s not found.\n", t->key);
        ret = AVERROR_OPTION_NOT_FOUND;
        goto fail;
    }

    is->ic = ic;

    if ( ffplaymodconf.genpts )
        ic->flags |= AVFMT_FLAG_GENPTS;

    opts = setup_find_stream_info_opts(ic, codec_opts);
    orig_nb_streams = ic->nb_streams;

    err = avformat_find_stream_info(ic, opts);
    if (err < 0)
    {
        ret = -1;
        goto fail;
    }

    for (i = 0; i < orig_nb_streams; i++)
        av_dict_free(&opts[i]);

    av_freep(&opts);

    if (ic->pb)
        ic->pb->eof_reached = 0; // FIXME hack, ffplay maybe should not use url_feof() to test for the end

    if (ffplaymodconf.seek_by_bytes < 0)
    {
        ffplaymodconf.seek_by_bytes = !!(ic->iformat->flags & AVFMT_TS_DISCONT) && strcmp("ogg", ic->iformat->name);
    }

    is->max_frame_duration = (ic->iformat->flags & AVFMT_TS_DISCONT) ? 10.0 : 3600.0;

    if ( !content_title && (t = av_dict_get(ic->metadata, "title", NULL, 0)) )
        content_title = av_asprintf("%s - %s", t->value, input_filename);

    /* if seeking requested, we execute it */
    if (ffplaymodconf.start_time != AV_NOPTS_VALUE)
    {
        int64_t timestamp = ffplaymodconf.start_time;

        /* add the stream start time */
        if (ic->start_time != AV_NOPTS_VALUE)
            timestamp += ic->start_time;

        ret = avformat_seek_file(ic, -1, INT64_MIN, timestamp, INT64_MAX, 0);
        if ( ret < 0 )
        {
            av_log(NULL, AV_LOG_WARNING, "%s: could not seek to position %0.3f\n",
                    is->filename, (double)timestamp / AV_TIME_BASE);
        }
    }

    is->realtime = is_realtime(ic);

    for (i = 0; i < ic->nb_streams; i++)
        ic->streams[i]->discard = AVDISCARD_ALL;

    if ( ffplaymodconf.video_disable == 0 )
        st_index[AVMEDIA_TYPE_VIDEO] = av_find_best_stream(ic, AVMEDIA_TYPE_VIDEO,
                                                           ffplaymodconf.wanted_stream[AVMEDIA_TYPE_VIDEO], -1, NULL, 0);

    if ( ffplaymodconf.audio_disable == 0 )
    {
        st_index[AVMEDIA_TYPE_AUDIO] = av_find_best_stream(ic, AVMEDIA_TYPE_AUDIO,
                                                           ffplaymodconf.wanted_stream[AVMEDIA_TYPE_AUDIO],
                                                           st_index[AVMEDIA_TYPE_VIDEO], NULL, 0);
    }

    if ( ( ffplaymodconf.video_disable == 0 ) && ( ffplaymodconf.subtitle_disable == 0 ) )
        st_index[AVMEDIA_TYPE_SUBTITLE] = av_find_best_stream(ic, AVMEDIA_TYPE_SUBTITLE,
                                                              ffplaymodconf.wanted_stream[AVMEDIA_TYPE_SUBTITLE],
                                                              (st_index[AVMEDIA_TYPE_AUDIO] >= 0 ?
                                                               st_index[AVMEDIA_TYPE_AUDIO] :
                                                               st_index[AVMEDIA_TYPE_VIDEO]),
                                                              NULL, 0);


    if (st_index[AVMEDIA_TYPE_VIDEO] >= 0)
    {
        AVStream *st = ic->streams[st_index[AVMEDIA_TYPE_VIDEO]];
        AVCodecContext *avctx = st->codec;
        VideoPicture vp = {0};

        vp.width = avctx->width;
        vp.height = avctx->height;
        vp.sar = av_guess_sample_aspect_ratio(ic, st, NULL);
    }

    /* open the streams */
    if (st_index[AVMEDIA_TYPE_AUDIO] >= 0)
    {
        stream_component_open(is, st_index[AVMEDIA_TYPE_AUDIO]);
    }

    ret = -1;
    if (st_index[AVMEDIA_TYPE_VIDEO] >= 0)
    {
        ret = stream_component_open(is, st_index[AVMEDIA_TYPE_VIDEO]);
    }

    if (st_index[AVMEDIA_TYPE_SUBTITLE] >= 0)
    {
        stream_component_open(is, st_index[AVMEDIA_TYPE_SUBTITLE]);
    }

    if ( is->video_stream < 0 )
    {
        av_log(NULL, AV_LOG_FATAL, "Failed to open file '%s': video filtergraph\n", is->filename);
        ret = -1;
        goto fail;
    }

    if ( ( ffplaymodconf.audio_disable == 0 ) && ( is->audio_stream < 0 ) )
    {
        av_log(NULL, AV_LOG_ERROR, "Failed to open file '%s': audio filtergraph\n", is->filename);
        //ret = -1;
        //goto fail;
    }

    if (ffplaymodconf.infinite_buffer < 0 && is->realtime)
        ffplaymodconf.infinite_buffer = 1;

    if ( fctrlsig != NULL )
    {
        fctrlsig(fctrlsig_p, FFPLAYMOD_CTRL_PLAY);
    }

    for (;;)
    {
        if (is->abort_request)
            break;

        if (is->paused != is->last_paused)
        {
            is->last_paused = is->paused;
            if (is->paused)
                is->read_pause_return = av_read_pause(ic);
            else
                av_read_play(ic);
        }

#if CONFIG_RTSP_DEMUXER || CONFIG_MMSH_PROTOCOL
        if (is->paused &&
                (!strcmp(ic->iformat->name, "rtsp") ||
                 (ic->pb && !strncmp(input_filename, "mmsh:", 5))))
        {
            /* wait 10 ms to avoid trying to get another packet */
            /* XXX: horrible */
            Sleep(10);
            continue;
        }
#endif
        if (is->seek_req)
        {
            int64_t seek_target = is->seek_pos;
            int64_t seek_min    = is->seek_rel > 0 ? seek_target - is->seek_rel + 2: INT64_MIN;
            int64_t seek_max    = is->seek_rel < 0 ? seek_target - is->seek_rel - 2: INT64_MAX;
// FIXME the +-2 is due to rounding being not done in the correct direction in generation
//      of the seek_pos/seek_rel variables

            ret = avformat_seek_file(is->ic, -1, seek_min, seek_target, seek_max, is->seek_flags);

            if (ret < 0)
            {
                av_log(NULL, AV_LOG_ERROR,
                       "%s: error while seeking\n", is->ic->filename);
            }
            else
            {
                if (is->audio_stream >= 0)
                {
                    packet_queue_flush(&is->audioq);
                    packet_queue_put(&is->audioq, &flush_pkt);
                }

                if (is->subtitle_stream >= 0)
                {
                    packet_queue_flush(&is->subtitleq);
                    packet_queue_put(&is->subtitleq, &flush_pkt);
                }

                if (is->video_stream >= 0)
                {
                    packet_queue_flush(&is->videoq);
                    packet_queue_put(&is->videoq, &flush_pkt);
                }

                if (is->seek_flags & AVSEEK_FLAG_BYTE)
                {
                   set_clock(&is->extclk, NAN, 0);
                }
                else
                {
                   set_clock(&is->extclk, seek_target / (double)AV_TIME_BASE, 0);
                }
            }

            is->seek_req = 0;
            is->queue_attachments_req = 1;
            eof = 0;

            if (is->paused)
                step_to_next_frame(is);
        }

        if (is->queue_attachments_req)
        {
            if (is->video_st && is->video_st->disposition & AV_DISPOSITION_ATTACHED_PIC)
            {
                AVPacket copy;
                if ((ret = av_copy_packet(&copy, &is->video_st->attached_pic)) < 0)
                    goto fail;
                packet_queue_put(&is->videoq, &copy);
                packet_queue_put_nullpacket(&is->videoq, is->video_stream);
            }
            is->queue_attachments_req = 0;
        }

        /* if the queue are full, no need to read more */
        if (ffplaymodconf.infinite_buffer<1 &&
              (is->audioq.size + is->videoq.size + is->subtitleq.size > MAX_QUEUE_SIZE
            || (   (is->audioq   .nb_packets > MIN_FRAMES || is->audio_stream < 0 || is->audioq.abort_request)
                && (is->videoq   .nb_packets > MIN_FRAMES || is->video_stream < 0 || is->videoq.abort_request
                    || (is->video_st->disposition & AV_DISPOSITION_ATTACHED_PIC))
                && (is->subtitleq.nb_packets > MIN_FRAMES || is->subtitle_stream < 0 || is->subtitleq.abort_request))))
        {
            /* wait 10 ms */
            pthread_mutex_lock(&wait_mutex);
            pthread_cond_timedwait(&is->continue_read_thread, &wait_mutex, 10);
            pthread_mutex_unlock(&wait_mutex);
            continue;
        }

        if (!is->paused &&
            (!is->audio_st || is->audio_finished == is->audioq.serial) &&
            (!is->video_st || (is->video_finished == is->videoq.serial && is->pictq_size == 0)))
        {
            if (ffplaymodconf.loop != 1 && (!ffplaymodconf.loop || --ffplaymodconf.loop))
            {
                stream_seek(is, ffplaymodconf.start_time != AV_NOPTS_VALUE ? ffplaymodconf.start_time : 0, 0, 0);
            }
            else
            if (ffplaymodconf.autoexit)
            {
                ret = AVERROR_EOF;
                goto fail;
            }
        }

        if (eof)
        {
            if (is->video_stream >= 0)
                packet_queue_put_nullpacket(&is->videoq, is->video_stream);

            if (is->audio_stream >= 0)
                packet_queue_put_nullpacket(&is->audioq, is->audio_stream);

            if (is->subtitle_stream >= 0)
                packet_queue_put_nullpacket(&is->subtitleq, is->subtitle_stream);

            Sleep(10);
            eof=0;

            continue;
        }

        ret = av_read_frame(ic, pkt);
        if (ret < 0)
        {
            if (ret == AVERROR_EOF || url_feof(ic->pb))
                eof = 1;

            if (ic->pb && ic->pb->error)
                break;

            pthread_mutex_lock(&wait_mutex);
            pthread_cond_timedwait(&is->continue_read_thread, &wait_mutex, 10);
            pthread_mutex_unlock(&wait_mutex);
            continue;
        }

        /* check if packet is in play range specified by user, then queue, otherwise discard */
        stream_start_time = ic->streams[pkt->stream_index]->start_time;
        pkt_in_play_range = ffplaymodconf.duration == AV_NOPTS_VALUE ||
                (pkt->pts - (stream_start_time != AV_NOPTS_VALUE ? stream_start_time : 0)) *
                av_q2d(ic->streams[pkt->stream_index]->time_base) -
                (double)(ffplaymodconf.start_time != AV_NOPTS_VALUE ? ffplaymodconf.start_time : 0) / 1000000
                <= ((double)ffplaymodconf.duration / 1000000);

        if (pkt->stream_index == is->audio_stream && pkt_in_play_range)
        {
            packet_queue_put(&is->audioq, pkt);
        }
        else
        if (pkt->stream_index == is->video_stream && pkt_in_play_range
            && !(is->video_st->disposition & AV_DISPOSITION_ATTACHED_PIC))
        {
            packet_queue_put(&is->videoq, pkt);
        }
        else
        if (pkt->stream_index == is->subtitle_stream && pkt_in_play_range)
        {
            packet_queue_put(&is->subtitleq, pkt);
        }
        else
        {
            av_free_packet(pkt);
        }
    }

    /* wait until the end */
    while (!is->abort_request)
    {
        Sleep(10);
    }

    ret = 0;

 fail:
    /* close each stream */
    if (is->audio_stream >= 0)
        stream_component_close(is, is->audio_stream);

    if (is->video_stream >= 0)
        stream_component_close(is, is->video_stream);

    if (is->subtitle_stream >= 0)
        stream_component_close(is, is->subtitle_stream);

    if (is->ic)
    {
        avformat_close_input(&is->ic);
    }

    pthread_mutex_destroy(&wait_mutex);

    return NULL;
}

static VideoState* stream_open(const char *filename, AVInputFormat *iformat)
{
    VideoState *is;

    is = av_mallocz(sizeof(VideoState));

    if (!is)
        return NULL;

    av_strlcpy(is->filename, filename, sizeof(is->filename));
    is->iformat = iformat;
    is->ytop    = 0;
    is->xleft   = 0;

    /* start video display */
    is->pictq_mutex = PTHREAD_MUTEX_INITIALIZER;
    is->pictq_cond  = PTHREAD_COND_INITIALIZER;

    is->subpq_mutex = PTHREAD_MUTEX_INITIALIZER;
    is->subpq_cond  = PTHREAD_COND_INITIALIZER;

    packet_queue_init(&is->videoq);
    packet_queue_init(&is->audioq);
    packet_queue_init(&is->subtitleq);

    is->continue_read_thread = PTHREAD_COND_INITIALIZER;

    init_clock(&is->vidclk, &is->videoq.serial);
    init_clock(&is->audclk, &is->audioq.serial);
    init_clock(&is->extclk, &is->extclk.serial);

    is->audio_clock_serial = -1;
    is->audio_last_serial = -1;
    is->av_sync_type = ffplaymodconf.av_sync_type;

    if ( pthread_create( &is->read_tid, NULL, read_thread, (void*)is ) != 0 )
    {
        av_free(is);
        return NULL;
    }

    if ( fctrlsig != NULL )
    {
        fctrlsig(fctrlsig_p, FFPLAYMOD_CTRL_LOAD);
    }

    return is;
}

static void stream_cycle_channel(VideoState *is, int codec_type)
{
    AVFormatContext *ic = is->ic;
    int start_index, stream_index;
    int old_index;
    AVStream *st;
    AVProgram *p = NULL;
    int nb_streams = is->ic->nb_streams;

    if (codec_type == AVMEDIA_TYPE_VIDEO)
    {
        start_index = is->last_video_stream;
        old_index = is->video_stream;
    }
    else
    if (codec_type == AVMEDIA_TYPE_AUDIO)
    {
        start_index = is->last_audio_stream;
        old_index = is->audio_stream;
    }
    else
    {
        start_index = is->last_subtitle_stream;
        old_index = is->subtitle_stream;
    }

    stream_index = start_index;

    if (codec_type != AVMEDIA_TYPE_VIDEO && is->video_stream != -1)
    {
        p = av_find_program_from_stream(ic, NULL, is->video_stream);

        if (p)
        {
            nb_streams = p->nb_stream_indexes;

            for (start_index = 0; start_index < nb_streams; start_index++)
                if (p->stream_index[start_index] == stream_index)
                    break;

            if (start_index == nb_streams)
                start_index = -1;

            stream_index = start_index;
        }
    }

    for (;;)
    {
        if (++stream_index >= nb_streams)
        {
            if (codec_type == AVMEDIA_TYPE_SUBTITLE)
            {
                stream_index = -1;
                is->last_subtitle_stream = -1;
                goto the_end;
            }

            if (start_index == -1)
                return;

            stream_index = 0;
        }

        if (stream_index == start_index)
            return;

        st = is->ic->streams[p ? p->stream_index[stream_index] : stream_index];

        if (st->codec->codec_type == codec_type)
        {
            /* check that parameters are OK */
            switch (codec_type)
            {
                case AVMEDIA_TYPE_AUDIO:
                    if ( st->codec->sample_rate != 0 && st->codec->channels != 0 )
                        goto the_end;
                    break;

                case AVMEDIA_TYPE_VIDEO:
                case AVMEDIA_TYPE_SUBTITLE:
                    goto the_end;

                default:
                    break;
            }
        }
    }
 the_end:
    if (p && stream_index != -1)
        stream_index = p->stream_index[stream_index];

    stream_component_close(is, old_index);
    stream_component_open(is, stream_index);
}

static void seek_chapter(VideoState *is, int incr)
{
    int64_t pos = get_master_clock(is) * AV_TIME_BASE;
    int i;

    if (!is->ic->nb_chapters)
        return;

    /* find the current chapter */
    for (i = 0; i < is->ic->nb_chapters; i++)
    {
        AVChapter *ch = is->ic->chapters[i];
        if (av_compare_ts(pos, AV_TIME_BASE_Q, ch->start, ch->time_base) < 0)
        {
            i--;
            break;
        }
    }

    i += incr;
    i = FFMAX(i, 0);
    if (i >= is->ic->nb_chapters)
        return;

    av_log(NULL, AV_LOG_VERBOSE, "Seeking to chapter %d.\n", i);
    stream_seek(is, av_rescale_q(is->ic->chapters[i]->start, is->ic->chapters[i]->time_base,
                                 AV_TIME_BASE_Q), 0, 0);
}

static int lockmgr(void **mtx, enum AVLockOp op)
{
   switch(op)
   {
      case AV_LOCK_CREATE:
          *mtx = PTHREAD_MUTEX_INITIALIZER;
          if(!*mtx)
              return 1;
          return 0;

      case AV_LOCK_OBTAIN:
          return !!pthread_mutex_lock(&*mtx);

      case AV_LOCK_RELEASE:
          return !!pthread_mutex_unlock(&*mtx);

      case AV_LOCK_DESTROY:
          pthread_mutex_destroy(&*mtx);
          return 0;
   }
   return 1;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

int ffplaymod_init( int argc, char** argv )
{
    reconfigure_ffplaymod_conf( &ffplaymodconf );

    av_log_set_flags(AV_LOG_SKIP_REPEATED);

    /* register all codecs, demux and protocols */
#if CONFIG_AVDEVICE
    avdevice_register_all();
#endif
    avfilter_register_all();
    av_register_all();
    avformat_network_init();

    init_opts();

    signal(SIGINT , sigterm_handler); /* Interrupt (ANSI).    */
    signal(SIGTERM, sigterm_handler); /* Termination (ANSI).  */

    if (av_lockmgr_register(lockmgr))
    {
        av_log(NULL, AV_LOG_FATAL, "Could not initialize lock manager!\n");
        do_stopall(NULL);
        return -1;
    }

    av_init_packet(&flush_pkt);
    flush_pkt.data = (uint8_t *)&flush_pkt;

    return 0;
}

int ffplaymod_final()
{
    if ( input_stream != NULL )
    {
        do_stopall( input_stream );
    }

    av_lockmgr_register(NULL);

    uninit_opts();

    av_freep(&vfilters);

    avformat_network_deinit();
}

int ffplaymod_open( const char* uri )
{
    input_stream = stream_open( uri, file_iformat );
    if ( input_stream == NULL )
    {
        av_log(NULL, AV_LOG_FATAL, "Failed to initialize VideoState!\n");
        do_stopall(NULL);
        return -1;
    }

    input_filename = uri;

    return 0;
}

int ffplaymod_close()
{
    if  ( input_stream != NULL )
    {
        do_stopall( input_stream );
        return 0;
    }

    return -1;
}


int ffplaymod_playcontrol( int controltype )
{
    double incr, pos, frac;

    switch ( controltype )
    {
        case 1: // PAUSE-RESUME
            toggle_pause(input_stream);
            return 0;

        case 2:  // REWIND 10sec.
            incr = -10.0;
            break;

        case 3: // FF 10sec;
            incr = 10.0;
            break;

        case 4: // next chapter;
            if (input_stream->ic->nb_chapters <= 1)
            {
                incr = 600.0;
            }
            else
            {
                seek_chapter(input_stream, 1);
            }
            break;

        case 5:
            if (input_stream->ic->nb_chapters <= 1)
            {
                incr = -600.0;
            }
            else
            {
                seek_chapter(input_stream, -1);
            }
            break;

        case 0:
        default:
            return -1;
    }


    if ( ffplaymodconf.seek_by_bytes > 0 )
    {
        if (input_stream->video_stream >= 0 && input_stream->video_current_pos >= 0)
        {
            pos = input_stream->video_current_pos;
        }
        else
        if (input_stream->audio_stream >= 0 && input_stream->audio_pkt.pos >= 0)
        {
            pos = input_stream->audio_pkt.pos;
        }
        else
        {
            pos = avio_tell(input_stream->ic->pb);
        }

        if (input_stream->ic->bit_rate)
            incr *= input_stream->ic->bit_rate / 8.0;
        else
            incr *= 180000.0;

        pos += incr;
        stream_seek(input_stream, pos, incr, 1);
    }
    else
    {
        pos = get_master_clock(input_stream);

        if (isnan(pos))
            pos = (double)input_stream->seek_pos / AV_TIME_BASE;

        pos += incr;

        if (input_stream->ic->start_time != AV_NOPTS_VALUE && pos < input_stream->ic->start_time / (double)AV_TIME_BASE)
            pos = input_stream->ic->start_time / (double)AV_TIME_BASE;

        stream_seek(input_stream, (int64_t)(pos * AV_TIME_BASE), (int64_t)(incr * AV_TIME_BASE), 0);
    }

}

int ffplaymod_pause_state()
{
    if ( input_stream != NULL )
    {
        return input_stream->paused;
    }

    return -1;
}

unsigned int ffplaymod_getvideobuffer( char** buffer )
{
    if ( input_stream == NULL )
    {
        return 0;
    }

    if ( input_stream->video_stream < 0 )
    {
        return 0;
    }

    render_surface( input_stream );

    VideoPicture* cur_vp = &input_stream->pictq[input_stream->pictq_rindex];

    if ( cur_vp != NULL )
    {
        bool gotit = false;

        while( gotit == false )
        {
            if ( ( cur_vp->ready == 0 ) && ( cur_vp->bmpsize == 0 ) )
            {
                av_usleep(100000.0);
            }
            else
            {
                gotit = true;
            }
        }

        static double prev_pts = 0.0;

        if ( prev_pts > 0.0 )
        {
            if ( prev_pts >= cur_vp->pts )
            {
                return 0;
            }
        }

        *buffer = new unsigned char[ cur_vp->bmpsize ];

        if ( *buffer != NULL )
        {
            int retsize = cur_vp->bmpsize;

            if ( retsize > 0 )
            {
                memcpy( *buffer, cur_vp->bmp, cur_vp->bmpsize );

                prev_pts = cur_vp->pts;
            }

            pthread_mutex_lock( &input_stream->pictq_mutex );
            pthread_cond_signal( &input_stream->pictq_cond );
            pthread_mutex_unlock( &input_stream->pictq_mutex );

            return retsize;
        }

    }

    return 0;
}

// *Notice*
// It must be called in thread type.
unsigned int ffplaymod_getaudiobuffer( char** buffer, unsigned* dur_ms )
{
#ifdef FAST_AUDIO
    if ( input_stream == NULL )
        return 0;

    VideoState* is = input_stream;

    int audio_size = audio_decode_frame(is);

   if (audio_size == 0)
   {
       return 0;
   }
   else
   if (audio_size < 0)
   {
        /* if error, just output silence */
       is->audio_buf      = is->silence_buf;
       is->audio_buf_size = sizeof(is->silence_buf) / is->audio_tgt.frame_size * is->audio_tgt.frame_size;
   }
   else
   {
       is->audio_buf_size = audio_size;
   }

   is->audio_buf_index = 0;

   *buffer = new char[ audio_size ];

   if ( buffer != NULL )
   {
        memcpy( *buffer, (uint8_t *)is->audio_buf, audio_size );
   }

   return audio_size;

#else
    if ( input_stream == NULL )
        return 0;

    VideoState* is = input_stream;

    int len = audio_temp_size;
    int audio_size = 0;
    int len1;
    int retlen = 0;

    audio_callback_time = av_gettime();

    *buffer = new char[ len ];

    while (len > 0)
    {
        if (is->audio_buf_index >= is->audio_buf_size)
        {
           audio_size = audio_decode_frame(is);
           if (audio_size == 0)
           {
               return 0;
           }
           else
           if (audio_size < 0)
           {
                /* if error, just output silence */
                is->audio_buf      = is->silence_buf;
                is->audio_buf_size = sizeof(is->silence_buf) / is->audio_tgt.frame_size * is->audio_tgt.frame_size;
           }
           else
           {
               is->audio_buf_size = audio_size;
           }

           is->audio_buf_index = 0;
        }

        len1 = is->audio_buf_size - is->audio_buf_index;

        if (len1 > len)
            len1 = len;


        if ( *buffer != NULL )
        {
            memcpy( *buffer, (uint8_t *)is->audio_buf + is->audio_buf_index, len1 );
            retlen = len1;

            if ( dur_ms != NULL )
            {
                *dur_ms = ( av_gettime() / 10000.0 ) - ( audio_callback_time / 10000.0 );
            }
        }

        len -= len1;
        *buffer + len1;
        is->audio_buf_index += len1;
    }

    is->audio_write_buf_size = is->audio_buf_size - is->audio_buf_index;

    if (!isnan(is->audio_clock))
    {
        set_clock_at(&is->audclk, is->audio_clock - (double)(2 * is->audio_hw_buf_size + is->audio_write_buf_size) / is->audio_tgt.bytes_per_sec, is->audio_clock_serial, audio_callback_time / 1000000.0);
        sync_clock_to_slave(&is->extclk, &is->audclk);
    }

    return retlen;
#endif
}

VideoState* ffplaymod_getVideoState()
{
    return input_stream;
}

void ffplaymod_set_controlsig( ffplaymod_controlsig fcs, void* param )
{
    fctrlsig = fcs;
    fctrlsig_p = param;
}

const char* ffplaymod_get_lasterrstr()
{
    return err_strs;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

/* Called from the main */
int main(int argc, char **argv)
{
    int retVal = 0;

    aMain = new appMain( argc, argv );
    if ( aMain != NULL )
    {
        ffplaymod_set_controlsig( appMain::recv_controlsig , aMain );

        Fl_Double_Window* aWin = aMain->getwindow();

        // Check FLTK window created !
        if ( aWin != NULL )
        {
            // And run !
            retVal = aMain->Run();
        }
    }

    return retVal;
}
