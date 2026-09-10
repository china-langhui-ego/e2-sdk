/* SPDX-License-Identifier: Apache-2.0 */
/* libsxr_recorder internal — record session save-half.
 *
 * Relocated from mi_pipeline.{h,cpp} (Plan 2b Task 4): the SxrRecord sink,
 * per-pipe RecPipeline (write_q + writer thread + current_record), the
 * write_cb state machine (runs on HAL main-grab thread) and the async writer.
 *
 * MI-FREE: depends only on sxr_hal.h (config types) + sxr_recorder.h
 * (sxr_fmp4_t / sxr_sensors_t typedefs + record config types). The fMP4/sensors
 * _impl entry points mi_record.cpp calls are declared locally there (extern "C"). */
#ifndef _MI_RECORD_H_
#define _MI_RECORD_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <pthread.h>
#include <stdio.h>
#include "sxr_recorder.h"   /* sxr_fmp4_t, sxr_sensors_t (internal opaque handles) +
                             * sxr_rec_evt_t — must precede sxr_hal.h so the latter
                             * aliases SxrRecordEvent to it instead of redefining. */
#include "sxr_hal.h"        /* SXR_RECORDER_SNR_MAX, SxrRecordConfig/Stats,
                             * sxr_record_t, sxr_record_event_cb_t, SxrAudioConfig */

#ifdef __cplusplus
extern "C" {
#endif

/* ---- record-segment state machine ---- */
enum { REC_PENDING_IDR = 0, REC_WRITING = 1, REC_STOPPING = 2 };

/* Async write queue capacity (~6 seconds of 30fps). Moved here from Pipeline. */
#define RECORD_WRITE_Q_CAP 180

/* Audio async write queue: one entry per PCM period (1024 frames = 64ms
 * @16kHz, 4KB S16_LE stereo). 256 entries ≈ 16s of audio, ~1MB RAM. The MI_AI
 * driver ring is only a few hundred ms deep — file I/O must NOT run on the
 * capture thread (10:20:04 xrun: "Buffer(s) is lost due to slow fetching",
 * 3 chunks/192ms of audio lost when a WiFi/statvfs SD stall blocked the
 * inline FMP4Writer flush). */
#define AUDIO_WRITE_Q_CAP 256

/* One queued PCM chunk. data ownership: audio_cb (capture thread) -> audio
 * writer thread (frees it). */
struct AudioWriteEntry {
    uint8_t *data;
    size_t   size;
    int64_t  pts_us;
    int64_t  mono_ns;
};

/* One entry in the async writer queue. sampleBuf ownership transfers from
 * write_cb (HAL main-grab thread) -> writer thread (which frees it). */
struct RecordWriteEntry {
    uint8_t *data;
    size_t   size;
    int64_t  pts_us;
    int      is_sync;
    int      sensor_ch;
    /* metainfo fields (mp4 sample linked to metainfo row by pts_us) */
    int64_t  exposure_start_ns;
    int64_t  exposure_duration_ns;
    uint32_t gain;
};

/* Record segment (short-lived sink set): one record_start->record_stop maps
 * to one independent directory. Moved from mi_pipeline.h. */
struct SxrRecord {
    int       pipe_id;
    sxr_sensors_t sensors;            /* may be NULL */
    char      dir[512];

    struct {
        sxr_fmp4_t fmp4;              /* video.mp4, NULL=not opened */
        FILE     *metainfo;          /* metainfo.csv, NULL=not opened */
        bool      idr_captured;      /* track opened for this eye? */
        int64_t   start_expo_ns;     /* exposure_start of the chosen start IDR (stereo-aligned) */
    } cam[SXR_RECORDER_SNR_MAX];

    /* Stereo start-frame alignment (v3): record_start forces an IDR per eye
     * independently; the two forced IDRs can land on different sensor frames
     * (up to one 33ms period apart), which used to make Camera0/Camera1
     * metainfo row i refer to different physical frames (k=±1 slip).
     *
     * Fix: during REC_PENDING_IDR write_cb buffers each eye's IDR candidates
     * (one entry per (sensor_ch, exposure_start_ns) is_sync frame) instead of
     * committing immediately. Once every enabled eye has >=1 candidate, pick
     * the common start frame = max over eyes of each eye's earliest candidate
     * (latest common IDR — every eye can reach it). Each eye then commits on
     * its own candidate with exposure_start == chosen (drops earlier ones),
     * so both metainfo.csv row 0 and the first mp4 sample are the SAME
     * physical exposure. PENDING cap bounds the wait; IDR-timeout watchdog
     * unchanged. */
    struct {
        uint8_t *data;
        size_t   size;
        int64_t  pts_us;
        int64_t  exposure_start_ns;
        int64_t  exposure_duration_ns;
        uint32_t gain;
        uint8_t  csd[256];
        size_t   csd_size;
        int      used;               /* slot occupied */
    } pending_idr[SXR_RECORDER_SNR_MAX][4];  /* [eye][slot], cap 4 IDRs per eye */
    int      pending_idr_count[SXR_RECORDER_SNR_MAX];
    int64_t  aligned_start_expo_ns;   /* chosen common start exposure (0 = not chosen yet) */

    struct {
        sxr_fmp4_t fmp4;        /* Audio/audio.mp4, NULL=not opened */
        FILE      *metainfo;    /* Audio/metainfo.csv */
        void      *hal_handle;  /* sxr_audio_start() return */
        int        channels;    /* copied from audio_cfg for metainfo frames calc */
        int        sample_rate; /* copied from audio_cfg (track timescale) */
        uint64_t frames_written;/* cumulative PCM frames -> fmp4 pts (track timescale = sample rate) */
        int64_t  base_frames;   /* 首块 PCM 的 pts_us 换算成采样帧（与视频同一 MI_SYS 时钟域） */
        bool     base_set;      /* base_frames 是否已初始化 */
        bool       opened;      /* audio sink ready (false on failure/absent mic) */

        /* async write queue (音频采集线程只做 memcpy 入队,SD I/O 在 writer 线程) */
        struct AudioWriteEntry q[AUDIO_WRITE_Q_CAP];
        uint32_t  q_wpos, q_rpos, q_count, q_dropped;
        pthread_mutex_t q_mutex;
        pthread_cond_t  q_cond;
        bool      q_exit;
        pthread_t q_thread;
        bool      q_running;
    } audio;

    SxrRecordConfig       cfg;
    SxrRecordStats        stats;
    sxr_record_event_cb_t evt_cb;
    void                *userdata;

    volatile int state;              /* REC_PENDING_IDR / REC_WRITING / REC_STOPPING */
    int          error;              /* 0 / ETIMEDOUT / EIO */
    int          enabled_mask;       /* bit0=cam0, bit1=cam1 */
    int64_t      pending_deadline_ns;/* IDR-timeout watchdog cutoff */

    /* Periodic sensor_time(MONOTONIC_RAW)->UTC(CLOCK_REALTIME) mapping sampler.
     * One row written at record_start (header + first sample), then the sampler
     * thread appends a row every SXR_UTC_MAP_PERIOD_US so a mid-record UTC
     * calibration (NTP slew/step) is captured. Output: <dir>/sensor_time_to_utc.csv.
     * Joined in record_stop; rollback only fcloses (the thread is post-publish
     * only, so rollback never observes a live sampler). */
    FILE         *utc_fp;            /* sensor_time_to_utc.csv, NULL=not opened */
    pthread_t     utc_thread;
    volatile int  utc_thread_running;/* 1 between pthread_create and join */
    volatile int  utc_stop;          /* sampler exit signal */
};

/* Per-pipe recorder state (replaces the write_q fields removed from HAL's
 * Pipeline struct). One slot per pipe_id; populated by create_venc
 * recorder-wrapper (camera_mask/width/height/pts_offset). */
#define MAX_REC_PIPES 4
struct RecPipeline {
    bool             in_use;
    struct SxrRecord *current_record;          /* atomic — the active record sink */

    /* async write queue */
    struct RecordWriteEntry write_q_entries[RECORD_WRITE_Q_CAP];
    uint32_t        wpos;
    uint32_t        rpos;
    uint32_t        count;
    uint32_t        dropped;
    pthread_mutex_t write_q_mutex;
    pthread_cond_t  write_q_cond;
    bool            write_q_exit;
    bool            write_q_busy;             /* writer is mid-I/O on a popped entry */
    pthread_t       writer_thread;
    bool            writer_running;

    /* per-camera dims for sxr_fmp4_set_video_track (populated by create_venc
     * recorder-wrapper from the HAL's [OUTPUT] detection results). */
    int             camera_mask;    /* bit0=Camera0, bit1=Camera1 (HAL 自动探测) */
    int             width[SXR_RECORDER_SNR_MAX];
    int             height[SXR_RECORDER_SNR_MAX];
    int64_t         pts_offset_ns;
    SxrAudioConfig  audio_cfg;   /* copied from SxrPipelineConfig.audio at create_venc */
};

/* RecPipeline slot management (called by Task 5's create_venc/destroy wrappers). */
struct RecPipeline *recpipeline_alloc(int pipe_id);
struct RecPipeline *recpipeline_get(int pipe_id);
void                recpipeline_free(int pipe_id);

/* ---- record session: _impl (called from sxr_recorder.c) ---- */
sxr_record_t sxr_record_start_impl(int pipe_id, sxr_sensors_t sensors,
                                  const char *dir,
                                  const SxrRecordConfig *cfg,
                                  sxr_record_event_cb_t evt_cb, void *userdata);
int sxr_record_stop_impl(sxr_record_t rec, SxrRecordStats *stats);

#ifdef __cplusplus
}
#endif

#endif /* _MI_RECORD_H_ */
