/* SPDX-License-Identifier: Apache-2.0 */
/* libsxr_hal — SXR Hardware Abstraction Layer (dynamically loaded).
 * Single exported symbol sxr_hal_get_api() fills a versioned vtable. */
#ifndef _SXR_HAL_H_
#define _SXR_HAL_H_

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SXR_HAL_API_VERSION 3

/* ---- Calibration structs ---- */
typedef struct {
    int    valid;
    int    eye;                 /* 0=left, 1=right */
    char   eye_name[16];        /* "left"/"right" */
    int    width, height;       /* native sensor size (1600x1200), unscaled */
    float  focal_x, focal_y;
    float  center_x, center_y;
    float  radial_distortion[8];/* [0..5] from XML, [6..7]=0 (tangential) */
    float  position[3];         /* SVR extrinsic translation */
    float  rotation[4];         /* SVR extrinsic quaternion (x,y,z,w) */
    int    is_rolling_shutter;
} sxr_hal_camera_calib_t;

typedef struct { char name[32]; float delta_sec; } sxr_hal_cam_time_align_t;

typedef struct {
    int    valid;
    int    imu_id;
    int    is_primary;
    float  accel_bias[3];       /* aBias */
    float  gyro_bias[3];        /* wBias */
    float  accel_scale[3];      /* ka */
    float  gyro_scale[3];       /* kg */
    float  accel_nonorth[3];    /* na */
    float  gyro_nonorth[3];     /* ng */
    float  imu_to_pose_delta;   /* <Stateinit delta> */
    float  accel_delta;         /* <Stateinit accelDelta> */
    sxr_hal_cam_time_align_t cam_time_align[8];
    int    cam_time_align_count;
    float  accel_noise_std[3];  /* <IMUNoise stationaryAccelNoise> or 0.02 */
    float  gyro_noise_std[3];   /* <IMUNoise stationaryGyroNoise>   or 0.0016 */
    float  accel_bias_std[3];   /* default 0.05 */
    float  gyro_bias_std[3];    /* default 0.005 */
} sxr_hal_imu_calib_t;

/* ---- Callbacks (sub-stream frame; main-frame + sensor used in Plan 2) ---- */
typedef void (*sxr_hal_frame_cb_t)(int pipe_id, int ch,
    const uint8_t *data, size_t size, int64_t pts_us, int is_sync,
    const uint8_t *csd, size_t csd_size, void *userdata);

/* Sensor sample delivered by the HAL IIO/mag threads to the recorder (for CSV). */
typedef enum {
    SXR_HAL_SAMPLE_ACCEL = 0,
    SXR_HAL_SAMPLE_GYRO,
    SXR_HAL_SAMPLE_MAG,
} sxr_hal_sample_type_t;

/* @param ud      opaque pointer from sensors_set_sample_cb
 * @param type    which sensor
 * @param ts_ns   sample timestamp in CLOCK_MONOTONIC_RAW domain (driver ts; raw passthrough, no REALTIME anchor)
 * @param vals    [3] calibrated body-frame value (m/s^2, rad/s, or uT) */
typedef void (*sxr_hal_sensor_cb_t)(void *ud, sxr_hal_sample_type_t type,
                                     int64_t ts_ns, const double vals[3]);

/* Main-stream encoded frame delivered to the recorder (carries AE for metainfo).
 * Runs on the HAL main-grab thread — MUST return quickly (enqueue; do slow fMP4
 * I/O on a recorder-owned thread). sampleBuf ownership transfers to the callback
 * (it frees it). HAL has already called MI_VENC_ReleaseStream before invoking this. */
typedef void (*sxr_hal_main_frame_cb_t)(void *sink, int pipe_id, int ch,
    uint8_t *data, size_t size, int64_t pts_us, int is_sync,
    uint8_t *csd, size_t csd_size,
    uint32_t expo_us, uint32_t gain, uint64_t drv_frame_seq,
    int64_t exposure_start_ns, int64_t exposure_duration_ns);

/* Audio PCM chunk delivered by the HAL MI_AI capture thread to the recorder.
 * Runs on the HAL audio thread — MUST return quickly (FMP4Writer buffers in memory).
 * @param ud          opaque pointer from audio_start
 * @param pcm         interleaved S16_LE samples (one MI_AI period)
 * @param bytes       pcm length in bytes
 * @param pts_us      MI_AI_Data_t.u64Pts (MI_SYS PTS, same clock as video)
 * @param mono_ns    pts_us*1000 + cfg.pts_offset_ns (CLOCK_MONOTONIC_RAW domain) */
typedef void (*sxr_hal_audio_cb_t)(void *ud, const uint8_t *pcm, size_t bytes,
                                    int64_t pts_us, int64_t mono_ns);

/* ---- Pipeline / Record Session config types (owned by HAL; relocated from
 *      libsxr_recorder/include/sxr_recorder.h so sxr_pipeline_core.cpp can consume
 *      them directly. All plain C — no MI headers required.) ---- */

#define SXR_RECORDER_SNR_MAX 2

/* Audio (dual AMIC) capture config. PCM S16LE only — the SDK has no audio encoder. */
typedef struct SxrAudioConfig {
    int     enable;          /* 1=record audio, 0=skip */
    int     sample_rate;     /* 8000/16000/32000/48000, default 16000 */
    int     channels;        /* 2 = stereo (MIC0=L, MIC1=R) */
    int     bits_per_sample; /* fixed 16 (S16_LE) */
    int     if_gain_l;       /* MIC0 analog IF gain, default 18 */
    int     if_gain_r;       /* MIC1 analog IF gain, default 18 */
    double  dpga_gain_db;    /* digital gain dB [-63.5,64.0], default 0 */
    int64_t pts_offset_ns;   /* filled by recorder at record_start (shared video offset) */
} SxrAudioConfig;

typedef struct SxrPipelineConfig {
    /* Clock frequencies in Hz (default: ISP=216000000, SCL=288000000) */
    int   isp_clk;
    int   scl_clk;

    /* VENC parameters (VENC pipeline only) */
    int   venc_gop;                            /* default: 30 */
    int   venc_bitrate_bps;                    /* default: 4000000 (4 Mbps) */

    /* Preview sub-stream (SCL port1 -> VENC preview channels, low-res HEVC) */
    int   enable_preview;          /* 0=off (default), 1=on */
    int   preview_width;           /* 0=auto -> main_width/4, ALIGN_UP(2) */
    int   preview_height;          /* 0=auto -> main_height/4 */
    int   preview_bitrate_bps;     /* 0=auto -> 2000000 (2 Mbps) */
    int   preview_fps;             /* 0=auto -> 15 */

    /* Exit flag: set to non-zero to stop all threads */
    volatile unsigned char *exit_flag;

    /* [OUTPUT] Filled by sxr_pipeline_create_venc: HAL 自动探测 pad0/pad2 上
     * 挂载的 sensor 并按型号决定主码流分辨率 / IQ bin, 回填下列字段。 */
    int   camera_mask;                    /* bit0=Camera0, bit1=Camera1; 0=无 sensor */
    int   width[SXR_RECORDER_SNR_MAX];    /* 各 camera 主码流宽 (detected) */
    int   height[SXR_RECORDER_SNR_MAX];   /* 各 camera 主码流高 (detected) */
    int64_t pts_offset_ns;                /* [OUTPUT] PTS -> CLOCK_MONOTONIC_RAW offset in ns */

    SxrAudioConfig audio;
} SxrPipelineConfig;

typedef struct SxrRecordConfig {
    size_t prealloc_bytes;      /* FMP4 prealloc bytes, 0=none (default) */
    int    sample_interval_us;  /* moov backpatch sample interval, default 33333 (~30fps) */
} SxrRecordConfig;

typedef struct SxrRecordStats {
    int64_t  start_realtime_ns;            /* first-frame IDR MONOTONIC_RAW capture (duration baseline; legacy name kept for stats ABI) */
    int64_t  duration_ns;                  /* first -> last frame */
    int64_t  pts_offset_ns;                /* for sensors alignment */
    uint64_t frames[SXR_RECORDER_SNR_MAX];  /* per-eye written frame count */
    uint64_t bytes [SXR_RECORDER_SNR_MAX];  /* per-eye video.mp4 byte count */
    uint32_t dropped_frames;               /* drain/backpressure drops (should be 0) */
    int      error;                        /* 0=ok; nonzero=mid-write failure */
} SxrRecordStats;

/* Event enum: v2 (sxr_recorder.h) is the canonical source. If it was already
 * included, alias SxrRecordEvent to its sxr_rec_evt_t so both names refer to
 * the same enum (identical values 0/1/2). Otherwise define locally. */
#ifdef _SXR_RECORDER_H_
typedef sxr_rec_evt_t SxrRecordEvent;
#else
typedef enum {
    SXR_REC_EVT_STARTED,       /* all eyes' first IDR written, recording underway */
    SXR_REC_EVT_IDR_TIMEOUT,   /* forced IDR timed out (default 1s) -> auto-finalize, no data */
    SXR_REC_EVT_WRITE_ERROR,   /* write failed -> auto-finalize, file may be incomplete */
} SxrRecordEvent;
#endif

/* Opaque handle (must precede sxr_record_event_cb_t — the callback signature
 * takes sxr_record_t as a parameter). */
typedef void* sxr_record_t;

typedef void (*sxr_record_event_cb_t)(sxr_record_t rec, SxrRecordEvent evt,
                                      const SxrRecordStats *stats, void *userdata);

/* ---- vtable ---- */
typedef struct sxr_hal_api {
    int api_version;
    /* pipeline (Plan 2 — NULL in Plan 1) */
    int      (*pipeline_create_venc)(struct SxrPipelineConfig *cfg);
    int      (*pipeline_destroy)(int pipe_id);
    int      (*framegrab_start)(int pipe_id, int ch, sxr_hal_frame_cb_t cb, void *ud);
    int      (*framegrab_stop)(int pipe_id, int ch);
    int      (*set_main_frame_cb)(int pipe_id, int ch, sxr_hal_main_frame_cb_t cb, void *sink);
    int      (*force_idr)(int pipe_id, int ch);
    /* sensors (Plan 2 — NULL in Plan 1) */
    void*    (*sensors_init)(unsigned ahz, unsigned ghz, unsigned mhz, volatile unsigned char *exit);
    int      (*sensors_set_sample_cb)(void *s, sxr_hal_sensor_cb_t cb, void *ud);
    int      (*sensors_get_latest)(void *s, double *ax, double *ay, double *az,
                                    double *gx, double *gy, double *gz);
    int      (*sensors_exit)(void *s);
    /* calibration (Plan 1) */
    int      (*calib_get_camera)(int eye, sxr_hal_camera_calib_t *out);
    int      (*calib_get_imu)(sxr_hal_imu_calib_t *out);
    /* uuid (Plan 1) */
    int      (*uuid_get)(char *buf, size_t len);
    /* utils */
    int64_t  (*pts_to_realtime_ns)(uint64_t pts, int64_t offset_ns);
    /* audio (Plan: dual AMIC, on-demand) */
    void* (*audio_start)(const SxrAudioConfig *cfg, sxr_hal_audio_cb_t cb, void *ud); /* handle, NULL=fail */
    int   (*audio_stop)(void *handle);                                               /* 0=ok */
} sxr_hal_api_t;

/* Single entry: fill *out vtable. Returns 0 on success, -1 on ABI mismatch. */
__attribute__((visibility("default"))) int sxr_hal_get_api(sxr_hal_api_t *out, int api_size);

#ifdef __cplusplus
}
#endif
#endif /* _SXR_HAL_H_ */
