/* SPDX-License-Identifier: Apache-2.0 */
/* libsxr_recorder — Public C API v2 (dlopen-loaded, single-symbol vtable).
 *
 * v2: 单例语义, 5 槽位. MI pipeline / FMP4 / sensors / audio / IQ / clock
 * 全部内部默认. app 只控制 init/deinit/start/stop + 读 IMU 最新值.
 *
 * 头文件完全自包含, 不依赖 sxr_hal.h.
 */
#ifndef _SXR_RECORDER_H_
#define _SXR_RECORDER_H_

#include <stdint.h>
#include <stddef.h>
#include <dlfcn.h>
#include <pthread.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SXR_RECORDER_API_VERSION 2

/* ---- Internal opaque handles (used by sensor_csv.h / mi_record.h /
 *      FMP4Writer.cpp — not part of the v2 vtable ABI but shared across
 *      the .so's TUs). ---- */
typedef void* sxr_fmp4_t;
typedef void* sxr_sensors_t;

/* 预览帧回调 (子码流 ch1/ch3). 与 libsxr_hal 的 sxr_hal_frame_cb_t 签名一致.
 * 回调运行在库内部 preview-grab 线程, MUST return quickly. */
typedef void (*sxr_frame_cb_t)(int pipe_id, int ch,
    const uint8_t *data, size_t size, int64_t pts_us, int is_sync,
    const uint8_t *csd, size_t csd_size, void *userdata);

typedef struct {
    double  accel[3];   /* m/s^2, body frame */
    double  gyro[3];    /* rad/s, body frame */
    double  mag[3];     /* uT */
    int64_t ts_ns;      /* REALTIME */
} sxr_imu_sample_t;

typedef enum {
    SXR_REC_EVT_STARTED = 0,
    SXR_REC_EVT_IDR_TIMEOUT,
    SXR_REC_EVT_WRITE_ERROR,
} sxr_rec_evt_t;

typedef void (*sxr_rec_event_cb_t)(sxr_rec_evt_t evt, int error_code, void *userdata);

typedef struct sxr_recorder_api {
    int api_version;
    int  (*record_init)(sxr_frame_cb_t preview_cb, void *userdata);
    void (*record_deinit)(void);
    int  (*record_start)(const char *dir, sxr_rec_event_cb_t evt_cb, void *userdata);
    int  (*record_stop)(void);
    int  (*imu_get_latest)(sxr_imu_sample_t *out);
} sxr_recorder_api_t;

int sxr_recorder_get_api(sxr_recorder_api_t *out, int api_size);

#ifdef __cplusplus
}
#endif

static inline const sxr_recorder_api_t *sxr_recorder_api(void) {
    static sxr_recorder_api_t api;
    static volatile int state = 0;
    static pthread_mutex_t mtx = PTHREAD_MUTEX_INITIALIZER;
    if (state) return state > 0 ? &api : NULL;
    pthread_mutex_lock(&mtx);
    if (state == 0) {
        void *h = dlopen("/customer/sd/libsxr_recorder.so", RTLD_NOW | RTLD_LOCAL);
        if (!h) h = dlopen("/customer/sample_code/lib/libsxr_recorder.so", RTLD_NOW | RTLD_LOCAL);
        int (*get)(sxr_recorder_api_t *, int) =
            h ? (int (*)(sxr_recorder_api_t *, int))dlsym(h, "sxr_recorder_get_api") : NULL;
        state = (get && get(&api, (int)sizeof(api)) == 0) ? 1 : -1;
    }
    pthread_mutex_unlock(&mtx);
    return state > 0 ? &api : NULL;
}

#endif /* _SXR_RECORDER_H_ */
