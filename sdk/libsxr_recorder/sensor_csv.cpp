// SPDX-License-Identifier: Apache-2.0
//
// Save-half of the sensor recorder. Owns the three CSV FILE*. The HAL
// SensorHal dispatches each sample via sample_cb; this cb writes the right
// CSV (gated by whether files are open). switch_output_dir opens/closes
// files locally (no HAL call).
#include "sensor_csv.h"
#include "hal_loader.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <pthread.h>
#include <sys/stat.h>
#include <unistd.h>     /* fsync */

class SensorCsv {
public:
    SensorCsv() : m_hal(NULL), m_fpAccel(NULL), m_fpGyro(NULL), m_fpMag(NULL),
                  m_accelSinceFlush(0), m_magSinceFlush(0),
                  m_latestTsNs(0), m_haveAccelGyro(false) {
        memset(m_latestAccel, 0, sizeof(m_latestAccel));
        memset(m_latestGyro,  0, sizeof(m_latestGyro));
        memset(m_latestMag,   0, sizeof(m_latestMag));
        pthread_mutex_init(&m_fileMutex, NULL);
    }
    ~SensorCsv() {
        CloseFiles();
        pthread_mutex_destroy(&m_fileMutex);
    }

    int Init(unsigned accel_hz, unsigned gyro_hz, unsigned mag_hz,
             const char *output_dir, volatile unsigned char *exit_flag) {
        const sxr_hal_api_t *hal = mi_hal();
        if (!hal || !hal->sensors_init) return -1;
        m_hal = hal->sensors_init(accel_hz, gyro_hz, mag_hz, exit_flag);
        if (!m_hal) return -1;
        /* register the sample callback (cache-only until SwitchOutputDir opens files) */
        if (hal->sensors_set_sample_cb)
            hal->sensors_set_sample_cb(m_hal, SensorCsv::SampleCbTramp, this);
        return 0;
    }

    int GetLatest(double *ax,double *ay,double *az,double *gx,double *gy,double *gz) {
        const sxr_hal_api_t *hal = mi_hal();
        return (hal && hal->sensors_get_latest && m_hal)
            ? hal->sensors_get_latest(m_hal, ax,ay,az,gx,gy,gz) : -1;
    }

    /* v2: all 9 axes + ts from the local preview cache. ts is in CLOCK_MONOTONIC_RAW
     * (all sensors arrive in that domain from HAL) — no anchor is applied to the
     * preview path, so the consumer (sxrservice imu_get_latest) sees uncorrected
     * monotonic time, immune to NTP jumps. Returns -1 until at least one accel
     * or gyro sample has been received. */
    int GetLatestEx(double accel[3], double gyro[3], double mag[3], int64_t *ts_ns) {
        pthread_mutex_lock(&m_fileMutex);
        if (!m_haveAccelGyro) {
            pthread_mutex_unlock(&m_fileMutex);
            return -1;
        }
        memcpy(accel, m_latestAccel, sizeof(m_latestAccel));
        memcpy(gyro,  m_latestGyro,  sizeof(m_latestGyro));
        memcpy(mag,   m_latestMag,   sizeof(m_latestMag));
        *ts_ns = m_latestTsNs;
        pthread_mutex_unlock(&m_fileMutex);
        return 0;
    }

    int SwitchOutputDir(const char *new_dir) {
        pthread_mutex_lock(&m_fileMutex);
        CloseFiles();
        if (!new_dir || !new_dir[0]) {
            pthread_mutex_unlock(&m_fileMutex);
            printf("[SensorCsv] SwitchOutputDir(NULL) -> cache-only\n");
            return 0;
        }
        char sensors_dir[512];
        snprintf(sensors_dir, sizeof(sensors_dir), "%s/Sensors", new_dir);
        mkdir(sensors_dir, 0777);
        char p[1024];
        snprintf(p, sizeof(p), "%s/accel.csv", sensors_dir); m_fpAccel = fopen(p, "a");
        snprintf(p, sizeof(p), "%s/gyro.csv",  sensors_dir); m_fpGyro  = fopen(p, "a");
        snprintf(p, sizeof(p), "%s/mag.csv",   sensors_dir); m_fpMag   = fopen(p, "a");
        int ok = (m_fpAccel && m_fpGyro && m_fpMag) ? 0 : -1;
        /* CSV header row — written once, only on a fresh (empty) file. Mode "a"
         * resumes an existing file; ftell==0 means it's new, so no duplicate
         * header on re-open. Units: ts_ns=CLOCK_MONOTONIC_RAW ns; accel m/s^2,
         * gyro rad/s, mag uT. (tools/imu_align_check.py tolerates the header.) */
        if (ok == 0) {
            if (ftell(m_fpAccel) == 0) fputs("ts_ns,ax,ay,az\n", m_fpAccel);
            if (ftell(m_fpGyro)  == 0) fputs("ts_ns,gx,gy,gz\n", m_fpGyro);
            if (ftell(m_fpMag)   == 0) fputs("ts_ns,mx,my,mz\n", m_fpMag);
        }
        printf("[SensorCsv] SwitchOutputDir -> %s (ok=%d)\n", new_dir, ok);
        pthread_mutex_unlock(&m_fileMutex);
        return ok;
    }

    int Destroy() {
        const sxr_hal_api_t *hal = mi_hal();
        pthread_mutex_lock(&m_fileMutex);
        CloseFiles();
        pthread_mutex_unlock(&m_fileMutex);
        int r = 0;
        if (hal && hal->sensors_exit && m_hal) r = hal->sensors_exit(m_hal);
        m_hal = NULL;
        return r;
    }

    /* called from HAL thread (via trampoline) */
    void OnSample(sxr_hal_sample_type_t type, int64_t ts_ns, const double v[3]) {
        pthread_mutex_lock(&m_fileMutex);
        /* latest-value preview cache: ALWAYS updated, in CLOCK_MONOTONIC_RAW
         * (all sensors arrive in that domain). No anchor applied — the preview
         * consumer (sxrservice imu_get_latest) wants uncorrected monotonic
         * values, immune to NTP jumps. */
        switch (type) {
        case SXR_HAL_SAMPLE_ACCEL:
            m_latestAccel[0] = v[0]; m_latestAccel[1] = v[1]; m_latestAccel[2] = v[2];
            m_haveAccelGyro = true;
            break;
        case SXR_HAL_SAMPLE_GYRO:
            m_latestGyro[0] = v[0]; m_latestGyro[1] = v[1]; m_latestGyro[2] = v[2];
            m_haveAccelGyro = true;
            break;
        case SXR_HAL_SAMPLE_MAG:
            m_latestMag[0] = v[0]; m_latestMag[1] = v[1]; m_latestMag[2] = v[2];
            break;
        }
        if (ts_ns > m_latestTsNs) m_latestTsNs = ts_ns;   /* raw MONOTONIC_RAW ts for preview */

        /* CSV file write: accel/gyro/mag arrive in CLOCK_MONOTONIC_RAW; write the
         * raw ts directly. No REALTIME anchor — every stream (video/sensor/audio)
         * shares the MONOTONIC_RAW domain so they align without offset. Gated by
         * fp (files open via SwitchOutputDir). */
        FILE *fp = NULL;
        switch (type) {
        case SXR_HAL_SAMPLE_ACCEL: fp = m_fpAccel; break;
        case SXR_HAL_SAMPLE_GYRO:  fp = m_fpGyro;  break;
        case SXR_HAL_SAMPLE_MAG:   fp = m_fpMag;   break;
        }
        if (fp) {
            fprintf(fp, "%" PRId64 ",%.7f,%.7f,%.7f\n", ts_ns, v[0], v[1], v[2]);
            /* periodic flush — preserves the old cadence (IMU ~1s, mag every 100) */
            if (type == SXR_HAL_SAMPLE_ACCEL || type == SXR_HAL_SAMPLE_GYRO) {
                if (++m_accelSinceFlush >= 1600) { fflush(fp); m_accelSinceFlush = 0; } /* ~2s @800Hz */
            } else {
                if (++m_magSinceFlush >= 100) { fflush(fp); m_magSinceFlush = 0; }
            }
        }
        pthread_mutex_unlock(&m_fileMutex);
    }

private:
    void *m_hal;                 /* HAL SensorHal handle */
    FILE *m_fpAccel, *m_fpGyro, *m_fpMag;
    int   m_accelSinceFlush, m_magSinceFlush;
    double  m_latestAccel[3];      /* latest-value cache, CLOCK_MONOTONIC_RAW ts */
    double  m_latestGyro[3];
    double  m_latestMag[3];
    int64_t m_latestTsNs;          /* max CLOCK_MONOTONIC_RAW ts across the three sets */
    bool    m_haveAccelGyro;       /* false until first accel/gyro sample */
    pthread_mutex_t m_fileMutex;

    /* fflush 只把 stdio 缓冲推进内核页缓存；FAT 上数据页和目录项(文件大小)
     * 要等 inode 回写才落卡，停止录制后立刻拔卡(未 unmount)会显示 0 字节。
     * fsync 强制数据 + 目录项立即写回 SD 卡。 */
    static void CloseOne(FILE **pp) {
        if (!*pp) return;
        fflush(*pp);
        fsync(fileno(*pp));
        fclose(*pp);
        *pp = NULL;
    }
    void CloseFiles() {
        CloseOne(&m_fpAccel);
        CloseOne(&m_fpGyro);
        CloseOne(&m_fpMag);
    }
    static void SampleCbTramp(void *ud, sxr_hal_sample_type_t type,
                              int64_t ts_ns, const double v[3]) {
        ((SensorCsv*)ud)->OnSample(type, ts_ns, v);
    }
};

/* ---- C-linkage impl wrappers (called by sxr_recorder.c) ---- */
extern "C" sxr_sensors_t sxr_sensors_init_impl(unsigned ahz, unsigned ghz, unsigned mhz,
                                             const char *output_dir,
                                             volatile unsigned char *exit_flag) {
    SensorCsv *s = new SensorCsv();
    if (s->Init(ahz, ghz, mhz, output_dir, exit_flag) != 0) { delete s; return NULL; }
    return (sxr_sensors_t)s;
}
extern "C" int sxr_sensors_exit_impl(sxr_sensors_t s) {
    if (!s) return -1;
    int r = ((SensorCsv*)s)->Destroy();
    delete (SensorCsv*)s;
    return r;
}
extern "C" int sxr_sensors_get_latest_impl(sxr_sensors_t s, double *ax,double *ay,double *az,
                                          double *gx,double *gy,double *gz) {
    return s ? ((SensorCsv*)s)->GetLatest(ax,ay,az,gx,gy,gz) : -1;
}
extern "C" int sxr_sensors_get_latest_ex_impl(sxr_sensors_t s, double accel[3], double gyro[3],
                                              double mag[3], int64_t *ts_ns) {
    if (!s || !accel || !gyro || !mag || !ts_ns) return -1;
    return ((SensorCsv*)s)->GetLatestEx(accel, gyro, mag, ts_ns);
}
extern "C" int sxr_sensors_switch_output_dir_impl(sxr_sensors_t s, const char *dir) {
    return s ? ((SensorCsv*)s)->SwitchOutputDir(dir) : -1;
}
