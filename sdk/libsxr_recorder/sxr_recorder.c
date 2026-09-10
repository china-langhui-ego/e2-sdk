/* SPDX-License-Identifier: Apache-2.0 */
/* libsxr_recorder.so — v2 单例 vtable.
 *
 * 5 个槽位: record_init/deinit/start/stop + imu_get_latest.
 * MI pipeline / FMP4 / sensors / audio / IQ / clock 全部内部默认.
 */
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include "sxr_recorder.h"
#include "hal_loader.h"
#include "sensor_csv.h"
#include "mi_record.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 内部默认配置 ----
 * 主码流分辨率 / IQ bin 不再硬编码: 由 HAL 按检测到的 sensor 型号
 * (SC233HGS/SC235HGS/SC132GS) 决定并回填到 cfg 的 [OUTPUT] 字段。 */
#define DEF_ISP_CLK          216000000
#define DEF_SCL_CLK          288000000
#define DEF_VENC_GOP         30
#define DEF_VENC_BITRATE     4000000
#define DEF_PREVIEW_BITRATE  2000000
#define DEF_PREVIEW_FPS      15
#define DEF_ACCEL_HZ         800
#define DEF_GYRO_HZ          800
#define DEF_MAG_HZ           200

/* ---- 单例状态 ---- */
static struct {
    int                  inited;
    int                  pipe_id;
    int                  camera_mask;   /* HAL 探测结果: bit0=Camera0, bit1=Camera1 */
    sxr_sensors_t        sensors;
    sxr_record_t         current_rec;
    sxr_frame_cb_t       preview_cb;
    void                *preview_ud;
    sxr_rec_event_cb_t   evt_cb;
    void                *evt_ud;
    volatile unsigned char lib_exit;
    pthread_mutex_t      lock;
} g_rec = { .lock = PTHREAD_MUTEX_INITIALIZER };

/* ---- 预览异步分发队列(参考 mi_record.cpp RecordWriterThreadFunc)----
 * app 的 preview_cb(WS 发送,SO_SNDTIMEO 最多阻塞数秒)不再直接跑在 HAL
 * framegrab 取帧线程上:framegrab 回调只做 malloc+memcpy+入队,专用分发线程
 * 出队后调用 app 回调。队列满时丢弃最旧帧(预览要"新"不要"全",与录制
 * writer 的 drop-new 策略相反)。 */
#define PREVIEW_Q_CAP    16   /* 6fps x 2 路 ≈ 12 帧/s,>1s 缓冲 */
#define PREVIEW_CSD_CAP  256  /* 与 HAL annexB_scan_nalu 的 csd[256] 对齐 */

struct PreviewEntry {
    uint8_t *data;      /* malloc 拷贝;所有权: 入队者 -> 分发线程(free) */
    size_t   size;
    int64_t  pts_us;
    int      is_sync;
    int      ch;
    uint8_t  csd[PREVIEW_CSD_CAP];
    size_t   csd_size;
};

static struct {
    struct PreviewEntry entries[PREVIEW_Q_CAP];
    uint32_t        wpos, rpos, count;
    uint32_t        dropped;
    pthread_mutex_t mutex;
    pthread_cond_t  cond;
    bool            exit;
    unsigned        generation; /* 会话代际: 每次 init 递增,经 framegrab 的
                                 * userdata 传入入队回调;旧会话(detach 后尚未
                                 * 退出)线程的迟入队因代际不符被丢弃 */
    pthread_t       thread;
    bool            running;
    int             pipe_id;
    sxr_frame_cb_t  app_cb;   /* 分发目标(init 时固定,线程生命周期内不变) */
    void           *app_ud;
} g_preview_q = { .mutex = PTHREAD_MUTEX_INITIALIZER,
                  .cond  = PTHREAD_COND_INITIALIZER };

/* 跑在 HAL framegrab 线程上 — 必须快速返回(只拷贝+入队) */
static void preview_enqueue_cb(int pipe_id, int ch,
                               const uint8_t *data, size_t size,
                               int64_t pts_us, int is_sync,
                               const uint8_t *csd, size_t csd_size,
                               void *userdata)
{
    (void)pipe_id;
    uint8_t *copy = (uint8_t *)malloc(size);
    if (!copy) return;
    memcpy(copy, data, size);

    pthread_mutex_lock(&g_preview_q.mutex);
    if (g_preview_q.exit ||
        (unsigned)(uintptr_t)userdata != g_preview_q.generation) {
        /* 两种情况都直接丢弃:
         * 1) deinit 已置退出 — 分发线程已退出,入队无人消费,残留条目还会
         *    被下次 init 的字段重置覆盖成泄漏;
         * 2) 代际不符 — 这是上一会话被 detach 的旧 framegrab 线程
         *    (framegrab_stop 只 detach 不 join,线程靠 exit_flag 自然退出,
         *    最长滞后 ~1s),其帧属于旧会话,不能进入新队列。 */
        pthread_mutex_unlock(&g_preview_q.mutex);
        free(copy);
        return;
    }
    if (g_preview_q.count == PREVIEW_Q_CAP) {
        /* 队满: 丢弃最旧帧,腾出槽位写新帧(消费者太慢时画面保持最新) */
        free(g_preview_q.entries[g_preview_q.rpos].data);
        g_preview_q.rpos = (g_preview_q.rpos + 1) % PREVIEW_Q_CAP;
        g_preview_q.count--;
        g_preview_q.dropped++;
    }
    struct PreviewEntry *e = &g_preview_q.entries[g_preview_q.wpos];
    e->data    = copy;
    e->size    = size;
    e->pts_us  = pts_us;
    e->is_sync = is_sync;
    e->ch      = ch;
    e->csd_size = 0;
    if (csd && csd_size > 0) {
        size_t n = csd_size > PREVIEW_CSD_CAP ? PREVIEW_CSD_CAP : csd_size;
        memcpy(e->csd, csd, n);
        e->csd_size = n;
    }
    g_preview_q.wpos = (g_preview_q.wpos + 1) % PREVIEW_Q_CAP;
    g_preview_q.count++;
    pthread_cond_signal(&g_preview_q.cond);
    pthread_mutex_unlock(&g_preview_q.mutex);
}

static void *PreviewDispatchThreadFunc(void *arg)
{
    (void)arg;
    while (1) {
        pthread_mutex_lock(&g_preview_q.mutex);
        while (g_preview_q.count == 0 && !g_preview_q.exit)
            pthread_cond_wait(&g_preview_q.cond, &g_preview_q.mutex);
        if (g_preview_q.count == 0 && g_preview_q.exit) {
            pthread_mutex_unlock(&g_preview_q.mutex);
            break;
        }
        struct PreviewEntry e = g_preview_q.entries[g_preview_q.rpos];
        g_preview_q.rpos = (g_preview_q.rpos + 1) % PREVIEW_Q_CAP;
        g_preview_q.count--;
        pthread_mutex_unlock(&g_preview_q.mutex);

        /* 锁外调用 app 回调(可能阻塞: WS 发送),只影响本线程 */
        if (g_preview_q.app_cb)
            g_preview_q.app_cb(g_preview_q.pipe_id, e.ch, e.data, e.size,
                               e.pts_us, e.is_sync,
                               e.csd_size ? e.csd : NULL, e.csd_size,
                               g_preview_q.app_ud);
        free(e.data);
    }
    return NULL;
}

/* ---- v1 事件回调 → v2 适配 ---- */
static void evt_trampoline(sxr_record_t rec, SxrRecordEvent evt,
                            const SxrRecordStats *stats, void *ud)
{
    (void)rec; (void)ud;
    sxr_rec_evt_t e = (evt == SXR_REC_EVT_STARTED)     ? SXR_REC_EVT_STARTED
                    : (evt == SXR_REC_EVT_IDR_TIMEOUT) ? SXR_REC_EVT_IDR_TIMEOUT
                                                       : SXR_REC_EVT_WRITE_ERROR;
    int err = stats ? stats->error : 0;
    if (g_rec.evt_cb) g_rec.evt_cb(e, err, g_rec.evt_ud);
    /* 错误事件仅置 STOPPING 并撤回 current_record(mi_record 内部),
     * sink 关闭 / writer join / SxrRecord 释放由 app 调用 record_stop 完成
     * (stop 对已 STOPPING 的 record 幂等)。不可在此清 g_rec.current_rec,
     * 否则 app 的 record_stop 返回 -1, 泄漏 record+线程+sink。 */
}

/* ---- record_init ---- */
static int v_record_init(sxr_frame_cb_t preview_cb, void *userdata)
{
    pthread_mutex_lock(&g_rec.lock);
    if (g_rec.inited) { pthread_mutex_unlock(&g_rec.lock); return -1; }

    const sxr_hal_api_t *hal = mi_hal();
    if (!hal) { pthread_mutex_unlock(&g_rec.lock); return -1; }

    /* clock /proc — hal vtable 无 clock_set_proc, 在 recorder 内联 */
    FILE *f;
    f = fopen("/proc/mi_modules/mi_isp/debug_hal/isp_clk", "w");
    if (f) { fprintf(f, "%d\n", DEF_ISP_CLK); fclose(f); }
    f = fopen("/proc/mi_modules/mi_scl/debug_hal/clk", "w");
    if (f) { fprintf(f, "%d\n", DEF_SCL_CLK); fclose(f); }
    usleep(50 * 1000);

    /* Reset shared exit flag BEFORE pipeline_create_venc reads it into
     * cfg.exit_flag: a prior record_deinit() raised lib_exit=1 to stop the old
     * worker threads, and without clearing it here the new pipeline's
     * main-grab / drop-caches / sensor threads would see exit=1 and quit
     * immediately on re-init (second BLE session). */
    g_rec.lib_exit = 0;

    /* pipeline cfg */
    SxrPipelineConfig cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.isp_clk          = DEF_ISP_CLK;
    cfg.scl_clk          = DEF_SCL_CLK;
    cfg.venc_gop         = DEF_VENC_GOP;
    cfg.venc_bitrate_bps = DEF_VENC_BITRATE;
    cfg.enable_preview   = preview_cb ? 1 : 0;
    cfg.preview_width    = 0;   /* 0=auto: 按主码流 /4 (由 HAL 检测后回填) */
    cfg.preview_height   = 0;
    cfg.preview_bitrate_bps = DEF_PREVIEW_BITRATE;
    cfg.preview_fps      = DEF_PREVIEW_FPS;
    cfg.exit_flag        = &g_rec.lib_exit;
    cfg.audio.enable          = 1;
    cfg.audio.sample_rate     = 16000;
    cfg.audio.channels        = 2;
    cfg.audio.bits_per_sample = 16;
    cfg.audio.if_gain_l       = 18;
    cfg.audio.if_gain_r       = 18;
    cfg.audio.dpga_gain_db    = 0.0;

    int pipe_id = hal->pipeline_create_venc(&cfg);
    if (pipe_id < 0) { pthread_mutex_unlock(&g_rec.lock); return -1; }

    struct RecPipeline *recp = recpipeline_alloc(pipe_id);
    if (!recp) { hal->pipeline_destroy(pipe_id); pthread_mutex_unlock(&g_rec.lock); return -1; }
    for (int ch = 0; ch < SXR_RECORDER_SNR_MAX; ch++) {
        recp->width[ch]  = cfg.width[ch];
        recp->height[ch] = cfg.height[ch];
    }
    recp->camera_mask   = cfg.camera_mask;
    g_rec.camera_mask   = cfg.camera_mask;
    recp->pts_offset_ns = cfg.pts_offset_ns;
    recp->audio_cfg     = cfg.audio;

    /* sensors (lib_exit already cleared above, before pipeline_create_venc) */
    sxr_sensors_t sensors = sxr_sensors_init_impl(DEF_ACCEL_HZ, DEF_GYRO_HZ, DEF_MAG_HZ,
                                                   NULL, &g_rec.lib_exit);
    if (!sensors) { recpipeline_free(pipe_id); hal->pipeline_destroy(pipe_id); pthread_mutex_unlock(&g_rec.lock); return -1; }

    /* preview framegrab:注册入队 trampoline,实际 app 回调由
     * PreviewDispatchThreadFunc 在独立线程执行(见文件头队列注释)。 */
    if (preview_cb) {
        /* 持锁重置: framegrab_stop 是异步 detach,旧会话线程可能仍在
         * enqueue 路径上 — 无锁重置 exit=false 会让它与入队检查形成
         * 数据竞争(UB)。代际递增经 userdata 下发,旧线程的迟入队
         * 在新会话里同样被拦截。 */
        unsigned gen;
        pthread_mutex_lock(&g_preview_q.mutex);
        g_preview_q.wpos = g_preview_q.rpos = g_preview_q.count = 0;
        g_preview_q.dropped = 0;
        g_preview_q.exit    = false;
        g_preview_q.pipe_id = pipe_id;
        g_preview_q.app_cb  = preview_cb;
        g_preview_q.app_ud  = userdata;
        gen = ++g_preview_q.generation;
        pthread_mutex_unlock(&g_preview_q.mutex);
        if (pthread_create(&g_preview_q.thread, NULL,
                           PreviewDispatchThreadFunc, NULL) == 0) {
            g_preview_q.running = true;
            /* 按 HAL 探测结果启动各 camera 的预览子码流 (VENC ch = cam*2+1) */
            for (int cam = 0; cam < SXR_RECORDER_SNR_MAX; cam++) {
                if (!(cfg.camera_mask & (1 << cam))) continue;
                hal->framegrab_start(pipe_id, cam * 2 + 1, preview_enqueue_cb,
                                     (void *)(uintptr_t)gen);
            }
        } else {
            /* 分发线程创建失败: 退化为无预览(不阻塞 init/录制) */
            g_preview_q.running = false;
            g_preview_q.app_cb  = NULL;
            g_preview_q.app_ud  = NULL;
        }
    }

    g_rec.inited     = 1;
    g_rec.pipe_id    = pipe_id;
    g_rec.sensors    = sensors;
    g_rec.preview_cb = preview_cb;
    g_rec.preview_ud = userdata;
    g_rec.current_rec = NULL;
    pthread_mutex_unlock(&g_rec.lock);
    return 0;
}

/* ---- record_deinit ---- */
static void v_record_deinit(void)
{
    pthread_mutex_lock(&g_rec.lock);
    if (!g_rec.inited) { pthread_mutex_unlock(&g_rec.lock); return; }

    const sxr_hal_api_t *hal = mi_hal();

    /* Signal ALL worker threads to exit BEFORE any join. lib_exit is shared by
     * the IMU/mag sensor threads (m_pExit), the main-grab drain threads and the
     * drop-caches thread (cfg.exit_flag). Without this, those threads loop on
     * `while (*pExit == 0)` forever and the joins inside sensors_exit_impl /
     * pipeline_destroy block permanently -> record_deinit never returns.
     * pipeline_destroy later checks p->exit_flag but never sets it, so the
     * flag must be raised here. */
    g_rec.lib_exit = 1;

    if (g_rec.current_rec) {
        sxr_record_stop_impl(g_rec.current_rec, NULL);
        g_rec.current_rec = NULL;
    }

    if (g_rec.preview_cb && hal) {
        for (int cam = 0; cam < SXR_RECORDER_SNR_MAX; cam++) {
            if (!(g_rec.camera_mask & (1 << cam))) continue;
            hal->framegrab_stop(g_rec.pipe_id, cam * 2 + 1);
        }
    }

    /* 先停 framegrab(生产者)再停分发线程: 置 exit + 唤醒 + join。
     * 分发线程只在 count==0 && exit 时退出,故 join 返回时队列必为空,
     * 无需(也不可)在此无锁排空 — 停止窗口期的迟入队由
     * preview_enqueue_cb 的 exit 检查拦截并自行 free。
     * join 返回后 app 回调不会再被触发,app 侧随后清 g_ws 等状态安全。 */
    if (g_preview_q.running) {
        pthread_mutex_lock(&g_preview_q.mutex);
        g_preview_q.exit = true;
        pthread_cond_broadcast(&g_preview_q.cond);
        pthread_mutex_unlock(&g_preview_q.mutex);
        pthread_join(g_preview_q.thread, NULL);
        g_preview_q.running = false;
        if (g_preview_q.dropped)
            printf("[recorder] preview dropped %u frames (queue full)\n",
                   g_preview_q.dropped);
        g_preview_q.app_cb = NULL;
        g_preview_q.app_ud = NULL;
    }

    if (g_rec.sensors) {
        sxr_sensors_exit_impl(g_rec.sensors);
        g_rec.sensors = NULL;
    }

    if (hal) hal->pipeline_destroy(g_rec.pipe_id);
    recpipeline_free(g_rec.pipe_id);

    g_rec.inited = 0;
    g_rec.pipe_id = -1;
    g_rec.preview_cb = NULL;
    g_rec.preview_ud = NULL;
    pthread_mutex_unlock(&g_rec.lock);
}

/* ---- record_start ---- */
static int v_record_start(const char *dir, sxr_rec_event_cb_t evt_cb, void *userdata)
{
    if (!dir || !dir[0]) return -1;
    pthread_mutex_lock(&g_rec.lock);
    if (!g_rec.inited || g_rec.current_rec) { pthread_mutex_unlock(&g_rec.lock); return -1; }

    g_rec.evt_cb = evt_cb;
    g_rec.evt_ud = userdata;

    sxr_record_t rec = sxr_record_start_impl(g_rec.pipe_id, g_rec.sensors, dir,
                                              NULL, evt_trampoline, NULL);
    if (!rec) { pthread_mutex_unlock(&g_rec.lock); return -1; }
    g_rec.current_rec = rec;
    pthread_mutex_unlock(&g_rec.lock);
    return 0;
}

/* ---- record_stop ---- */
static int v_record_stop(void)
{
    pthread_mutex_lock(&g_rec.lock);
    if (!g_rec.inited || !g_rec.current_rec) { pthread_mutex_unlock(&g_rec.lock); return -1; }
    sxr_record_t rec = g_rec.current_rec;
    g_rec.current_rec = NULL;
    int ret = sxr_record_stop_impl(rec, NULL);
    pthread_mutex_unlock(&g_rec.lock);
    return ret;
}

/* ---- imu_get_latest ---- */
static int v_imu_get_latest(sxr_imu_sample_t *out)
{
    if (!out) return -1;
    pthread_mutex_lock(&g_rec.lock);
    if (!g_rec.inited || !g_rec.sensors) { pthread_mutex_unlock(&g_rec.lock); return -1; }
    int ret = sxr_sensors_get_latest_ex_impl(g_rec.sensors,
                                              out->accel, out->gyro, out->mag, &out->ts_ns);
    pthread_mutex_unlock(&g_rec.lock);
    return ret;
}

/* ---- get_api ---- */
int sxr_recorder_get_api(sxr_recorder_api_t *out, int api_size)
{
    if (!out) return -1;
    if (api_size < (int)sizeof(sxr_recorder_api_t)) return -1;
    memset(out, 0, sizeof(*out));
    out->api_version    = SXR_RECORDER_API_VERSION;
    out->record_init    = v_record_init;
    out->record_deinit  = v_record_deinit;
    out->record_start   = v_record_start;
    out->record_stop    = v_record_stop;
    out->imu_get_latest = v_imu_get_latest;
    return 0;
}

#ifdef __cplusplus
}
#endif
