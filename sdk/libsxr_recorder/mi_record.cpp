// SPDX-License-Identifier: Apache-2.0
//
// libsxr_recorder internal — record session save-half (Plan 2b Task 4).
//
// Relocated from mi_pipeline.cpp: record_fire_event, RecordWriterThreadFunc,
// sxr_record_start_impl, sxr_record_stop_impl, and the save-half of
// MainGrabThreadFunc (now the write_cb state machine).
//
// The HAL (sxr_pipeline_core.cpp) owns the device half: it runs MainGrabThreadFunc,
// does select()->GetStream->pure-Annex-B-concat->NAL-scan->ReleaseStream, reads
// the AE /proc ring, and dispatches main_frame_cb with sampleBuf ownership
// transferred. This file owns the SAVE half: the record state machine
// (PENDING_IDR -> WRITING -> STOPPING), the async fMP4/metainfo writer thread,
// and the record_start/stop orchestration.
//
// MI-FREE: no mi_*.h includes. fMP4 + sensors are consumed via their
// sxr_*_impl entry points (FMP4Writer.cpp / sensor_csv.cpp), linked
// internally within the .so alongside this TU (declared below).

#include "mi_record.h"
#include "hal_loader.h"      // mi_hal()
#include "calib_json.h"      // calib_json_write_*

#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <unistd.h>
#include <sys/stat.h>        // mkdir
#include <sys/types.h>
#include <inttypes.h>        // PRId64

/* Recursively mkdir a path (like `mkdir -p`), creating missing parents.
 * Returns 0 on success or if the path already exists as a dir. */
static int mkdir_p(const char *path)
{
    char tmp[768];
    size_t len = strlen(path);
    if (len == 0 || len >= sizeof(tmp)) return -1;
    strcpy(tmp, path);
    /* strip trailing slashes */
    while (len > 1 && tmp[len - 1] == '/') tmp[--len] = 0;
    for (char *p = tmp + 1; *p; p++) {
        if (*p == '/') {
            *p = 0;
            if (mkdir(tmp, 0777) != 0 && errno != EEXIST) return -1;
            *p = '/';
        }
    }
    if (mkdir(tmp, 0777) != 0 && errno != EEXIST) return -1;
    return 0;
}

/* fflush 只推进到内核页缓存；FAT 上数据页 + 目录项(文件大小)要等 inode
 * 回写才落卡，停止录制后立刻拔卡(未 unmount)会得到 0 字节文件。统一用
 * 本函数关闭录制产物：fflush + fsync(强制落卡) + fclose。 */
static void fclose_fsync(FILE *fp)
{
    if (!fp) return;
    fflush(fp);
    fsync(fileno(fp));
    fclose(fp);
}

/* fMP4 + sensors entry points (defined in FMP4Writer.cpp / sensor_csv.cpp,
 * linked internally within the .so). C linkage — declared extern "C" at their
 * definition. The public API is now vtable-only (sxr_recorder_get_api); the
 * direct sxr_fmp4_xxx and sxr_sensors_xxx wrappers were removed, so internal
 * TUs call these _impl entry points directly. */
extern "C" {
extern sxr_fmp4_t sxr_fmp4_open_impl(const char *path);
extern int sxr_fmp4_set_video_track_impl(sxr_fmp4_t f, int w, int h, int ts,
                                          const uint8_t *csd, size_t csd_len, int si);
extern int sxr_fmp4_start_impl(sxr_fmp4_t f, size_t prealloc);
extern int sxr_fmp4_write_sample_impl(sxr_fmp4_t f, const uint8_t *d, size_t sz,
                                       int64_t pts, int sync);
extern int sxr_fmp4_close_impl(sxr_fmp4_t f);
extern int sxr_fmp4_set_audio_track_impl(sxr_fmp4_t f, int sample_rate, int channels,
                                          int bits_per_sample);
extern int sxr_sensors_switch_output_dir_impl(sxr_sensors_t s, const char *dir);
}

/* Forward decls for the stereo start-frame alignment helpers (used by
 * write_cb's REC_PENDING_IDR branch). Defined after write_cb. */
static void pending_idr_free_all(struct SxrRecord *r);
static int  try_choose_aligned_start(struct SxrRecord *r, int just_buffered);
static void commit_eye_idr(struct SxrRecord *r, struct RecPipeline *recp,
                           int sensorCh, int slot);

/* ========================================================================
 * RecPipeline slot table  (one per pipe_id; populated by Task 5's
 * sxr_pipeline_create_venc recorder-wrapper via recpipeline_alloc)
 * ======================================================================== */

static struct RecPipeline g_recpipes[MAX_REC_PIPES];

struct RecPipeline *recpipeline_get(int pipe_id)
{
    if (pipe_id < 0 || pipe_id >= MAX_REC_PIPES) return NULL;
    return &g_recpipes[pipe_id];
}

struct RecPipeline *recpipeline_alloc(int pipe_id)
{
    struct RecPipeline *recp = recpipeline_get(pipe_id);
    if (!recp) return NULL;
    memset(recp, 0, sizeof(*recp));
    recp->in_use = true;
    return recp;
}

void recpipeline_free(int pipe_id)
{
    struct RecPipeline *recp = recpipeline_get(pipe_id);
    if (!recp) return;
    recp->in_use = false;
}

/* ========================================================================
 * Event dispatch  (verbatim port from mi_pipeline.cpp:1192)
 * ======================================================================== */

static void record_fire_event(SxrRecord *r, SxrRecordEvent evt)
{
    if (r && r->evt_cb) r->evt_cb((sxr_record_t)r, evt, &r->stats, r->userdata);
}

/* ========================================================================
 * sensor_time(MONOTONIC_RAW) -> UTC(CLOCK_REALTIME) mapping
 *
 * Every sensor/video/audio timestamp is CLOCK_MONOTONIC_RAW (arbitrary boot
 * epoch, no wall-clock meaning). sensor_time_to_utc.csv lets a consumer
 * recover UTC for any such ts. A single row captured at record_start would go
 * stale if the device clock is re-calibrated mid-record (NTP slew/step after
 * boot sync, manual date set), so a row is appended every
 * SXR_UTC_MAP_PERIOD_US and the consumer piecewise-linear-interpolates between
 * adjacent rows; a UTC jump then shows up directly in the offset_ns column.
 *
 * The sampler runs on its own normal-priority thread — never on the RT
 * main-grab thread (write_cb), where file I/O would jitter the frame path.
 * Spawned in record_start after the first row + publish; joined in
 * record_stop; rollback only fcloses (the thread is post-publish only).
 * ======================================================================== */

#define SXR_UTC_MAP_PERIOD_US 1000000   /* ~1 Hz: slew interp err < µs, step ambiguity < 1s */

/* Capture MONOTONIC_RAW and REALTIME near-simultaneously and append one
 * <sensor_time_ns,utc_ns,offset_ns> row. Three-point read of the monotonic
 * clock (m1, m2 bracketing the realtime read) -> midpoint halves the
 * inter-syscall jitter. fp must be non-NULL and open. */
static void utc_map_append_row(FILE *fp)
{
    struct timespec m1, m2, rt;
    clock_gettime(CLOCK_MONOTONIC_RAW, &m1);
    clock_gettime(CLOCK_REALTIME,      &rt);
    clock_gettime(CLOCK_MONOTONIC_RAW, &m2);
    int64_t mono = (  (int64_t)m1.tv_sec * 1000000000LL + m1.tv_nsec
                    + (int64_t)m2.tv_sec * 1000000000LL + m2.tv_nsec) / 2;
    int64_t utc  =  (int64_t)rt.tv_sec * 1000000000LL + rt.tv_nsec;
    int64_t off  = utc - mono;
    fprintf(fp, "%" PRId64 ",%" PRId64 ",%" PRId64 "\n", mono, utc, off);
    fflush(fp);
}

static void *UtcMapSamplerThreadFunc(void *arg)
{
    struct SxrRecord *r = (struct SxrRecord *)arg;
    /* Sleep in period/10 chunks so record_stop's join returns within one chunk
     * (~100ms) instead of blocking up to a full period. Sample cadence stays
     * ~1 Hz. */
    const useconds_t chunk = SXR_UTC_MAP_PERIOD_US / 10;
    useconds_t elapsed = 0;
    while (!__atomic_load_n(&r->utc_stop, __ATOMIC_ACQUIRE)) {
        usleep(chunk);
        elapsed += chunk;
        if (__atomic_load_n(&r->utc_stop, __ATOMIC_ACQUIRE)) break;
        if (elapsed >= SXR_UTC_MAP_PERIOD_US) {
            elapsed = 0;
            if (r->utc_fp) utc_map_append_row(r->utc_fp);
        }
    }
    return NULL;
}

/* ========================================================================
 * Async record writer thread
 * (port of mi_pipeline.cpp:1225 RecordWriterThreadFunc; operates on RecPipeline)
 *
 * Decouples fMP4/metainfo writes from the HAL main-grab thread. write_cb
 * pushes pure-Annex-B sampleBuf entries here; this thread does the slow
 * sxr_fmp4_write_sample_impl + fprintf(metainfo) and frees sampleBuf.
 * ======================================================================== */

static void *RecordWriterThreadFunc(void *arg)
{
    struct RecPipeline *recp = (struct RecPipeline *)arg;
    while (1) {
        pthread_mutex_lock(&recp->write_q_mutex);
        while (recp->count == 0 && !recp->write_q_exit)
            pthread_cond_wait(&recp->write_q_cond, &recp->write_q_mutex);
        if (recp->count == 0 && recp->write_q_exit) {
            pthread_mutex_unlock(&recp->write_q_mutex);
            break;
        }

        /* Pop entry under lock; set busy flag so record_stop knows I/O in-flight */
        struct RecordWriteEntry e = recp->write_q_entries[recp->rpos];
        recp->rpos = (recp->rpos + 1) % RECORD_WRITE_Q_CAP;
        recp->count--;
        recp->write_q_busy = true;
        pthread_mutex_unlock(&recp->write_q_mutex);

        /* Re-acquire the current record and check state. Write while the record
         * is active and this eye has committed (idr_captured). The start IDR is
         * enqueued by commit_eye_idr while the record is still REC_PENDING_IDR
         * (state flips to WRITING only after ALL eyes commit), so gating on
         * state==REC_WRITING here would DROP the committed start IDR — exactly
         * the missing frame#1 that caused the k=+1 stereo slip. Accept
         * PENDING_IDR (start IDR en route) as well as WRITING. After
         * record_stop nullifies current_record, rec==NULL and we drop. */
        struct SxrRecord *rec = NULL;
        __atomic_load(&recp->current_record, &rec, __ATOMIC_ACQUIRE);
        int state = rec ? __atomic_load_n(&rec->state, __ATOMIC_ACQUIRE) : -1;

        if (rec && state != REC_STOPPING && rec->cam[e.sensor_ch].idr_captured) {
            int wr = sxr_fmp4_write_sample_impl(rec->cam[e.sensor_ch].fmp4,
                                          e.data, e.size, e.pts_us, e.is_sync);
            if (rec->cam[e.sensor_ch].metainfo) {
                fprintf(rec->cam[e.sensor_ch].metainfo,
                        "%" PRId64 ",%" PRId64 ",%" PRId64 ",%u\n",
                        e.pts_us, e.exposure_start_ns, e.exposure_duration_ns,
                        e.gain);
            }
            if (wr != 0) {
                rec->error = EIO;
                int expected = REC_WRITING;
                if (__atomic_compare_exchange_n(&rec->state, &expected, REC_STOPPING,
                                                 1, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE)) {
                    __atomic_store_n(&recp->current_record, NULL, __ATOMIC_RELEASE);
                    record_fire_event(rec, SXR_REC_EVT_WRITE_ERROR);
                }
            }
        }

        recp->write_q_busy = false;
        free(e.data);   /* sampleBuf ownership ends here */
    }

    if (recp->dropped > 0)
        printf("[mi_record] writer: dropped %u frames (queue full)\n",
               recp->dropped);
    return NULL;
}

/* ========================================================================
 * write_cb — the main_frame_cb target (runs on HAL main-grab thread)
 *
 * Port of the SAVE half of mi_pipeline.cpp:1293 MainGrabThreadFunc. The HAL
 * already did: pure-Annex-B concat, NAL scan (csd/is_sync), ReleaseStream,
 * and the AE /proc ring read (exposure_start_ns/duration_ns/gain computed).
 *
 * This runs on the HAL thread and MUST be fast: state check + first-IDR
 * set_video_track/start + enqueue. It must NOT call write_sample (the writer
 * thread does that). sampleBuf (data)
 * ownership is transferred to the write_q entry (writer frees) OR freed
 * here on every non-enqueued path (drain/drop/not-captured).
 *
 * Signature = sxr_hal_main_frame_cb_t (declared extern "C" in sxr_hal.h, so
 * this function needs C language linkage to be assignable to the vtable slot).
 * ======================================================================== */

extern "C" {
static void write_cb(void *sink, int pipe_id, int ch, uint8_t *data, size_t size,
                     int64_t pts_us, int is_sync, uint8_t *csd, size_t csd_size,
                     uint32_t expo_us, uint32_t gain, uint64_t drv_seq,
                     int64_t exposure_start_ns, int64_t exposure_duration_ns)
{
    (void)expo_us; (void)drv_seq;   /* AE timing already resolved by HAL */

    struct SxrRecord *r = (struct SxrRecord *)sink;
    if (!r || !data) { if (data) free(data); return; }

    struct RecPipeline *recp = recpipeline_get(pipe_id);
    int sensorCh = ch / 2;          /* HAL passes VENC main ch (0 or 2) */
    int st = __atomic_load_n(&r->state, __ATOMIC_ACQUIRE);
    int64_t pts_offset = recp ? recp->pts_offset_ns : 0;

    /* STOPPING -> drain: free sampleBuf, do not process */
    if (st == REC_STOPPING) { free(data); return; }

    /* IDR-timeout watchdog: PENDING_IDR past deadline -> STOPPING + IDR_TIMEOUT.
     * (port of mi_pipeline.cpp:1366-1379) */
    if (st == REC_PENDING_IDR) {
        struct timespec tsNow;
        clock_gettime(CLOCK_REALTIME, &tsNow);
        int64_t now_ns = (int64_t)tsNow.tv_sec * 1000000000LL + tsNow.tv_nsec;
        if (now_ns > r->pending_deadline_ns) {
            int expected = REC_PENDING_IDR;
            if (__atomic_compare_exchange_n(&r->state, &expected, REC_STOPPING,
                                             1, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE)) {
                r->error = ETIMEDOUT;
                if (recp)
                    __atomic_store_n(&recp->current_record, NULL, __ATOMIC_RELEASE);
                record_fire_event(r, SXR_REC_EVT_IDR_TIMEOUT);
            }
            free(data);
            return;
        }
    }

    /* PENDING_IDR: buffer this eye's IDR candidates; once every enabled eye
     * has >=1, compare their forced-IDR exposure_start. If all eyes are within
     * one frame period (exposure-synchronous), commit each on its own IDR ->
     * Camera0/Camera1 row 0 = same physical exposure (no k=±1 slip). If they
     * differ by >= one period (the record_start forced-IDR landed on different
     * sensor frames), re-request an IDR on the LAGGING eye(s) so they emit a
     * fresh IDR on the current frame, then re-compare on the next callbacks. */
    if (st == REC_PENDING_IDR && is_sync && !r->cam[sensorCh].idr_captured &&
        csd_size > 0 && csd) {
        int cnt = r->pending_idr_count[sensorCh];
        if (cnt < 4) {
            /* buffer this IDR candidate (take ownership of data) */
            typeof(r->pending_idr[sensorCh][0]) *s = &r->pending_idr[sensorCh][cnt];
            s->data                 = data;
            s->size                 = size;
            s->pts_us               = pts_us;
            s->exposure_start_ns    = exposure_start_ns;   /* raw MONOTONIC_RAW */
            s->exposure_duration_ns = exposure_duration_ns;
            s->gain                 = gain;
            memcpy(s->csd, csd, csd_size > sizeof(s->csd) ? sizeof(s->csd) : csd_size);
            s->csd_size             = csd_size > sizeof(s->csd) ? sizeof(s->csd) : csd_size;
            s->used                 = 1;
            r->pending_idr_count[sensorCh] = cnt + 1;

            /* Record duration baseline on FIRST candidate: capture the first
             * IDR's MONOTONIC_RAW capture instant (pts_us*1000 + pts_offset).
             * No REALTIME anchor — all outputs (video/sensor/audio) stay raw
             * MONOTONIC_RAW, sharing one domain so they align without offset.
             * NB start_realtime_ns holds MONOTONIC_RAW here (legacy field name
             * kept for the stats ABI; consumed only by duration_ns below). */
            if (r->stats.start_realtime_ns == 0) {
                r->stats.start_realtime_ns = pts_us * 1000LL + pts_offset;
            }

            try_choose_aligned_start(r, sensorCh);
        } else {
            /* cap: degraded — commit each eye on its earliest so all start. */
            printf("[mi_record:%d] pending-IDR cap: committing earliest per eye "
                   "(slip possible)\n", pipe_id);
            for (int e = 0; e < SXR_RECORDER_SNR_MAX; e++)
                if ((r->enabled_mask & (1 << e)) && r->pending_idr_count[e] > 0)
                    commit_eye_idr(r, recp, e, 0);
            free(data);
        }
        return;
    }

    /* PENDING_IDR non-IDR frames (P-frames before the aligned start): drop.
     * Also any frame for an eye that hasn't committed yet. */
    if (st == REC_PENDING_IDR || !r->cam[sensorCh].idr_captured) {
        free(data);
        return;
    }

    /* WRITING (or just-transitioned) + idr_captured -> enqueue sampleBuf to the
     * async writer; else free it. (port of mi_pipeline.cpp:1456-1508, s/p->/recp->/) */
    if (r->cam[sensorCh].idr_captured) {
        r->stats.frames[sensorCh]++;
        r->stats.bytes[sensorCh] += size;
        r->stats.duration_ns = pts_us * 1000LL + pts_offset
                                - r->stats.start_realtime_ns;

        bool pushed = false;
        if (recp) {
            pthread_mutex_lock(&recp->write_q_mutex);
            if (recp->count < RECORD_WRITE_Q_CAP) {
                struct RecordWriteEntry *e = &recp->write_q_entries[recp->wpos];
                e->data                  = data;            /* ownership -> writer */
                e->size                  = size;
                e->pts_us                = pts_us;
                e->is_sync               = is_sync ? 1 : 0;
                e->sensor_ch             = sensorCh;
                e->exposure_start_ns     = exposure_start_ns;   /* raw MONOTONIC_RAW */
                e->exposure_duration_ns  = exposure_duration_ns;
                e->gain                  = gain;
                recp->wpos = (recp->wpos + 1) % RECORD_WRITE_Q_CAP;
                recp->count++;
                pthread_cond_signal(&recp->write_q_cond);
                pushed = true;
            } else {
                recp->dropped++;
            }
            pthread_mutex_unlock(&recp->write_q_mutex);
        }
        if (!pushed) free(data);   /* queue full or no recp: reclaim sampleBuf */
        /* sampleBuf now owned by writer thread; don't free here */
    } else {
        free(data);                /* not yet idr_captured: discard */
    }
}


/* AudioWriterThreadFunc — owns ALL audio file I/O (fMP4 write + metainfo
 * fprintf). Pops PCM chunks queued by audio_cb; on exit signal, drains the
 * queue before returning so record_stop loses nothing. The audio track
 * timescale = sample rate, so 1 fmp4 time-unit = 1 PCM frame. Sample PTS =
 * base_frames + cumulative frames (becomes the fragment tfdt). base_frames
 * 是首块 PCM 的 pts_us 换算成采样帧：pts_us 与视频 pts_us 共用同一 MI_SYS
 * 时钟域（见 sxr_hal.h），因此音轨起始时间与视频轨一致，不再从 0 开始。
 * pts_us/mono_ns 仍记录在 metainfo.csv 用于跨流对齐。 */
static void *AudioWriterThreadFunc(void *arg)
{
    struct SxrRecord *r = (struct SxrRecord *)arg;
    int ch = r->audio.channels > 0 ? r->audio.channels : 2;

    for (;;) {
        pthread_mutex_lock(&r->audio.q_mutex);
        while (r->audio.q_count == 0 && !r->audio.q_exit)
            pthread_cond_wait(&r->audio.q_cond, &r->audio.q_mutex);
        if (r->audio.q_count == 0 && r->audio.q_exit) {
            pthread_mutex_unlock(&r->audio.q_mutex);
            break;
        }
        struct AudioWriteEntry e = r->audio.q[r->audio.q_rpos];
        r->audio.q_rpos = (r->audio.q_rpos + 1) % AUDIO_WRITE_Q_CAP;
        r->audio.q_count--;
        pthread_mutex_unlock(&r->audio.q_mutex);

        if (__atomic_load_n(&r->audio.opened, __ATOMIC_ACQUIRE)) {
            unsigned frames = (unsigned)(e.size / ((size_t)ch * 2u));  // S16_LE
            if (r->audio.fmp4) {
                if (!r->audio.base_set) {
                    int rate = r->audio.sample_rate > 0 ? r->audio.sample_rate : 16000;
                    r->audio.base_frames = e.pts_us * rate / 1000000;
                    r->audio.base_set    = true;
                }
                if (sxr_fmp4_write_sample_impl(r->audio.fmp4, e.data, e.size,
                        r->audio.base_frames + (int64_t)r->audio.frames_written, 0) != 0) {
                    /* write failed (e.g. SD full): disable audio for the rest
                     * of this segment, keep the video record intact (spec §8:
                     * audio write errors are non-fatal to video). */
                    __atomic_store_n(&r->audio.opened, false, __ATOMIC_RELEASE);
                    printf("[mi_record] audio write failed -> audio disabled for segment\n");
                    free(e.data);
                    continue;
                }
            }
            if (r->audio.metainfo)
                fprintf(r->audio.metainfo, "%" PRId64 ",%" PRId64 ",%u\n",
                        e.pts_us, e.mono_ns, frames);
            r->audio.frames_written += frames;
        }
        free(e.data);
    }
    return NULL;
}

/* audio_cb — the HAL audio-thread target. The HAL MI_AI capture thread calls
 * this for every PCM period (S16_LE interleaved). Runs on the HAL audio thread
 * — MUST NOT do file I/O: the MI_AI driver ring is only a few hundred ms deep
 * and a slow SD write here drops capture buffers ("Buffer(s) is lost due to
 * slow fetching"). Enqueue a PCM copy and return; the audio writer thread
 * (AudioWriterThreadFunc) performs all fMP4/metainfo writes. */
static void audio_cb(void *ud, const uint8_t *pcm, size_t bytes,
                     int64_t pts_us, int64_t mono_ns)
{
    struct SxrRecord *r = (struct SxrRecord *)ud;
    if (!r || !pcm || bytes == 0) return;
    if (!r->audio.q_running) return;    /* queue not up / already torn down */
    if (!__atomic_load_n(&r->audio.opened, __ATOMIC_ACQUIRE)) return;
    int st = __atomic_load_n(&r->state, __ATOMIC_ACQUIRE);
    if (st == REC_STOPPING) return;

    uint8_t *copy = (uint8_t *)malloc(bytes);
    if (!copy) return;
    memcpy(copy, pcm, bytes);

    pthread_mutex_lock(&r->audio.q_mutex);
    if (r->audio.q_count < AUDIO_WRITE_Q_CAP) {
        struct AudioWriteEntry *e = &r->audio.q[r->audio.q_wpos];
        e->data    = copy;
        e->size    = bytes;
        e->pts_us  = pts_us;
        e->mono_ns = mono_ns;
        r->audio.q_wpos = (r->audio.q_wpos + 1) % AUDIO_WRITE_Q_CAP;
        r->audio.q_count++;
        pthread_cond_signal(&r->audio.q_cond);
    } else {
        r->audio.q_dropped++;
        free(copy);
    }
    pthread_mutex_unlock(&r->audio.q_mutex);
}
}  /* extern "C" */

/* ---- stereo start-frame alignment helpers (REC_PENDING_IDR) ---- */

/* Free every buffered pending-IDR candidate (record_stop / fallback paths). */
static void pending_idr_free_all(struct SxrRecord *r)
{
    for (int e = 0; e < SXR_RECORDER_SNR_MAX; e++) {
        for (int s = 0; s < 4; s++) {
            if (r->pending_idr[e][s].used) {
                free(r->pending_idr[e][s].data);
                r->pending_idr[e][s].data = NULL;
                r->pending_idr[e][s].used = 0;
            }
        }
        r->pending_idr_count[e] = 0;
    }
}

/* Compare each enabled eye's latest buffered forced-IDR exposure_start.
 * - If the spread (max-min) is within one frame period, the eyes are
 *   exposure-synchronous: commit each eye on its latest candidate -> row 0 of
 *   both metainfo.csv is the same physical exposure (no k=±1 slip).
 * - If the latest candidates differ by >= one period, the record_start
 *   forced-IDRs landed on different sensor frames; re-request an IDR on ALL
 *   enabled eyes at once and drop every buffered candidate. Re-forcing only
 *   the LAGGING eye deadlocks: its fresh IDR always lands exactly one frame
 *   AFTER the leader's kept candidate (the re-force is issued at/after the
 *   leader's frame instant), so the roles flip every round and the skew stays
 *   at one period forever (ES201_1735: endless 33.3ms eye0/eye1 ping-pong
 *   until the 5s watchdog, no data written). The eyes' frame grids are
 *   exposure-locked to <1ms, so IDRs forced at the same instant land on the
 *   same physical frame pair and the next decision commits.
 *
 * Returns 1 once committed, 0 while still converging. */
#define SXR_FRAME_PERIOD_NS 33333000LL   /* 30fps; eyes are exposure-locked */
/* Sync threshold: exposure-locked eyes agree to <1ms, so anything up to half
 * a frame is "same frame". MUST be well under one full period — a threshold of
 * exactly one period misjudges a 33.333ms (one-frame) skew as "aligned" at the
 * integer boundary (clip_001 committed a 33.33ms slip when using < PERIOD). */
#define SXR_SYNC_TOL_NS     (SXR_FRAME_PERIOD_NS / 2)

static int try_choose_aligned_start(struct SxrRecord *r, int just_buffered)
{
    if (r->aligned_start_expo_ns != 0) return 1;   /* already chosen */

    /* We only make a commit/re-force DECISION on the callback that newly armed
     * an eye (its pending count went 0->1 with this buffer). A leader's LATER
     * IDR (count>1, arriving while a laggard is mid re-force) must NOT trigger
     * a decision: comparing a fresh leader candidate against a stale laggard
     * one could commit a 1-frame slip if they happen to fall within one period
     * (clip_003 race). */
    if (r->pending_idr_count[just_buffered] != 1) return 0;  /* not a 0->1 arming */

    /* all enabled eyes must be armed (>=1 candidate) before deciding */
    for (int e = 0; e < SXR_RECORDER_SNR_MAX; e++) {
        if (!(r->enabled_mask & (1 << e))) continue;
        if (r->pending_idr_count[e] == 0) return 0;   /* still waiting */
    }

    /* decide on each eye's LATEST candidate (the freshest forced IDR). If a
     * leader armed extra IDRs before the laggard's first, those extras are
     * NEWER frames; comparing latest-to-latest matches how each eye actually
     * starts (on its freshest committed IDR) and keeps the two starts on the
     * same physical frame. */
    int64_t latest = 0, earliest = 0;
    int have = 0;
    for (int e = 0; e < SXR_RECORDER_SNR_MAX; e++) {
        if (!(r->enabled_mask & (1 << e))) continue;
        int64_t ex = r->pending_idr[e][r->pending_idr_count[e] - 1].exposure_start_ns;
        if (!have) { latest = earliest = ex; have = 1; }
        else { if (ex > latest) latest = ex; if (ex < earliest) earliest = ex; }
    }

    if (latest - earliest < SXR_SYNC_TOL_NS) {
        /* synchronous: commit each eye on its LATEST candidate */
        struct RecPipeline *recp = recpipeline_get(r->pipe_id);
        r->aligned_start_expo_ns = latest;
        for (int e = 0; e < SXR_RECORDER_SNR_MAX; e++) {
            if (!(r->enabled_mask & (1 << e))) continue;
            commit_eye_idr(r, recp, e, r->pending_idr_count[e] - 1);
        }
        printf("[mi_record:%d] stereo IDR aligned (spread %.2fms)\n",
               r->pipe_id, (latest - earliest) / 1e6);
        return 1;
    }

    /* skewed: drop ALL candidates on every enabled eye, then re-force ALL
     * eyes back-to-back. Both drops precede both forces so the two force_idr
     * calls are issued at (nearly) the same instant; with exposure-locked
     * grids the fresh IDRs land on the same physical frame pair and the next
     * arming decision commits. */
    const sxr_hal_api_t *hal = mi_hal();
    for (int e = 0; e < SXR_RECORDER_SNR_MAX; e++) {
        if (!(r->enabled_mask & (1 << e))) continue;
        int cnt = r->pending_idr_count[e];
        for (int s = 0; s < cnt; s++) {
            if (r->pending_idr[e][s].used) {
                free(r->pending_idr[e][s].data);
                r->pending_idr[e][s].data = NULL;
                r->pending_idr[e][s].used = 0;
            }
        }
        r->pending_idr_count[e] = 0;
    }
    for (int e = 0; e < SXR_RECORDER_SNR_MAX; e++) {
        if (!(r->enabled_mask & (1 << e))) continue;
        if (hal && hal->force_idr)
            hal->force_idr(r->pipe_id, e);
    }
    printf("[mi_record:%d] stereo IDR skew %.1fms: re-forced IDR all eyes\n",
           r->pipe_id, (latest - earliest) / 1e6);
    return 0;
}

/* Open the eye's fmp4 track + start, mark idr_captured, enqueue the chosen
 * pending-IDR candidate, free the rest. Transitions to REC_WRITING when all
 * enabled eyes are captured (fires SXR_REC_EVT_STARTED). */
static void commit_eye_idr(struct SxrRecord *r, struct RecPipeline *recp,
                           int sensorCh, int slot)
{
    if (r->cam[sensorCh].idr_captured) return;
    typeof(r->pending_idr[sensorCh][0]) *s = &r->pending_idr[sensorCh][slot];
    if (!s->used) return;

    int w = recp ? recp->width[sensorCh] : 0;
    int h = recp ? recp->height[sensorCh] : 0;
    sxr_fmp4_set_video_track_impl(r->cam[sensorCh].fmp4, w, h,
                             1000000, s->csd, s->csd_size, r->cfg.sample_interval_us);
    sxr_fmp4_start_impl(r->cam[sensorCh].fmp4, r->cfg.prealloc_bytes);
    r->cam[sensorCh].idr_captured = true;
    r->cam[sensorCh].start_expo_ns = s->exposure_start_ns;

    /* enqueue the chosen IDR as the eye's first sample + metainfo row 0 */
    r->stats.frames[sensorCh]++;
    r->stats.bytes[sensorCh] += s->size;
    if (recp) {
        pthread_mutex_lock(&recp->write_q_mutex);
        if (recp->count < RECORD_WRITE_Q_CAP) {
            struct RecordWriteEntry *e = &recp->write_q_entries[recp->wpos];
            e->data                 = s->data;   /* ownership -> writer */
            e->size                 = s->size;
            e->pts_us               = s->pts_us;
            e->is_sync              = 1;
            e->sensor_ch            = sensorCh;
            e->exposure_start_ns    = s->exposure_start_ns;
            e->exposure_duration_ns = s->exposure_duration_ns;
            e->gain                 = s->gain;
            recp->wpos = (recp->wpos + 1) % RECORD_WRITE_Q_CAP;
            recp->count++;
            pthread_cond_signal(&recp->write_q_cond);
            s->data = NULL;   /* consumed */
        } else {
            recp->dropped++;
        }
        pthread_mutex_unlock(&recp->write_q_mutex);
    }
    if (s->data) { free(s->data); s->data = NULL; }
    s->used = 0;

    /* drop this eye's other buffered candidates */
    for (int i = 0; i < r->pending_idr_count[sensorCh]; i++) {
        if (r->pending_idr[sensorCh][i].used) {
            free(r->pending_idr[sensorCh][i].data);
            r->pending_idr[sensorCh][i].data = NULL;
            r->pending_idr[sensorCh][i].used = 0;
        }
    }
    r->pending_idr_count[sensorCh] = 0;

    /* all enabled eyes captured -> WRITING + STARTED */
    int allReady = 1;
    for (int c = 0; c < SXR_RECORDER_SNR_MAX; c++)
        if ((r->enabled_mask & (1 << c)) && !r->cam[c].idr_captured) allReady = 0;
    if (allReady) {
        int expected = REC_PENDING_IDR;
        if (__atomic_compare_exchange_n(&r->state, &expected, REC_WRITING,
                                         1, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE))
            record_fire_event(r, SXR_REC_EVT_STARTED);
    }
}

/* ========================================================================
 * sxr_record_start_impl  (port of mi_pipeline.cpp:2026)
 *
 * Builds <dir>/Camera{0,1}/{video.mp4,metainfo.csv} + <dir>/Sensors CSV,
 * forces an IDR per enabled main channel, installs write_cb on the HAL, and
 * atomically publishes the record sink. Returns immediately; the first IDR
 * triggers SXR_REC_EVT_STARTED from write_cb (on the HAL thread).
 * ======================================================================== */

sxr_record_t sxr_record_start_impl(int pipe_id, sxr_sensors_t sensors,
                                  const char *dir,
                                  const SxrRecordConfig *cfg,
                                  sxr_record_event_cb_t evt_cb, void *userdata)
{
    struct RecPipeline *recp = recpipeline_get(pipe_id);
    if (!recp || !recp->in_use || !dir || !dir[0]) return NULL;

    /* cfg==NULL -> defaults (v2 record_start no longer exposes SxrRecordConfig) */
    SxrRecordConfig default_cfg = {0, 33333};
    if (!cfg) cfg = &default_cfg;

    /* Only one active record per pipe */
    struct SxrRecord *active = NULL;
    __atomic_load(&recp->current_record, &active, __ATOMIC_ACQUIRE);
    if (active) {
        printf("[mi_record:%d] record_start: another record active\n", pipe_id);
        return NULL;
    }

    const sxr_hal_api_t *hal = mi_hal();

    struct SxrRecord *r = new struct SxrRecord;
    memset(r, 0, sizeof(*r));
    r->pipe_id  = pipe_id;
    r->sensors  = sensors;
    snprintf(r->dir, sizeof(r->dir), "%s", dir);
    r->cfg = *cfg;
    if (r->cfg.sample_interval_us <= 0) r->cfg.sample_interval_us = 33333;
    r->evt_cb   = evt_cb;
    r->userdata = userdata;
    r->state    = REC_PENDING_IDR;
    r->error    = 0;

    r->stats.pts_offset_ns = recp->pts_offset_ns;

    /* enabled_mask: HAL 自动探测结果 (bit0=Camera0, bit1=Camera1) */
    r->enabled_mask = (uint8_t)recp->camera_mask;
    if (!r->enabled_mask) r->enabled_mask = 0x3;   /* 防御: 未回填则按双目 */

    bool q_inited = false;

    /* 0) mkdir segment root + any missing parents (app only passes the dir;
     *     the output root may not exist yet on a fresh --output). */
    if (mkdir_p(dir) != 0)
        printf("[mi_record:%d] record_start: mkdir_p(%s) errno=%d (%s)\n",
               pipe_id, dir, errno, strerror(errno));

    /* 0a) sensor_time_to_utc.csv: open + header + first sample now, so even a
     *     zero-frame segment has >=1 mapping row. Periodic rows are appended
     *     by UtcMapSamplerThreadFunc, spawned after publish. Non-fatal on
     *     failure (matches calib JSON / id.txt handling). */
    {
        char utcpath[768];
        snprintf(utcpath, sizeof(utcpath), "%s/sensor_time_to_utc.csv", dir);
        r->utc_fp = fopen(utcpath, "w");
        if (r->utc_fp) {
            fprintf(r->utc_fp,
                "# sensor_time(MONOTONIC_RAW) -> UTC mapping, sampled periodically during record\n"
                "# utc(t) for sensor_time t: linear interpolation between adjacent rows\n"
                "# offset_ns = utc_ns - sensor_time_ns (changes if UTC re-calibrated mid-record)\n"
                "# target period = %.3f s (actual cadence may vary slightly)\n"
                "sensor_time_ns,utc_ns,offset_ns\n",
                SXR_UTC_MAP_PERIOD_US / 1000000.0);
            utc_map_append_row(r->utc_fp);
        } else {
            printf("[mi_record:%d] record_start: open %s errno=%d (%s) (non-fatal)\n",
                   pipe_id, utcpath, errno, strerror(errno));
        }
    }

    /* 1) mkdir + open sinks per camera. Any failure -> rollback. */
    for (int ch = 0; ch < SXR_RECORDER_SNR_MAX; ch++) {
        if (!(r->enabled_mask & (1 << ch))) continue;
        char camdir[640], mp4path[768], metapath[768];
        snprintf(camdir,  sizeof(camdir),   "%s/Camera%d", dir, ch);
        if (mkdir(camdir, 0777) != 0 && errno != EEXIST)
            printf("[mi_record:%d] record_start: mkdir(%s) errno=%d (%s)\n",
                   pipe_id, camdir, errno, strerror(errno));
        snprintf(mp4path, sizeof(mp4path),  "%s/video.mp4", camdir);
        snprintf(metapath,sizeof(metapath), "%s/metainfo.csv", camdir);
        r->cam[ch].fmp4 = sxr_fmp4_open_impl(mp4path);
        r->cam[ch].metainfo = fopen(metapath, "w");
        if (!r->cam[ch].fmp4 || !r->cam[ch].metainfo) {
            printf("[mi_record:%d] record_start: open failed cam%d (%s)\n",
                   pipe_id, ch, mp4path);
            r->error = EIO;
            goto rollback;
        }
        fprintf(r->cam[ch].metainfo,
                "pts_us(exp_end),exposure_start_ns,"
                "exposure_duration_ns,gain\n");
    }

    /* 1a) audio sink: <dir>/Audio/{audio.mp4,metainfo.csv} + HAL mic capture.
     *     Non-fatal: on failure audio.opened stays false and video records
     *     without audio (mic absent / MI_AI open fail). */
    if (recp->audio_cfg.enable) {
        const sxr_hal_api_t *halA = mi_hal();
        if (halA && halA->audio_start) {
            char adir[640], amp4[768], ameta[768];
            snprintf(adir,  sizeof(adir),  "%s/Audio", dir);
            mkdir(adir, 0777);
            snprintf(amp4,  sizeof(amp4),  "%s/audio.mp4", adir);
            snprintf(ameta, sizeof(ameta), "%s/metainfo.csv", adir);
            r->audio.fmp4 = sxr_fmp4_open_impl(amp4);
            if (r->audio.fmp4) {
                sxr_fmp4_set_audio_track_impl(r->audio.fmp4,
                    recp->audio_cfg.sample_rate, recp->audio_cfg.channels,
                    recp->audio_cfg.bits_per_sample);
                sxr_fmp4_start_impl(r->audio.fmp4, r->cfg.prealloc_bytes);
            }
            r->audio.metainfo = fopen(ameta, "w");
            if (r->audio.metainfo)
                fprintf(r->audio.metainfo, "pts_us,ts_ns,samples\n");
            r->audio.channels = recp->audio_cfg.channels;
            r->audio.sample_rate = recp->audio_cfg.sample_rate;

            /* Only start the mic if both sinks opened; otherwise the ADC + capture
             * thread would spin calling audio_cb (which early-returns) for the
             * whole segment, wasting mic resources. Half-opened sinks are closed
             * in record_stop/rollback (closed unconditionally when non-NULL).
             * The async writer queue + thread come up BEFORE audio_start so the
             * capture thread's first audio_cb already has somewhere to enqueue. */
            if (r->audio.fmp4 && r->audio.metainfo) {
                r->audio.q_wpos = r->audio.q_rpos = r->audio.q_count = r->audio.q_dropped = 0;
                r->audio.q_exit = false;
                pthread_mutex_init(&r->audio.q_mutex, NULL);
                pthread_cond_init(&r->audio.q_cond, NULL);
                if (pthread_create(&r->audio.q_thread, NULL,
                                   AudioWriterThreadFunc, r) == 0) {
                    r->audio.q_running = true;
                    SxrAudioConfig acfg = recp->audio_cfg;     // share the video offset
                    acfg.pts_offset_ns = recp->pts_offset_ns;
                    r->audio.hal_handle = halA->audio_start(&acfg, audio_cb, r);
                } else {
                    printf("[mi_record:%d] record_start: audio writer thread failed\n",
                           pipe_id);
                    pthread_mutex_destroy(&r->audio.q_mutex);
                    pthread_cond_destroy(&r->audio.q_cond);
                }
            }
            r->audio.opened = (r->audio.fmp4 && r->audio.metainfo &&
                               r->audio.q_running && r->audio.hal_handle);
            if (!r->audio.opened)
                printf("[mi_record:%d] record_start: audio disabled "
                       "(fmp4=%p meta=%p hal=%p)\n", pipe_id,
                       (void*)r->audio.fmp4, (void*)r->audio.metainfo, r->audio.hal_handle);
        } else {
            printf("[mi_record:%d] record_start: HAL audio unavailable, skipping audio\n",
                   pipe_id);
        }
    }

    /* 1b) calib JSON + id.txt (HAL dlopen; failure non-fatal to recording).
     *     Plan 1 hook, kept verbatim. */
    if (hal) {
        for (int ch = 0; ch < SXR_RECORDER_SNR_MAX; ch++) {
            if (!(r->enabled_mask & (1 << ch))) continue;
            char camdir[640];
            snprintf(camdir, sizeof(camdir), "%s/Camera%d", dir, ch);
            if (calib_json_write_camera(hal, ch, camdir) != 0)
                printf("[mi_record:%d] record_start: calib_json camera%d skipped\n",
                       pipe_id, ch);
        }
        char sensordir[640];
        snprintf(sensordir, sizeof(sensordir), "%s/Sensors", dir);
        mkdir(sensordir, 0777);
        if (calib_json_write_imu(hal, sensordir) != 0)
            printf("[mi_record:%d] record_start: calib_json imu skipped\n", pipe_id);
        if (calib_json_write_id_txt(hal, dir) != 0)
            printf("[mi_record:%d] record_start: id.txt skipped\n", pipe_id);
    } else {
        printf("[mi_record:%d] record_start: HAL unavailable, skipping calib JSON/id.txt\n",
               pipe_id);
    }

    /* 2) sensors CSV -> <dir>/Sensors (cache-only -> writing) */
    if (r->sensors) {
        if (sxr_sensors_switch_output_dir_impl(r->sensors, dir) != 0)
            printf("[mi_record:%d] record_start: sensors switch failed (non-fatal)\n",
                   pipe_id);
    }

    /* 3) init write_q + start per-record writer thread */
    pthread_mutex_init(&recp->write_q_mutex, NULL);
    pthread_cond_init(&recp->write_q_cond, NULL);
    q_inited = true;
    recp->wpos = 0; recp->rpos = 0; recp->count = 0; recp->dropped = 0;
    recp->write_q_busy = false;
    recp->write_q_exit  = false;
    recp->writer_running = true;
    {
        int wret = pthread_create(&recp->writer_thread, NULL,
                                  RecordWriterThreadFunc, (void *)recp);
        if (wret) {
            recp->writer_running = false;
            printf("[mi_record:%d] record_start: writer thread failed: %d\n",
                   pipe_id, wret);
            r->error = EIO;
            goto rollback;
        }
    }

    /* 4) force IDR per enabled main channel + install write_cb (sink = r).
     *    HAL force_idr/set_main_frame_cb take the SENSOR channel (0/1); the
     *    callback receives the VENC main channel (0/2) and derives sensorCh. */
    if (hal && hal->force_idr && hal->set_main_frame_cb) {
        for (int ch = 0; ch < SXR_RECORDER_SNR_MAX; ch++) {
            if (!(r->enabled_mask & (1 << ch))) continue;
            if (hal->force_idr(pipe_id, ch) != 0)
                printf("[mi_record:%d] record_start: force_idr chn%d failed\n",
                       pipe_id, ch);
            hal->set_main_frame_cb(pipe_id, ch, write_cb, r);
        }
    } else {
        printf("[mi_record:%d] record_start: HAL vtable incomplete, no IDR/cb\n",
               pipe_id);
    }

    /* 5) IDR-timeout watchdog deadline (+5s) + atomic publish */
    {
        struct timespec _ts;
        clock_gettime(CLOCK_REALTIME, &_ts);
        r->pending_deadline_ns = (int64_t)_ts.tv_sec * 1000000000LL
                               + (int64_t)_ts.tv_nsec + 5000000000LL;
    }
    __atomic_store_n(&recp->current_record, r, __ATOMIC_RELEASE);

    /* 6) periodic UTC-mapping sampler. Spawned AFTER publish so the rollback
     *    paths (all pre-publish) never observe a live sampler thread — they
     *    only fclose utc_fp. If create fails the file still holds the
     *    record_start sample (>=1 row). */
    if (r->utc_fp) {
        r->utc_stop = 0;
        if (pthread_create(&r->utc_thread, NULL, UtcMapSamplerThreadFunc, r) == 0)
            r->utc_thread_running = 1;
        else
            printf("[mi_record:%d] record_start: utc sampler thread failed (start-only map)\n",
                   pipe_id);
    }

    printf("[mi_record:%d] record_start -> %s (pending IDR)\n", pipe_id, dir);
    return (sxr_record_t)r;

rollback:
    /* Stop writer if it was started (only reachable for sink-open failure
     * [q not init] or writer-create failure [writer_running already false]). */
    if (recp->writer_running) {
        pthread_mutex_lock(&recp->write_q_mutex);
        recp->write_q_exit = true;
        pthread_cond_signal(&recp->write_q_cond);
        pthread_mutex_unlock(&recp->write_q_mutex);
        pthread_join(recp->writer_thread, NULL);
        recp->writer_running = false;
    }
    if (q_inited) {
        pthread_mutex_destroy(&recp->write_q_mutex);
        pthread_cond_destroy(&recp->write_q_cond);
    }
    for (int ch = 0; ch < SXR_RECORDER_SNR_MAX; ch++) {
        if (r->cam[ch].fmp4)      { sxr_fmp4_close_impl(r->cam[ch].fmp4); r->cam[ch].fmp4 = NULL; }
        if (r->cam[ch].metainfo)  { fclose_fsync(r->cam[ch].metainfo); r->cam[ch].metainfo = NULL; }
    }
    if (r->audio.hal_handle) { if (mi_hal()) mi_hal()->audio_stop(r->audio.hal_handle); r->audio.hal_handle = NULL; }
    if (r->audio.q_running) {
        pthread_mutex_lock(&r->audio.q_mutex);
        r->audio.q_exit = true;
        pthread_cond_signal(&r->audio.q_cond);
        pthread_mutex_unlock(&r->audio.q_mutex);
        pthread_join(r->audio.q_thread, NULL);
        r->audio.q_running = false;
        pthread_mutex_destroy(&r->audio.q_mutex);
        pthread_cond_destroy(&r->audio.q_cond);
    }
    if (r->audio.fmp4)       { sxr_fmp4_close_impl(r->audio.fmp4); r->audio.fmp4 = NULL; }
    if (r->audio.metainfo)   { fclose_fsync(r->audio.metainfo); r->audio.metainfo = NULL; }
    if (r->utc_fp)           { fclose_fsync(r->utc_fp); r->utc_fp = NULL; }
    if (r->sensors) sxr_sensors_switch_output_dir_impl(r->sensors, NULL);
    delete r;
    return NULL;
}

/* ========================================================================
 * sxr_record_stop_impl  (port of mi_pipeline.cpp:2149)
 *
 * Atomic withdraw -> drain barrier (HAL set_main_frame_cb NULL blocks until
 * the main-grab thread observes the NULL sink + finishes in-flight cb) ->
 * drain write_q -> stop writer -> close sinks (moov backpatch) -> sensors
 * back to cache-only -> stats.
 * ======================================================================== */

int sxr_record_stop_impl(sxr_record_t rec, SxrRecordStats *stats)
{
    struct SxrRecord *r = (struct SxrRecord *)rec;
    if (!r) return -1;
    struct RecPipeline *recp = recpipeline_get(r->pipe_id);
    if (!recp) return -1;

    /* 1) withdraw: STOPPING + nullify current_record */
    __atomic_store_n(&r->state, REC_STOPPING, __ATOMIC_RELEASE);
    __atomic_store_n(&recp->current_record, NULL, __ATOMIC_RELEASE);

    /* 1a) sensors back to cache-only NOW (was step 5 at the tail of stop).
     * 立刻关闭 accel/gyro/mag.csv，避免 barrier/drain/moov 回写期间
     * (最坏 1.5s+2s+SD 慢写) 传感器继续追加导致 CSV 尾部比视频末帧
     * 多出数秒。视频末帧 pts 是曝光结束时间，比此刻早一个 VENC 管道
     * 延迟(~0.5s)，而传感器采样持续到本调用为止，故 CSV 末 ts 仍
     * 必然晚于视频末帧 pts，后处理截断对齐不受影响。 */
    if (r->sensors) sxr_sensors_switch_output_dir_impl(r->sensors, NULL);

    /* 1b) stop mic capture first: join HAL audio thread so no audio_cb runs
     *     while we close the audio sink below. */
    {
        const sxr_hal_api_t *halA = mi_hal();
        if (r->audio.hal_handle && halA && halA->audio_stop) {
            halA->audio_stop(r->audio.hal_handle);
            r->audio.hal_handle = NULL;
        }
    }

    /* 1c) stop the async audio writer: the capture thread is already joined
     *     (1b) so no more chunks can be enqueued; signal exit + join — the
     *     writer drains every queued PCM chunk to SD before returning, so the
     *     tail of the recording is not lost. Must precede the audio sink
     *     close (step 4, moov backpatch). */
    if (r->audio.q_running) {
        pthread_mutex_lock(&r->audio.q_mutex);
        r->audio.q_exit = true;
        pthread_cond_signal(&r->audio.q_cond);
        pthread_mutex_unlock(&r->audio.q_mutex);
        pthread_join(r->audio.q_thread, NULL);
        r->audio.q_running = false;
        if (r->audio.q_dropped)
            printf("[mi_record:%d] audio chunks dropped (queue overflow): %u\n",
                   r->pipe_id, r->audio.q_dropped);
        pthread_mutex_destroy(&r->audio.q_mutex);
        pthread_cond_destroy(&r->audio.q_cond);
    }

    /* 2) drain barrier: unregister write_cb per enabled main channel. The HAL
     *    set_main_frame_cb(NULL) blocks until the main-grab thread has observed
     *    the NULL sink and finished any in-flight callback (<= ~1.5s). Replaces
     *    the old main_drained spin. After this, no more write_cb fires. */
    const sxr_hal_api_t *hal = mi_hal();
    if (hal && hal->set_main_frame_cb) {
        for (int ch = 0; ch < SXR_RECORDER_SNR_MAX; ch++) {
            if (!(r->enabled_mask & (1 << ch))) continue;
            hal->set_main_frame_cb(r->pipe_id, ch, NULL, NULL);
        }
    }

    /* 2b) writer queue drain: wait for the writer to consume all queued frames
     *     + finish final I/O (<= ~2s timeout). */
    for (int wait_ms = 0; wait_ms < 2000; wait_ms += 50) {
        int qcnt = 0, busy = 0;
        pthread_mutex_lock(&recp->write_q_mutex);
        qcnt = (int)recp->count;
        busy = recp->write_q_busy ? 1 : 0;
        pthread_mutex_unlock(&recp->write_q_mutex);
        if (qcnt == 0 && !busy) break;
        usleep(50 * 1000);
    }
    r->stats.dropped_frames = recp->dropped;

    /* 2c) free any pending-IDR candidates never committed (record stopped
     *     while still REC_PENDING_IDR, before the aligned start was chosen). */
    pending_idr_free_all(r);

    /* 3) stop writer thread (signal exit + join) */
    pthread_mutex_lock(&recp->write_q_mutex);
    recp->write_q_exit = true;
    pthread_cond_signal(&recp->write_q_cond);
    pthread_mutex_unlock(&recp->write_q_mutex);
    if (recp->writer_running) {
        pthread_join(recp->writer_thread, NULL);
        recp->writer_running = false;
    }
    pthread_mutex_destroy(&recp->write_q_mutex);
    pthread_cond_destroy(&recp->write_q_cond);

    /* 3b) stop the periodic UTC-mapping sampler: signal + join, then close.
     *     Joined before 'delete r' so the thread never touches freed memory. */
    if (r->utc_thread_running) {
        __atomic_store_n(&r->utc_stop, 1, __ATOMIC_RELEASE);
        pthread_join(r->utc_thread, NULL);
        r->utc_thread_running = 0;
    }
    if (r->utc_fp) { fclose_fsync(r->utc_fp); r->utc_fp = NULL; }

    /* 4) close sinks (moov backpatch) */
    for (int ch = 0; ch < SXR_RECORDER_SNR_MAX; ch++) {
        if (!(r->enabled_mask & (1 << ch))) continue;
        if (r->cam[ch].fmp4) {
            if (sxr_fmp4_close_impl(r->cam[ch].fmp4) != 0 && r->error == 0)
                r->error = EIO;   /* close/moov-backpatch failure also recorded */
            r->cam[ch].fmp4 = NULL;
        }
        if (r->cam[ch].metainfo) {
            fclose_fsync(r->cam[ch].metainfo);
            r->cam[ch].metainfo = NULL;
        }
    }
    if (r->audio.fmp4)     { if (sxr_fmp4_close_impl(r->audio.fmp4) != 0 && r->error == 0) r->error = EIO; r->audio.fmp4 = NULL; }
    if (r->audio.metainfo) { fclose_fsync(r->audio.metainfo); r->audio.metainfo = NULL; }

    /* 6) stats summary */
    if (stats) memcpy(stats, &r->stats, sizeof(*stats));

    printf("[mi_record:%d] record_stop: %s (err=%d)\n", r->pipe_id, r->dir, r->error);
    int err = r->error;
    delete r;
    return err ? -1 : 0;
}
