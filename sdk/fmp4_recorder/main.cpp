#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <math.h>
#include "sxr_recorder.h"

static volatile unsigned char g_bExit = 0;
static volatile unsigned char g_recStarted = 0;
static volatile unsigned char g_recError = 0;

static void sig_handler(int sig) { (void)sig; g_bExit = 1; }

static void preview_cb(int pipe_id, int ch, const uint8_t *data, size_t size,
                        int64_t pts_us, int is_sync, const uint8_t *csd, size_t csd_size,
                        void *ud)
{
    (void)pipe_id; (void)data; (void)size; (void)is_sync; (void)csd; (void)csd_size; (void)ud;
    static int cnt[2] = {0, 0};
    int cam = ch / 2;
    if (cam >= 0 && cam < 2 && cnt[cam] < 3) {
        printf("[preview] ch%d pts=%lld\n", ch, (long long)pts_us);
        cnt[cam]++;
    }
}

static void evt_cb(sxr_rec_evt_t evt, int err, void *ud)
{
    (void)ud;
    const char *name = (evt == SXR_REC_EVT_STARTED)     ? "STARTED"
                     : (evt == SXR_REC_EVT_IDR_TIMEOUT) ? "IDR_TIMEOUT"
                                                        : "WRITE_ERROR";
    printf("[rec] %s err=%d\n", name, err);
    if (evt == SXR_REC_EVT_STARTED) g_recStarted = 1;
    else g_recError = 1;
}

static void usage(const char *prog)
{
    printf("Usage: %s [--output DIR] [--segs N] [--segdur S]\n", prog);
    printf("  --output DIR   output root (default /customer/sd/fmp4_recorder)\n");
    printf("  --segs N       segments (default 2)\n");
    printf("  --segdur S     seconds per segment (default 4)\n");
}

int main(int argc, char **argv)
{
    setvbuf(stdout, NULL, _IONBF, 0);

    const char *output = "/customer/sd/fmp4_recorder";
    int segs = 2, segdur = 4;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--output") && i+1 < argc) output = argv[++i];
        else if (!strcmp(argv[i], "--segs") && i+1 < argc) segs = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--segdur") && i+1 < argc) segdur = atoi(argv[++i]);
        else if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) { usage(argv[0]); return 0; }
    }

    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);

    const sxr_recorder_api_t *REC = sxr_recorder_api();
    if (!REC) { fprintf(stderr, "load libsxr_recorder.so failed\n"); return -1; }

    if (REC->record_init(preview_cb, NULL) != 0) { fprintf(stderr, "record_init failed\n"); return -1; }

    sleep(2);
    printf("[main] recording %d segment(s) x %ds\n", segs, segdur);

    for (int seg = 1; seg <= segs && !g_bExit; seg++) {
        char dir[640];
        snprintf(dir, sizeof(dir), "%s/clip_%03d", output, seg);
        g_recStarted = 0; g_recError = 0;
        if (REC->record_start(dir, evt_cb, NULL) != 0) { fprintf(stderr, "record_start seg%d failed\n", seg); break; }
        for (int i = 0; i < 100 && !g_recStarted && !g_recError && !g_bExit; i++) usleep(20*1000);
        time_t t0 = time(NULL);
        while (!g_bExit && !g_recError && (time(NULL) - t0) < segdur) usleep(100*1000);
        REC->record_stop();
        printf("[main] seg%d done\n", seg);
    }

    /* 10 Hz IMU */
    printf("[main] IMU 10Hz x 20 samples:\n");
    for (int i = 0; i < 20 && !g_bExit; i++) {
        sxr_imu_sample_t s;
        if (REC->imu_get_latest(&s) == 0) {
            double a = sqrt(s.accel[0]*s.accel[0] + s.accel[1]*s.accel[1] + s.accel[2]*s.accel[2]);
            double g = sqrt(s.gyro[0]*s.gyro[0] + s.gyro[1]*s.gyro[1] + s.gyro[2]*s.gyro[2]);
            double m = sqrt(s.mag[0]*s.mag[0] + s.mag[1]*s.mag[1] + s.mag[2]*s.mag[2]);
            printf("  |a|=%.2f m/s^2  |g|=%.3f rad/s  |m|=%.1f uT  ts=%lld\n",
                   a, g, m, (long long)s.ts_ns);
        } else {
            printf("  (no data)\n");
        }
        usleep(100*1000);
    }

    REC->record_deinit();
    printf("[main] done\n");
    return 0;
}
