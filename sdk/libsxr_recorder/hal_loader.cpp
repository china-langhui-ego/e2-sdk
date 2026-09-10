// SPDX-License-Identifier: Apache-2.0
#include "hal_loader.h"
#include <dlfcn.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>

static sxr_hal_api_t g_hal;
static const sxr_hal_api_t *g_hal_p = nullptr;
static pthread_once_t g_once = PTHREAD_ONCE_INIT;

static void hal_load_once(void) {
    /* RTLD_GLOBAL (not LOCAL): libsxr_hal.so pulls in the SigmaStar MI libs
     * (libmi_*.so), which were global-scope when the app linked them directly
     * (Plan 1/2a). Under RTLD_LOCAL dlopen their inter-library symbol binding
     * fails ("can't resolve symbol"). GLOBAL restores that scope; safe here
     * because the app + libsxr_recorder are MI-free, so no symbol clash. */
    void *h = dlopen("libsxr_hal.so", RTLD_NOW | RTLD_GLOBAL);
    if (!h) {
        // fall back to a couple of common deploy locations
        h = dlopen("/customer/sd/libsxr_hal.so", RTLD_NOW | RTLD_GLOBAL);
        if (!h) h = dlopen("/customer/sample_code/lib/libsxr_hal.so", RTLD_NOW | RTLD_GLOBAL);
    }
    if (!h) { fprintf(stderr, "[hal_loader] dlopen libsxr_hal.so failed: %s\n", dlerror()); return; }
    int (*get_api)(sxr_hal_api_t *, int) =
        (int (*)(sxr_hal_api_t *, int))dlsym(h, "sxr_hal_get_api");
    if (!get_api) { fprintf(stderr, "[hal_loader] dlsym sxr_hal_get_api failed: %s\n", dlerror()); return; }
    if (get_api(&g_hal, (int)sizeof(g_hal)) != 0) {
        fprintf(stderr, "[hal_loader] sxr_hal_get_api ABI mismatch\n"); return;
    }
    g_hal_p = &g_hal;
}

extern "C" const sxr_hal_api_t *mi_hal(void) {
    pthread_once(&g_once, hal_load_once);
    return g_hal_p;
}
