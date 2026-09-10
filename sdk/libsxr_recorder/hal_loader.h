/* SPDX-License-Identifier: Apache-2.0 */
#ifndef _HAL_LOADER_H_
#define _HAL_LOADER_H_
#include "sxr_hal.h"
#ifdef __cplusplus
extern "C" {
#endif
/* Returns cached HAL vtable (dlopen libsxr_hal.so on first call), or NULL. */
const sxr_hal_api_t *mi_hal(void);
#ifdef __cplusplus
}
#endif
#endif
