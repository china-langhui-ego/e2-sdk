/* SPDX-License-Identifier: Apache-2.0 */
#ifndef _CALIB_JSON_H_
#define _CALIB_JSON_H_
#include "sxr_hal.h"
#ifdef __cplusplus
extern "C" {
#endif
/* Write Camera<eye>/camera_params.json (single-element cameras array,
 * this eye only). Returns 0 on success, -1 on failure (non-fatal to recording). */
int calib_json_write_camera(const sxr_hal_api_t *hal, int eye, const char *cam_dir);
/* Write Sensors/imu_calibration.json (device_uid = hardware UUID). */
int calib_json_write_imu(const sxr_hal_api_t *hal, const char *sensors_dir);
/* Write <dir>/id.txt with the hardware UUID. */
int calib_json_write_id_txt(const sxr_hal_api_t *hal, const char *dir);
#ifdef __cplusplus
}
#endif
#endif
