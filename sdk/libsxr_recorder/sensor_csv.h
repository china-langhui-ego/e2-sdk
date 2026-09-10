/* SPDX-License-Identifier: Apache-2.0 */
/* Save-half of the sensor recorder: owns the CSV FILE*, writes samples
 * delivered by the HAL sample callback, handles dir switching. */
#ifndef _SENSOR_CSV_H_
#define _SENSOR_CSV_H_
#include "sxr_recorder.h"
#ifdef __cplusplus
extern "C" {
#endif

/* These replace the sxr_sensors_*_impl that lived in mi_pipeline.cpp. */
sxr_sensors_t sxr_sensors_init_impl(unsigned int accel_hz, unsigned int gyro_hz,
                                  unsigned int mag_hz, const char *output_dir,
                                  volatile unsigned char *exit_flag);
int sxr_sensors_exit_impl(sxr_sensors_t s);
int sxr_sensors_get_latest_impl(sxr_sensors_t s, double *ax, double *ay, double *az,
                               double *gx, double *gy, double *gz);
int sxr_sensors_get_latest_ex_impl(sxr_sensors_t s, double accel[3], double gyro[3],
                                    double mag[3], int64_t *ts_ns);
int sxr_sensors_switch_output_dir_impl(sxr_sensors_t s, const char *dir);
#ifdef __cplusplus
}
#endif
#endif
