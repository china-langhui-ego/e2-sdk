// SPDX-License-Identifier: Apache-2.0
// Serialize sxr_hal calibration structs to the refcode JSON formats.
#include "calib_json.h"
#include <cstdio>
#include <cstring>
#include <string>

static FILE *openw(const char *dir, const char *fname, std::string &path) {
    path = std::string(dir) + "/" + fname;
    return fopen(path.c_str(), "w");
}

extern "C" int calib_json_write_camera(const sxr_hal_api_t *hal, int eye, const char *cam_dir) {
    if (!hal || !hal->calib_get_camera || !cam_dir) return -1;
    sxr_hal_camera_calib_t c{};
    if (hal->calib_get_camera(eye, &c) != 0) return -1;
    std::string path; FILE *f = openw(cam_dir, "camera_params.json", path);
    if (!f) return -1;
    fprintf(f, "{\n");
    fprintf(f, "  \"group\": \"tracking\",\n");
    fprintf(f, "  \"cameras\": [\n    {\n");
    fprintf(f, "      \"eye\": \"%s\",\n", c.eye_name);
    fprintf(f, "      \"width\": %d,\n", c.width);
    fprintf(f, "      \"height\": %d,\n", c.height);
    fprintf(f, "      \"intrinsics\": {\n");
    fprintf(f, "        \"focalX\": %.9g,\n", c.focal_x);
    fprintf(f, "        \"focalY\": %.9g,\n", c.focal_y);
    fprintf(f, "        \"centerX\": %.9g,\n", c.center_x);
    fprintf(f, "        \"centerY\": %.9g,\n", c.center_y);
    fprintf(f, "        \"radialDistortion\": [%.9g, %.9g, %.9g, %.9g, %.9g, %.9g, %.9g, %.9g]\n",
            c.radial_distortion[0], c.radial_distortion[1], c.radial_distortion[2], c.radial_distortion[3],
            c.radial_distortion[4], c.radial_distortion[5], c.radial_distortion[6], c.radial_distortion[7]);
    fprintf(f, "      },\n");
    fprintf(f, "      \"extrinsics\": {\n");
    fprintf(f, "        \"position\": [%.9g, %.9g, %.9g],\n",
            c.position[0], c.position[1], c.position[2]);
    fprintf(f, "        \"rotation\": [%.9g, %.9g, %.9g, %.9g]\n",
            c.rotation[0], c.rotation[1], c.rotation[2], c.rotation[3]);
    fprintf(f, "      }\n");
    fprintf(f, "    }\n  ]\n}\n");
    fclose(f);
    return 0;
}

extern "C" int calib_json_write_imu(const sxr_hal_api_t *hal, const char *sensors_dir) {
    if (!hal || !hal->calib_get_imu || !hal->uuid_get || !sensors_dir) return -1;
    sxr_hal_imu_calib_t m{};
    if (hal->calib_get_imu(&m) != 0) return -1;
    char uid[64] = {0};
    if (hal->uuid_get(uid, sizeof(uid)) != 0) uid[0] = '\0';

    std::string path; FILE *f = openw(sensors_dir, "imu_calibration.json", path);
    if (!f) return -1;
    fprintf(f, "{\n");
    fprintf(f, "  \"device_uid\": \"%s\",\n", uid);
    fprintf(f, "  \"imu\": {\n");
    fprintf(f, "    \"imu_id\": %d,\n", m.imu_id);
    fprintf(f, "    \"is_primary\": %s,\n", m.is_primary ? "true" : "false");
    fprintf(f, "    \"bias\": {\n");
    fprintf(f, "      \"accelerometer_mps2\": [%.9g, %.9g, %.9g],\n", m.accel_bias[0], m.accel_bias[1], m.accel_bias[2]);
    fprintf(f, "      \"gyroscope_rads\": [%.9g, %.9g, %.9g]\n", m.gyro_bias[0], m.gyro_bias[1], m.gyro_bias[2]);
    fprintf(f, "    },\n");
    fprintf(f, "    \"scale_factor\": {\n");
    fprintf(f, "      \"accelerometer\": [%.9g, %.9g, %.9g],\n", m.accel_scale[0], m.accel_scale[1], m.accel_scale[2]);
    fprintf(f, "      \"gyroscope\": [%.9g, %.9g, %.9g]\n", m.gyro_scale[0], m.gyro_scale[1], m.gyro_scale[2]);
    fprintf(f, "    },\n");
    fprintf(f, "    \"nonorthogonality\": {\n");
    fprintf(f, "      \"accelerometer\": [%.9g, %.9g, %.9g],\n", m.accel_nonorth[0], m.accel_nonorth[1], m.accel_nonorth[2]);
    fprintf(f, "      \"gyroscope\": [%.9g, %.9g, %.9g]\n", m.gyro_nonorth[0], m.gyro_nonorth[1], m.gyro_nonorth[2]);
    fprintf(f, "    },\n");
    fprintf(f, "    \"time_alignment_s\": {\n");
    fprintf(f, "      \"imu_to_pose\": %.9g,\n", m.imu_to_pose_delta);
    fprintf(f, "      \"cameras\": {");
    for (int i = 0; i < m.cam_time_align_count; i++) {
        fprintf(f, "%s\"%s\": %.9g", i ? ", " : " ", m.cam_time_align[i].name, m.cam_time_align[i].delta_sec);
    }
    fprintf(f, "%s},\n", m.cam_time_align_count ? " " : "");
    fprintf(f, "      \"accel\": %.9g\n", m.accel_delta);
    fprintf(f, "    }\n");
    fprintf(f, "  },\n");
    fprintf(f, "  \"noise\": {\n");
    fprintf(f, "    \"accel_noise_std_mps2\": [%.9g, %.9g, %.9g],\n", m.accel_noise_std[0], m.accel_noise_std[1], m.accel_noise_std[2]);
    fprintf(f, "    \"gyro_noise_std_rads\": [%.9g, %.9g, %.9g],\n", m.gyro_noise_std[0], m.gyro_noise_std[1], m.gyro_noise_std[2]);
    fprintf(f, "    \"accel_bias_std_mps2\": [%.9g, %.9g, %.9g],\n", m.accel_bias_std[0], m.accel_bias_std[1], m.accel_bias_std[2]);
    fprintf(f, "    \"gyro_bias_std_rads\": [%.9g, %.9g, %.9g]\n", m.gyro_bias_std[0], m.gyro_bias_std[1], m.gyro_bias_std[2]);
    fprintf(f, "  }\n}\n");
    fclose(f);
    return 0;
}

extern "C" int calib_json_write_id_txt(const sxr_hal_api_t *hal, const char *dir) {
    if (!hal || !hal->uuid_get || !dir) return -1;
    char uid[64] = {0};
    if (hal->uuid_get(uid, sizeof(uid)) != 0) return -1;
    std::string path; FILE *f = openw(dir, "id.txt", path);
    if (!f) return -1;
    fprintf(f, "%s\n", uid);
    fclose(f);
    return 0;
}
