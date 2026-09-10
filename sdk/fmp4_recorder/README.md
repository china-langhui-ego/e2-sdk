# FMP4 Recorder (`prog_fmp4_recorder`)

基于 `libsxr_recorder` 的双目 HEVC 录制示例应用。运行期 dlopen 加载录制库，按段录制双目视频 + IMU/磁力计 + 音频数据集。

## 功能

| 特性 | 说明 |
|------|------|
| **双目录制** | Camera0（左目）+ Camera1（右目），首帧曝光对齐 |
| **编码封装** | HEVC (H.265) fMP4，IP-only，GOP=30，CBR 4 Mbps |
| **多段录制** | 一次运行录 N 段，每段独立 `clip_NNN/` 目录 |
| **传感器** | 加速度/陀螺 800 Hz、磁力计 200 Hz，CSV 输出 |
| **音频** | 16 kHz / 2ch / S16（无麦克风时自动跳过） |
| **IMU 读数** | 录制后用 `imu_get_latest` 读 9 轴最新值（示例） |

> 录制参数（分辨率/码率/传感器速率/IQ 等）为 `libsxr_recorder` 内部默认，本应用不暴露 CLI。当前默认：1600×1200、4 Mbps、IQ = `sc235hgs_api.bin`。

## 编译

**一键编译**（同时构建 `libsxr_recorder.so` + `prog_fmp4_recorder`）：

```bash
./build.sh
# 产物：build/libsxr_recorder/libsxr_recorder.so  +  build/fmp4_recorder/prog_fmp4_recorder
```

或手动 CMake（共用 `libsxr_recorder` 的工具链，host 上编译，无需 Docker）：

```bash
mkdir -p build/fmp4_recorder && cd build/fmp4_recorder
cmake -DCMAKE_TOOLCHAIN_FILE=../../cmake/arm-uclibc.cmake ../../fmp4_recorder
make
# 产物：build/fmp4_recorder/prog_fmp4_recorder
```

`libsxr_recorder.so` 不被本应用链接，运行期由库内 inline loader dlopen；`build.sh` 已一并构建它。部署见 [`../../docs/E2_HMD_SDK_zh.html`](../../docs/E2_HMD_SDK_zh.html)（Part I · SDK API 使用流程）。

## 使用

设备端运行（二进制 + `libsxr_recorder.so` 均部署到 `/customer/sd/`）：

```bash
# 录制 2 段 × 4 秒（默认）到 /customer/sd/fmp4_recorder
LD_LIBRARY_PATH=/customer/sd /customer/sd/prog_fmp4_recorder

# 自定义：3 段 × 10 秒，输出到指定目录
LD_LIBRARY_PATH=/customer/sd /customer/sd/prog_fmp4_recorder \
    --output /customer/sd/myrun --segs 3 --segdur 10
```

录制流程：`record_init` → 每段 `record_start`（异步，等 `STARTED` 事件）→ 计时 `segdur` 秒 → `record_stop` → 下一段；全部段结束后打印 10 Hz × 20 次 IMU 最新读数，再 `record_deinit`。

## 参数

| 参数 | 默认 | 说明 |
|------|------|------|
| `--output DIR` | `/customer/sd/fmp4_recorder` | 输出根目录（每段在其下 `clip_NNN/`） |
| `--segs N` | `2` | 录制段数 |
| `--segdur S` | `4` | 每段秒数 |
| `-h` / `--help` | — | 打印用法 |

## 输出

每段一个独立片段目录：

```
<output>/
├── clip_001/
│   ├── Camera0/{video.mp4, metainfo.csv, camera_params.json}   # 左目
│   ├── Camera1/{video.mp4, metainfo.csv, camera_params.json}   # 右目
│   ├── Sensors/{accel.csv, gyro.csv, mag.csv, imu_calibration.json}
│   ├── Audio/{audio.mp4, metainfo.csv}     # 无麦克风则无此目录
│   ├── sensor_time_to_utc.csv              # MONOTONIC_RAW → UTC 映射
│   └── id.txt                              # 硬件 UUID
├── clip_002/
└── ...
```

各文件列定义、单位、时基、标定参数与坐标系意义详见 [`../../docs/E2_HMD_SDK_zh.html`](../../docs/E2_HMD_SDK_zh.html)（Part II · 数据集结构与格式）· [English](../../docs/E2_HMD_SDK_en.html)。

## 限制

- **输出写 SD 卡**（`/customer/sd/...`），绝不要写 `/tmp`（tmpfs，写大文件会 OOM）。
- **跨 MI 应用需 reboot**：本应用可反复运行（teardown 干净）；但与其它占用 MI 的应用互相切换后需 `adb reboot`。
- **单时刻一个录制段**：多段由 `start→stop` 串行实现。
- **录制参数内部固定**：改分辨率/码率/传感器速率等需重编 `libsxr_recorder`（v2 无运行时配置入口）。
