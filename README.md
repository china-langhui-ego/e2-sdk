[English](README_en.md) · **简体中文** ｜ [E6 XR 数采头显 SDK](https://github.com/china-langhui-ego/e6-sdk)

# E2 双目数据采集头环 · 开发者资料

E2 是面向具身智能 / Egocentric 数据采集的**双目头环**：Linux（uClibc、ARM 32bit）+ SigmaStar 风格 MI 多媒体管线，双路 HEVC fMP4 与 IMU / 磁力计 / 音频同步落盘，时间戳统一到 `CLOCK_MONOTONIC_RAW` 纳秒域。

- SDK 版本：**v1.1**
- 公开文档全文：[`docs/E2_HMD_SDK_zh.html`](docs/E2_HMD_SDK_zh.html) · [English](docs/E2_HMD_SDK_en.html)
- 产品介绍：[`docs/E2-product-introduction-v2.0.pptx`](docs/E2-product-introduction-v2.0.pptx)

---

## 目录结构

```
e2/
├── README.md / README_en.md
├── sdk/                                  录制 SDK 源码（纯文本，可直接编译）
│   ├── build.sh                             一键交叉编译（libsxr_recorder.so + prog_fmp4_recorder）
│   ├── cmake/arm-uclibc.cmake                工具链描述（★ 需改成本地工具链路径）
│   ├── libsxr_recorder/                      录制库源码
│   │   ├── sxr_recorder.c                     对外 API（v2 vtable，5 槽）
│   │   ├── hal_loader.{h,cpp}                  内联 dlopen 加载器（加载 libsxr_hal.so）
│   │   ├── mi_record.{h,cpp}                   MI 管线取流 / 编码封装
│   │   ├── FMP4Writer.{h,cpp}                  fragmented MP4 写入器
│   │   ├── sensor_csv.{h,cpp}                  accel / gyro / mag / metainfo CSV
│   │   ├── calib_json.{h,cpp}                  camera_params.json / imu_calibration.json
│   │   └── include/{sxr_recorder.h,sxr_hal.h}    公开头文件
│   └── fmp4_recorder/                          参考应用 prog_fmp4_recorder
│       ├── main.cpp · CMakeLists.txt · Makefile
│       └── README.md                            ★ 编译 / 部署 / 运行说明
└── docs/
    ├── E2_HMD_SDK_zh.html / E2_HMD_SDK_en.html   API 使用流程 + 数据集结构与格式
    └── E2-product-introduction-v2.0.pptx
```

> 官方整包 `E2-HMD-SDK-v1.1.tar.gz`（122 MB，内含 387 MB 交叉工具链）体积超限，见下方[云端下载](#云端下载)。

---

## 架构：两个可 dlopen 的库

| | `libsxr_recorder.so` | `libsxr_hal.so` |
| --- | --- | --- |
| 定位 | 录制会话（公开 API 主入口） | 硬件抽象：MI 管线 / 传感器 / 标定 / 音频 |
| 导出 | `sxr_recorder_get_api()` → **v2** vtable（5 槽） | `sxr_hal_get_api()` → **v3** vtable |
| MI 依赖 | 无（MI-free，纯 C/C++ 运行时） | 链接 `libmi_*` + CUS3A / ISPALGO，自带 RPATH `/config/lib` |
| 谁来加载 | 应用 dlopen（`sxr_recorder_api()` 内联加载器） | recorder 内部 dlopen，**必须 `RTLD_GLOBAL`** |

`libsxr_hal.so` 已随 ROM 内置，开发者**无需**手动 push；应用侧只需 `LD_LIBRARY_PATH=/customer/sd`。

### 最小工作流（单例会话）

```
① record_init(preview_cb, userdata)   建管线（默认配置）、起 IMU/磁线程   → 0 成功 / −1 失败
② record_start(dir)                   异步开始一段，强制 IDR             → 0 已入队 / −1 未 init 或已在录
③ 录制中…                              落盘 Camera{0,1}/ + Sensors/ + Audio/
④ record_stop()                       关 video.mp4（回写 moov）+ 收尾本段  → 0 成功 / −1 无活动段
⑤ record_deinit()                     finalize 残余段、停线程、拆管线
   imu_get_latest(&sample)             O(1) 读最新 IMU/磁力缓存            → 0 有数据 / −1 无数据
```

②~④ 可循环 N 段，每段独立 `clip_NNN/`，无需重启管线。`record_start` **返回 0 只代表已入队**，真正开始落盘要等 `SXR_REC_EVT_STARTED` 回调。

---

## 编译

```bash
cd sdk
./build.sh            # 产物自动 strip
#   build/libsxr_recorder/libsxr_recorder.so
#   build/fmp4_recorder/prog_fmp4_recorder
```

前置条件：CMake ≥ 3.10 + ARM uClibc 交叉工具链
`arm-linaro-linux-uclibcgnueabihf-9.1.0`（**未随本仓库收录**，从云端「设备 SDK」下载）。

工具链路径写在 [`sdk/cmake/arm-uclibc.cmake`](sdk/cmake/arm-uclibc.cmake) 的 `set(TOOLCHAIN_BASE ...)`，默认指向原厂内网构建机，**本地编译前必须改成自己的路径**。

## 部署与运行

```bash
adb push build/fmp4_recorder/prog_fmp4_recorder /customer/sd/
adb push build/libsxr_recorder/libsxr_recorder.so /customer/sd/   # 若使用自编译 recorder
adb shell "chmod +x /customer/sd/prog_fmp4_recorder"

adb shell 'cd /customer/sd && LD_LIBRARY_PATH=/customer/sd ./prog_fmp4_recorder \
           --output /customer/sd/out --segs 2 --segdur 10'
```

| 参数 | 含义 |
| --- | --- |
| `--output DIR` | 数据集输出目录（**务必写在 SD 卡 `/customer/sd/`**） |
| `--segs N` | 录制片段数（每段一个 `clip_NNN/`） |
| `--segdur S` | 每段时长（秒） |

注意事项（详见 [`sdk/fmp4_recorder/README.md`](sdk/fmp4_recorder/README.md)）：

- `/tmp` 是 tmpfs，写大文件会 OOM —— 输出目录不要指向 `/tmp`。
- 同一应用内可反复 `init/start/stop/deinit`；**切换到其它 MI 应用（如参考 demo `prog_pipeline`）需 `adb reboot`**，否则对方 VENC 通道未释放会 `MI_VENC_CreateChn` 失败（`0xa0022004`）。
- 录制参数在 `libsxr_recorder` 内固定；要改码率 / GOP / 预览 / 音频 / 时钟，需绕过 recorder 直接调用 `libsxr_hal` 的 `pipeline_create_venc(SxrPipelineConfig*)`。

---

## 数据集格式

所有流（视频曝光、accel/gyro/mag、音频）**统一 `CLOCK_MONOTONIC_RAW` 纳秒域**，自上电起单调递增、不受 NTP 影响，因此天然互相可以对齐；**这不是墙钟时间**，不能 `date -d @ts`。换算 UTC 请用侧车文件 `sensor_time_to_utc.csv`（约 1 Hz 周期采样，可插值跟踪 NTP step）。

```
<output>/                                # 默认 /customer/sd/fmp4_recorder
└── clip_001/                            # NNN 从 001 起，三位补零
    ├── id.txt                           # 硬件 UUID
    ├── sensor_time_to_utc.csv           # MONOTONIC_RAW → UTC 映射
    ├── Camera0/  Camera1/               # 左眼 / 右眼
    │   ├── video.mp4                    # HEVC fMP4
    │   ├── metainfo.csv                 # 每帧曝光元数据
    │   └── camera_params.json           # 内参 + 外参
    ├── Sensors/
    │   ├── accel.csv  gyro.csv  mag.csv
    │   └── imu_calibration.json
    └── Audio/                           # 可选：无麦克风则整个目录不生成
        ├── audio.mp4                    # PCM fMP4，S16_LE 交错 16 kHz 2ch
        └── metainfo.csv                 # pts_us, ts_ns, samples
```

| 默认参数 | 值 |
| --- | --- |
| 分辨率 | 自动探测：SC235HGS = 1600×1200、SC233HGS = 1920×1200 |
| 主码流 | HEVC (H.265) Main，**IP-only 无 B 帧**，GOP=30，CBR 4 Mbps |
| 子码流（仅预览，不入数据集） | 主码流 / 4（400×300 / 480×300），2 Mbps，15 fps；VENC ch1=Cam0 / ch3=Cam1 |
| 采样率 | accel 800 Hz · gyro 800 Hz · mag 200 Hz |
| 音频 | 16 kHz、2 ch、S16_LE（SDK 无音频编码器，直存 PCM fMP4） |
| IQ | 按 sensor 型号自动选 `sc235hgs_api.bin` / `sc233hgs_api.bin` |
| 时钟 | ISP 216 MHz、SCL 288 MHz（内部经 `/proc` 设置） |

字段级定义、坐标系意义与对齐换算见 [`docs/E2_HMD_SDK_zh.html`](docs/E2_HMD_SDK_zh.html) Part II。

---

## 云端下载

| 分类 | Google Drive | 百度网盘（提取码） |
| --- | --- | --- |
| 设备 SDK（含交叉工具链） | [Drive](https://drive.google.com/drive/folders/1PO7V0ip8HJR5GmlG18mPJAG_f-jUrqwd?usp=drive_link) | [pan.baidu.com/s/1jxtSpGT4zHtHYtWLeg7hNQ](https://pan.baidu.com/s/1jxtSpGT4zHtHYtWLeg7hNQ) `9mqq` |
| 头环 ROM（`SgsUpgradeSD.bin`） | [Drive](https://drive.google.com/drive/folders/1u9Pulibxpo12T_amFZ8u82-AhPMJPk7b?usp=drive_link) | [pan.baidu.com/s/1z69drlVivOh68w6jMt10sA](https://pan.baidu.com/s/1z69drlVivOh68w6jMt10sA) `fw6r` |
| 手机端 APP 与 SDK | [Drive](https://drive.google.com/drive/folders/1bI0_epjY9s8yilUQ8GBD48v23xIB0WzA?usp=drive_link) | [pan.baidu.com/s/1bkdWndppacK0lsErQPr2FQ](https://pan.baidu.com/s/1bkdWndppacK0lsErQPr2FQ) `akh3` |
| 其他调试环境（PotPlayer / VLC / adb-setup） | [Drive](https://drive.google.com/drive/folders/1vFnwbaoV0jHKZI4RYlprebZa4C1r45hQ?usp=drive_link) | [pan.baidu.com/s/1k1v5Zt3lnU0IQe9J_t8KGw](https://pan.baidu.com/s/1k1v5Zt3lnU0IQe9J_t8KGw) `86c8` |
| 设备使用说明（视频） | [Drive](https://drive.google.com/file/d/1RxNcjXfsZo3Ah8fLYhHO6Z3dqtN3ZYWG/view?usp=drive_link) | [pan.baidu.com/s/1-B5qp9GgTy4KmHj2MAo6vw](https://pan.baidu.com/s/1-B5qp9GgTy4KmHj2MAo6vw) `uadc` |
| 样例视频 / 样例数据集 | [Drive](https://drive.google.com/drive/folders/1a1lDFsN2bnnkSDcSnP5Nrqzk5Eb78XEP?usp=drive_link) | [pan.baidu.com/s/1APj3P7N2iwz5dIyouLma0w](https://pan.baidu.com/s/1APj3P7N2iwz5dIyouLma0w?pwd=67cs) `67cs` |

---

## 许可证

本项目采用 [MIT License](LICENSE)。

第三方组件（如 `sdk/` 下引用的开源库）遵循各自的原始许可证，详见对应目录内的说明文件。
