**English** · [简体中文](README.md) ｜ [E6 XR Headset SDK](https://github.com/china-langhui-ego/e6-sdk)

# E2 Binocular Capture Head-Mounted Device · Developer Resources

E2 is a **binocular head-mounted device** for embodied-AI / egocentric data collection: Linux (uClibc, ARM 32-bit) with a SigmaStar-style MI multimedia pipeline. It writes two synchronized HEVC fMP4 streams plus IMU / magnetometer / audio, with all timestamps in the `CLOCK_MONOTONIC_RAW` nanosecond domain.

- SDK version: **v1.1**
- Full public documentation: [`docs/E2_HMD_SDK_en.html`](docs/E2_HMD_SDK_en.html) · [中文](docs/E2_HMD_SDK_zh.html)
- Product introduction: [`docs/E2-product-introduction-v2.0.pptx`](docs/E2-product-introduction-v2.0.pptx)

---

## Repository layout

```
e2/
├── README.md / README_en.md
├── sdk/                                  Recording SDK sources (plain text, directly compilable)
│   ├── build.sh                             One-shot cross build (libsxr_recorder.so + prog_fmp4_recorder)
│   ├── cmake/arm-uclibc.cmake                Toolchain description (★ must be repointed to your local toolchain)
│   ├── libsxr_recorder/                      Recording library sources
│   │   ├── sxr_recorder.c                     Public API (v2 vtable, 5 slots)
│   │   ├── hal_loader.{h,cpp}                  Inline dlopen loader (loads libsxr_hal.so)
│   │   ├── mi_record.{h,cpp}                   MI stream capture / muxing
│   │   ├── FMP4Writer.{h,cpp}                  fragmented-MP4 writer
│   │   ├── sensor_csv.{h,cpp}                  accel / gyro / mag / metainfo CSV
│   │   ├── calib_json.{h,cpp}                  camera_params.json / imu_calibration.json
│   │   └── include/{sxr_recorder.h,sxr_hal.h}    Public headers
│   └── fmp4_recorder/                          Reference app prog_fmp4_recorder
│       ├── main.cpp · CMakeLists.txt · Makefile
│       └── README.md                            ★ Build / deploy / run notes
└── docs/
    ├── E2_HMD_SDK_en.html / E2_HMD_SDK_zh.html   API workflow + dataset structure and format
    └── E2-product-introduction-v2.0.pptx
```

> The official bundle `E2-HMD-SDK-v1.1.tar.gz` (122 MB, containing a 387 MB cross toolchain) exceeds the repository size limits — see [Cloud downloads](#cloud-downloads) below.

---

## Architecture: two dlopen-able libraries

| | `libsxr_recorder.so` | `libsxr_hal.so` |
| --- | --- | --- |
| Role | Recording session (main public entry point) | Hardware abstraction: MI pipeline / sensors / calibration / audio |
| Exports | `sxr_recorder_get_api()` → **v2** vtable (5 slots) | `sxr_hal_get_api()` → **v3** vtable |
| MI dependency | None (MI-free, plain C/C++ runtime) | Links `libmi_*` + CUS3A / ISPALGO, carries RPATH `/config/lib` |
| Loaded by | The application via dlopen (`sxr_recorder_api()` inline loader) | Internally by the recorder, **must be `RTLD_GLOBAL`** |

`libsxr_hal.so` ships built into the ROM, so developers **do not** need to push it manually; on the app side only `LD_LIBRARY_PATH=/customer/sd` is required.

### Minimal workflow (single-instance session)

```
① record_init(preview_cb, userdata)   Create the pipeline (default config), start IMU/mag threads → 0 ok / −1 failure
② record_start(dir)                   Start one segment asynchronously, force IDR            → 0 queued / −1 not init'ed or already recording
③ recording…                          Writes Camera{0,1}/ + Sensors/ + Audio/
④ record_stop()                       Close video.mp4 (patch moov) and finish the segment     → 0 ok / −1 no active segment
⑤ record_deinit()                     Finalize pending segments, stop threads, tear down pipeline
   imu_get_latest(&sample)             O(1) read of the latest IMU/mag cache                  → 0 data / −1 no data
```

Steps ②–④ can be looped for N segments, each producing its own `clip_NNN/` without restarting the pipeline. `record_start` **returning 0 only means the request was queued**; actual writing begins with the `SXR_REC_EVT_STARTED` callback.

---

## Build

```bash
cd sdk
./build.sh            # artifacts are stripped automatically
#   build/libsxr_recorder/libsxr_recorder.so
#   build/fmp4_recorder/prog_fmp4_recorder
```

Prerequisites: CMake ≥ 3.10 plus the ARM uClibc cross toolchain
`arm-linaro-linux-uclibcgnueabihf-9.1.0` (**not included in this repository**; download it from the cloud "Device SDK" folder).

The toolchain path lives in [`sdk/cmake/arm-uclibc.cmake`](sdk/cmake/arm-uclibc.cmake) as `set(TOOLCHAIN_BASE ...)`. It defaults to the vendor's internal build machine and **must be changed to your own path before building locally**.

## Deploy and run

```bash
adb push build/fmp4_recorder/prog_fmp4_recorder /customer/sd/
adb push build/libsxr_recorder/libsxr_recorder.so /customer/sd/   # only if you use a self-built recorder
adb shell "chmod +x /customer/sd/prog_fmp4_recorder"

adb shell 'cd /customer/sd && LD_LIBRARY_PATH=/customer/sd ./prog_fmp4_recorder \
           --output /customer/sd/out --segs 2 --segdur 10'
```

| Option | Meaning |
| --- | --- |
| `--output DIR` | Dataset output directory (**always put it on the SD card, `/customer/sd/`**) |
| `--segs N` | Number of recording segments (one `clip_NNN/` each) |
| `--segdur S` | Duration per segment, in seconds |

Caveats (details in [`sdk/fmp4_recorder/README.md`](sdk/fmp4_recorder/README.md)):

- `/tmp` is a tmpfs and writing large files there causes OOM — never point the output directory at `/tmp`.
- Within the same application you may loop `init/start/stop/deinit` freely; **switching to another MI application (e.g. the reference demo `prog_pipeline`) requires `adb reboot`**, otherwise the VENC channel is still held and `MI_VENC_CreateChn` fails with `0xa0022004`.
- Recording parameters are fixed inside `libsxr_recorder`. To change bitrate / GOP / preview / audio / clocks you must bypass the recorder and call `libsxr_hal`'s `pipeline_create_venc(SxrPipelineConfig*)` directly.

---

## Dataset format

All streams (video exposure, accel/gyro/mag, audio) share the **`CLOCK_MONOTONIC_RAW` nanosecond domain**, monotonically increasing since power-on and unaffected by NTP, so they align with each other by construction. **This is not wall-clock time** — `date -d @ts` will not work. Convert to UTC with the sidecar file `sensor_time_to_utc.csv` (sampled at ~1 Hz, interpolable so NTP steps can be tracked).

```
<output>/                                # default /customer/sd/fmp4_recorder
└── clip_001/                            # NNN starts at 001, zero-padded to three digits
    ├── id.txt                           # hardware UUID
    ├── sensor_time_to_utc.csv           # MONOTONIC_RAW → UTC mapping
    ├── Camera0/  Camera1/               # left eye / right eye
    │   ├── video.mp4                    # HEVC fMP4
    │   ├── metainfo.csv                 # per-frame exposure metadata
    │   └── camera_params.json           # intrinsics + extrinsics
    ├── Sensors/
    │   ├── accel.csv  gyro.csv  mag.csv
    │   └── imu_calibration.json
    └── Audio/                           # optional: omitted entirely if no microphone
        ├── audio.mp4                    # PCM fMP4, S16_LE interleaved 16 kHz 2ch
        └── metainfo.csv                 # pts_us, ts_ns, samples
```

| Default parameters | Value |
| --- | --- |
| Resolution | Auto-detected: SC235HGS = 1600×1200, SC233HGS = 1920×1200 |
| Main stream | HEVC (H.265) Main, **IP-only with no B-frames**, GOP=30, CBR 4 Mbps |
| Sub-stream (preview only, not in the dataset) | Main / 4 (400×300 / 480×300), 2 Mbps, 15 fps; VENC ch1=Cam0 / ch3=Cam1 |
| Sample rates | accel 800 Hz · gyro 800 Hz · mag 200 Hz |
| Audio | 16 kHz, 2 ch, S16_LE (the SDK has no audio encoder; PCM is stored as fMP4) |
| IQ | Auto-selected by sensor model: `sc235hgs_api.bin` / `sc233hgs_api.bin` |
| Clocks | ISP 216 MHz, SCL 288 MHz (set internally via `/proc`) |

Field-level definitions, coordinate-frame semantics and alignment conversion: [`docs/E2_HMD_SDK_en.html`](docs/E2_HMD_SDK_en.html) Part II.

---

## Cloud downloads

| Category | Google Drive | Baidu Netdisk (access code) |
| --- | --- | --- |
| Device SDK (incl. cross toolchain) | [Drive](https://drive.google.com/drive/folders/1PO7V0ip8HJR5GmlG18mPJAG_f-jUrqwd?usp=drive_link) | [pan.baidu.com/s/1jxtSpGT4zHtHYtWLeg7hNQ](https://pan.baidu.com/s/1jxtSpGT4zHtHYtWLeg7hNQ) `9mqq` |
| Headset ROM (`SgsUpgradeSD.bin`) | [Drive](https://drive.google.com/drive/folders/1u9Pulibxpo12T_amFZ8u82-AhPMJPk7b?usp=drive_link) | [pan.baidu.com/s/1z69drlVivOh68w6jMt10sA](https://pan.baidu.com/s/1z69drlVivOh68w6jMt10sA) `fw6r` |
| Phone APP & SDK | [Drive](https://drive.google.com/drive/folders/1bI0_epjY9s8yilUQ8GBD48v23xIB0WzA?usp=drive_link) | [pan.baidu.com/s/1bkdWndppacK0lsErQPr2FQ](https://pan.baidu.com/s/1bkdWndppacK0lsErQPr2FQ) `akh3` |
| Debug environment (PotPlayer / VLC / adb-setup) | [Drive](https://drive.google.com/drive/folders/1vFnwbaoV0jHKZI4RYlprebZa4C1r45hQ?usp=drive_link) | [pan.baidu.com/s/1k1v5Zt3lnU0IQe9J_t8KGw](https://pan.baidu.com/s/1k1v5Zt3lnU0IQe9J_t8KGw) `86c8` |
| User guide (video) | [Drive](https://drive.google.com/file/d/1RxNcjXfsZo3Ah8fLYhHO6Z3dqtN3ZYWG/view?usp=drive_link) | [pan.baidu.com/s/1-B5qp9GgTy4KmHj2MAo6vw](https://pan.baidu.com/s/1-B5qp9GgTy4KmHj2MAo6vw) `uadc` |
| Sample videos / datasets | [Drive](https://drive.google.com/drive/folders/1a1lDFsN2bnnkSDcSnP5Nrqzk5Eb78XEP?usp=drive_link) | [pan.baidu.com/s/1APj3P7N2iwz5dIyouLma0w](https://pan.baidu.com/s/1APj3P7N2iwz5dIyouLma0w?pwd=67cs) `67cs` |

---

## License

This project is licensed under the [MIT License](LICENSE).

Third-party components (for example, open-source libraries referenced under `sdk/`) remain under their own original licenses; see the notice file in each directory.
