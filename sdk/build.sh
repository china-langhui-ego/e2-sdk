#!/bin/bash
# 一键编译：libsxr_recorder.so + prog_fmp4_recorder
# CMake 交叉编译（共用 cmake/arm-uclibc.cmake 工具链，host 上运行，无需 Docker）。
# 统一构建到 ego 根的 build/ 下；产物自动 strip（release 交付，剥符号）。
# 产物：
#   build/libsxr_recorder/libsxr_recorder.so   (app 运行期 dlopen 它)
#   build/fmp4_recorder/prog_fmp4_recorder      (app)
set -e

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TOOLCHAIN="$ROOT/cmake/arm-uclibc.cmake"

if [ ! -f "$TOOLCHAIN" ]; then
    echo "error: toolchain not found: $TOOLCHAIN" >&2
    exit 1
fi

# build_cmake <src-dir> <name>  →  $ROOT/build/<name>，随后 strip 产物。
# strip 路径：cmake configure 后从该 build 目录的 CMakeCache.txt 读展开后的
# CMAKE_C_COMPILER（工具链文件里是 ${TOOLCHAIN_BASE} 变量，未展开），把 -gcc 换 -strip。
build_cmake() {
    local src="$1" name="$2"
    local bd="$ROOT/build/$name"
    mkdir -p "$bd"
    ( cd "$bd" && cmake -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN" "$src" >/dev/null && make -j )
    local cc strip
    # CMake 3.10 cache 不存裸 CMAKE_C_COMPILER，但存 *_COMPILER_AR（.../bin/<triple>-gcc-ar）。
    # 纯 C++ 工程只有 CXX 变体，所以两个都试。从 -gcc-ar 路径派生 strip（-gcc-ar → -strip）。
    cc=$(grep -m1 -E '^CMAKE_(C|CXX)_COMPILER_AR:FILEPATH=' "$bd/CMakeCache.txt" 2>/dev/null | cut -d= -f2-)
    strip=${cc%-gcc-ar}-strip
    if [ -x "$strip" ]; then
        for a in "$bd"/*.so "$bd"/prog_*; do
            [ -e "$a" ] || continue
            "$strip" --strip-unneeded "$a" 2>/dev/null || echo "  warn: strip failed: $a" >&2
        done
    else
        echo "  warning: strip not found ($strip)；$name 产物未 strip" >&2
    fi
}

echo "==> [1/2] libsxr_recorder.so  (运行期被 app dlopen)"
build_cmake "$ROOT/libsxr_recorder" libsxr_recorder
echo "    -> $ROOT/build/libsxr_recorder/libsxr_recorder.so"

echo "==> [2/2] prog_fmp4_recorder"
build_cmake "$ROOT/fmp4_recorder" fmp4_recorder
echo "    -> $ROOT/build/fmp4_recorder/prog_fmp4_recorder"

echo "==> done (产物已 strip)。把上面两个产物部署到设备 /customer/sd/ 即可运行。"
