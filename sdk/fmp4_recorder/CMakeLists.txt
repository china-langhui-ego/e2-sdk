cmake_minimum_required(VERSION 3.10)
project(prog_fmp4_recorder LANGUAGES CXX)

# prog_fmp4_recorder uses libsxr_recorder via dlopen (NOT linked against it).
# Only its public header is needed at compile time; build libsxr_recorder.so
# separately (../libsxr_recorder) and deploy it to the device.
#
# Build — shares libsxr_recorder's toolchain file (no Docker):
#   cd fmp4_recorder
#   mkdir build && cd build
#   cmake -DCMAKE_TOOLCHAIN_FILE=../../cmake/arm-uclibc.cmake ..
#   make
#   -> build/prog_fmp4_recorder
add_executable(prog_fmp4_recorder main.cpp)

target_include_directories(prog_fmp4_recorder PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}                     # main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../libsxr_recorder/include   # sxr_recorder.h (self-contained)
)

target_compile_options(prog_fmp4_recorder PRIVATE -w -O2)
target_compile_features(prog_fmp4_recorder PRIVATE cxx_std_11)

# dlopen inline loader needs -ldl; -pthread/-lm for the app's own use.
# libsxr_recorder is intentionally NOT linked (dlopen'd at runtime).
target_link_libraries(prog_fmp4_recorder PRIVATE dl pthread m)

# Device runtime library path (matches the standalone Makefile).
set_target_properties(prog_fmp4_recorder PROPERTIES
    LINK_FLAGS "-Wl,-rpath,/customer/sd/"
)
