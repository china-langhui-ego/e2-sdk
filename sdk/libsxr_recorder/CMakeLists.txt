cmake_minimum_required(VERSION 3.10)
project(libsxr_recorder LANGUAGES C CXX)

set(SOURCES
    sxr_recorder.c
    mi_record.cpp
    FMP4Writer.cpp
    hal_loader.cpp
    calib_json.cpp
    sensor_csv.cpp
)

add_library(sxr_recorder SHARED ${SOURCES})

target_include_directories(sxr_recorder PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${CMAKE_CURRENT_SOURCE_DIR}/../libsxr_hal
)

target_compile_options(sxr_recorder PRIVATE -w -O2 -fPIC)
target_compile_features(sxr_recorder PRIVATE cxx_std_11)

# No VERSION/SOVERSION set — CMake defaults SONAME to output filename:
# target "sxr_recorder" → libsxr_recorder.so → SONAME libsxr_recorder.so
# (matches the existing Makefile's -Wl,-soname,libsxr_recorder.so)
set_target_properties(sxr_recorder PROPERTIES
    LINKER_LANGUAGE CXX
    C_STANDARD 11
)

target_link_libraries(sxr_recorder PRIVATE
    dl pthread stdc++ m
)
