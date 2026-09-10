set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(TOOLCHAIN_BASE /home/svr00003/workspace/Embodied_Intelligence/E2-SDK/toolchains/arm-linaro-linux-uclibcgnueabihf-9.1.0)

set(CMAKE_C_COMPILER   ${TOOLCHAIN_BASE}/bin/arm-linaro-linux-uclibcgnueabihf-gcc)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_BASE}/bin/arm-linaro-linux-uclibcgnueabihf-g++)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
