# Standalone Makefile for libsxr_recorder.so
# Build from Docker:
#   /home/sgs/SourceCode/sdk/ego/libsxr_recorder && make

-include /home/sgs/SourceCode/project/configs/current.configs

TARGET = libsxr_recorder.so

# SRCS — MI-free. MI pipeline/device access is dlopen'd via libsxr_hal.so
# (hal_loader.cpp); no libmi_* / libcam_os_wrapper linked here.
SRCS  = sxr_recorder.c mi_record.cpp FMP4Writer.cpp hal_loader.cpp calib_json.cpp sensor_csv.cpp

INC  = -I./include
INC += -I.
INC += -I../libsxr_hal

# MI-free link: only C runtime + dl/pthread/stdc++/m. libsxr_hal.so is NOT
# linked here — it is dlopen'd at runtime by hal_loader.cpp.
LIBS  = -ldl -lpthread -lstdc++ -lm

CXXFLAGS = -w -O2 -fPIC
CFLAGS   = -w -O2 -fPIC

# 32-bit ARM: off_t ->64 bit, fopen/lseek/ftruncate redirects to * 64 variant,
# Supports>2GB video files (fMP4 prealloc/moov back patch using lseek system calls).
CFLAGS   += -D_FILE_OFFSET_BITS=64
CXXFLAGS += -D_FILE_OFFSET_BITS=64

OBJS  = $(SRCS:.cpp=.o)
OBJS := $(OBJS:.c=.o)

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CROSS_COMPILE)g++ -shared -o $@ $^ $(LIBS) -Wl,-soname,$(TARGET)

%.o: %.cpp
	$(CROSS_COMPILE)g++ $(CXXFLAGS) $(INC) -c $< -o $@

%.o: %.c
	$(CROSS_COMPILE)gcc $(CFLAGS) $(INC) -c $< -o $@

clean:
	rm -f $(TARGET) *.o

.PHONY: all clean
