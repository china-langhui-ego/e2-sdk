# Standalone Makefile for prog_fmp4_recorder
# Build from Docker:
#   /home/sgs/SourceCode/sdk/ego/fmp4_recorder && make
#
# libsxr_recorder.so is dlopen'd at runtime (NOT linked here — only its header
# is needed at compile time). Build + push it for deployment: cd ../libsxr_recorder && make

-include /home/sgs/SourceCode/project/configs/current.configs

TARGET = prog_fmp4_recorder

SRCS = main.cpp

INC  = -I.
INC += -I../libsxr_recorder/include
INC += -I../libsxr_hal          # sxr_hal.h is now included transitively by sxr_recorder.h

# dlopen link: the app does NOT link libsxr_recorder — it is dlopen'd at
# runtime by the inline loader in sxr_recorder.h (SD-first). Only -ldl needed.
LIBS  = -ldl -lpthread -lstdc++ -lm

CXXFLAGS = -w -O2

OBJS = $(SRCS:.cpp=.o)

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CROSS_COMPILE)g++ -o $@ $^ $(LIBS) -Wl,-rpath,/customer/sd/

%.o: %.cpp
	$(CROSS_COMPILE)g++ $(CXXFLAGS) $(INC) -c $< -o $@

clean:
	rm -f $(TARGET) *.o

.PHONY: all clean
