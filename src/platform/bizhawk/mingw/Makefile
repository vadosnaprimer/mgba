CXX = gcc
RM = rm
CP = cp

MACHINE = $(shell $(CXX) -dumpmachine)
ifneq (,$(findstring i686,$(MACHINE)))
	ARCH = 32
else ifneq (,$(findstring x86_64,$(MACHINE)))
	ARCH = 64
else
	$(error Unknown arch)
endif

CXXFLAGS = -s -I../../../arm -I../../.. -Wall -O3 -std=gnu99 -fgnu89-inline -fomit-frame-pointer -fexpensive-optimizations -flto -DRESAMPLE_LIBRARY=RESAMPLE_BLIP_BUF -DHAVE_LOCALTIME_R -DMINIMAL_CORE=2 -DDISABLE_THREADING

TARGET = mgba.dll

LDFLAGS_32 = -static-libgcc
LDFLAGS_64 =
LDFLAGS = -shared $(LDFLAGS_$(ARCH)) $(CXXFLAGS)

DEST_32 = ../../../../../output/dll
DEST_64 = ../../../../../output64/dll

SRCS = \
	../../../arm/arm.c \
	../../../arm/decoder-arm.c \
	../../../arm/decoder-thumb.c \
	../../../arm/decoder.c \
	../../../arm/isa-arm.c \
	../../../arm/isa-thumb.c \
	../../../gba/audio.c \
	../../../gba/bios.c \
	../../../gba/cheats.c \
	../../../gba/cheats/codebreaker.c \
	../../../gba/cheats/gameshark.c \
	../../../gba/cheats/parv3.c \
	../../../gba/context/config.c \
	../../../gba/context/overrides.c \
	../../../gba/context/sync.c \
	../../../gba/gba.c \
	../../../gba/hardware.c \
	../../../gba/hle-bios.c \
	../../../gba/io.c \
	../../../gba/memory.c \
	../../../gba/renderers/software-bg.c \
	../../../gba/renderers/software-mode0.c \
	../../../gba/renderers/software-obj.c \
	../../../gba/renderers/video-software.c \
	../../../gba/savedata.c \
	../../../gba/serialize.c \
	../../../gba/sio.c \
	../../../gba/sio/lockstep.c \
	../../../gba/video.c \
	../../../third-party/blip_buf/blip_buf.c \
	../../../third-party/inih/ini.c \
	../../../util/circle-buffer.c \
	../../../util/configuration.c \
	../../../util/crc32.c \
	../../../util/formatting.c \
	../../../util/hash.c \
	../../../util/string.c \
	../../../util/table.c \
	../../../util/vfs.c \
	../../../util/vfs/vfs-mem.c \
	../../windows/memory.c \
	../bizinterface.c \
	../localtime_r.c

OBJS = $(SRCS:.c=.o)

all: $(TARGET)

%.o: %.c
	$(CXX) -c -o $@ $< $(CXXFLAGS)

$(TARGET) : $(OBJS)
	$(CXX) -o $@ $(LDFLAGS) $(OBJS)

clean:
	$(RM) $(OBJS)
	$(RM) $(TARGET)

install:
	$(CP) $(TARGET) $(DEST_$(ARCH))
