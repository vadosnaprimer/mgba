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

#-I../../../arm 
# -s -O3 -fomit-frame-pointer -fexpensive-optimizations -flto
CXXFLAGS = -s -O3 -fomit-frame-pointer -fexpensive-optimizations -flto -I../../.. -Wall -std=gnu99 -fgnu89-inline -DRESAMPLE_LIBRARY=RESAMPLE_BLIP_BUF -DHAVE_LOCALTIME_R -DMINIMAL_CORE=2 -DDISABLE_THREADING -DM_CORE_GBA

TARGET = mgba.dll

LDFLAGS_32 = -static-libgcc
LDFLAGS_64 =
LDFLAGS = -shared $(LDFLAGS_$(ARCH)) $(CXXFLAGS)

DEST_32 = ../../../../../output/dll
DEST_64 = ../../../../../output64/dll

SRCS = \
	../../../core/config.c \
	../../../core/cheats.c \
	../../../core/core.c \
	../../../core/directories.c \
	../../../core/input.c \
	../../../core/interface.c \
	../../../core/library.c \
	../../../core/log.c \
	../../../core/rewind.c \
	../../../core/serialize.c \
	../../../core/sync.c \
	../../../core/thread.c \
	../../../arm/debugger/debugger.c \
	../../../arm/debugger/memory-debugger.c \
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
	../../../gba/core.c \
	../../../gba/gba.c \
	../../../gba/hardware.c \
	../../../gba/hle-bios.c \
	../../../gba/io.c \
	../../../gba/memory.c \
	../../../gba/overrides.c \
	../../../gba/renderers/software-bg.c \
	../../../gba/renderers/software-mode0.c \
	../../../gba/renderers/software-obj.c \
	../../../gba/renderers/video-software.c \
	../../../gba/renderers/tile-cache.c \
	../../../gba/savedata.c \
	../../../gba/serialize.c \
	../../../gba/sio.c \
	../../../gba/sio/lockstep.c \
	../../../gba/video.c \
	../../../gba/vfame.c \
	../../../gb/audio.c \
	../../../third-party/blip_buf/blip_buf.c \
	../../../third-party/inih/ini.c \
	../../../util/circle-buffer.c \
	../../../util/configuration.c \
	../../../util/crc32.c \
	../../../util/patch-fast.c \
	../../../util/patch-ips.c \
	../../../util/patch-ups.c \
	../../../util/patch.c \
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
