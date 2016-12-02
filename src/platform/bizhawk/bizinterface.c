#include <stdlib.h>
#include "core/core.h"
#include "core/log.h"
#include "util/common.h"
#include "gba/core.h"
#include "gba/gba.h"
#include "gba/renderers/video-software.h"
#include "gba/overrides.h"
#include "util/vfs.h"
#include "core/serialize.h"

const char* const binaryName = "mgba";
const uint32_t DEBUGGER_ID = 0xFEEDFACE;

#define EXP __declspec(dllexport)

/**
 * container_of - cast a member of a structure out to the containing structure
 * @ptr:		the pointer to the member.
 * @type:	   the type of the container struct this is embedded in.
 * @member:	 the name of the member within the struct.
 *
 */
#define container_of(ptr, type, member) ({					  \
		const typeof( ((type *)0)->member ) *__mptr = (ptr);	\
		(type *)( (char *)__mptr - offsetof(type,member) );})
void mDebuggerEnter(struct mDebugger* debugger, enum mDebuggerEntryReason reason, struct mDebuggerEntryInfo* info) { }
struct VFile* VFileOpenFD(const char* path, int flags) { return NULL; }

typedef struct
{
	struct mCore* core;
	struct mLogger logger;
	struct GBA* gba; // anything that uses this will be deprecated eventually
	color_t vbuff[VIDEO_HORIZONTAL_PIXELS * VIDEO_VERTICAL_PIXELS];
	void* rom;
	struct VFile* romvf;
	char bios[16384];
	struct VFile* biosvf;
	char sram[131072];
	struct VFile* sramvf;
	struct mKeyCallback keysource;
	struct mRotationSource rotsource;
	struct mRTCSource rtcsource;
	struct GBALuminanceSource lumasource;
	int16_t tiltx;
	int16_t tilty;
	int16_t tiltz;
	int64_t time;
	uint8_t light;
	uint16_t keys;
	int lagged;
	int skipbios;
} bizctx;

static int32_t GetX(struct mRotationSource* rotationSource)
{
	return container_of(rotationSource, bizctx, rotsource)->tiltx << 16;
}
static int32_t GetY(struct mRotationSource* rotationSource)
{
	return container_of(rotationSource, bizctx, rotsource)->tilty << 16;
}
static int32_t GetZ(struct mRotationSource* rotationSource)
{
	return container_of(rotationSource, bizctx, rotsource)->tiltz << 16;
}
static uint8_t GetLight(struct GBALuminanceSource* luminanceSource)
{
	return container_of(luminanceSource, bizctx, lumasource)->light;
}
static time_t GetTime(struct mRTCSource* rtcSource)
{
	return container_of(rtcSource, bizctx, rtcsource)->time;
}
static uint16_t GetKeys(struct mKeyCallback* keypadSource)
{
	bizctx *ctx = container_of(keypadSource, bizctx, keysource);
	ctx->lagged = FALSE;
	return ctx->keys;
}
static void RotationCB(struct mRotationSource* rotationSource)
{
	bizctx* ctx = container_of(rotationSource, bizctx, rotsource);
	ctx->lagged = FALSE;
}
static void LightCB(struct GBALuminanceSource* luminanceSource)
{
	bizctx* ctx = container_of(luminanceSource, bizctx, lumasource);
	ctx->lagged = FALSE;
}
static void TimeCB(struct mRTCSource* rtcSource)
{
	// no, reading the rtc registers should not unset the lagged flag
	// bizctx* ctx = container_of(rtcSource, bizctx, rtcsource);
	// ctx->lagged = FALSE;
}
static void logdebug(struct mLogger* logger, int category, enum mLogLevel level, const char* format, va_list args)
{

}

static void resetinternal(bizctx* ctx)
{
	ctx->core->reset(ctx->core);
	if (ctx->skipbios)
		GBASkipBIOS(ctx->gba);
}

EXP void BizDestroy(bizctx* ctx)
{
	ctx->core->deinit(ctx->core);
	free(ctx->rom);
	free(ctx);
}

typedef struct
{
	enum SavedataType savetype;
	enum GBAHardwareDevice hardware;
	uint32_t idleLoop;
} overrideinfo;

EXP bizctx* BizCreate(const void* bios, const void* data, int length, const overrideinfo* dbinfo, int skipbios)
{
	bizctx* ctx = calloc(1, sizeof(*ctx));
	if (!ctx)
	{
		return NULL;
	}

	ctx->rom = malloc(length);
	if (!ctx->rom)
	{
		free(ctx);
		return NULL;
	}

	ctx->logger.log = logdebug;
	mLogSetDefaultLogger(&ctx->logger);
	ctx->skipbios = skipbios;

	memcpy(ctx->rom, data, length);
	ctx->romvf = VFileFromMemory(ctx->rom, length);
	ctx->core = GBACoreCreate();
	if (!ctx->core)
	{
		ctx->romvf->close(ctx->romvf);
		free(ctx->rom);
		free(ctx);
		return NULL;
	}
	mCoreInitConfig(ctx->core, NULL);
	if (!ctx->core->init(ctx->core))
	{
		free(ctx->rom);
		free(ctx);
		return NULL;
	}
	ctx->gba = ctx->core->board;

	ctx->core->setVideoBuffer(ctx->core, ctx->vbuff, VIDEO_HORIZONTAL_PIXELS);
	ctx->core->setAudioBufferSize(ctx->core, 1024);

	blip_set_rates(ctx->core->getAudioChannel(ctx->core, 0), ctx->core->frequency(ctx->core), 44100);
	blip_set_rates(ctx->core->getAudioChannel(ctx->core, 1), ctx->core->frequency(ctx->core), 44100);

	ctx->core->loadROM(ctx->core, ctx->romvf);
	ctx->sramvf = VFileFromMemory(ctx->sram, 131072);
	ctx->core->loadSave(ctx->core, ctx->sramvf);

	ctx->core->setRTC(ctx->core, &ctx->rtcsource);
	ctx->core->setRotation(ctx->core, &ctx->rotsource);

	ctx->gba->luminanceSource = &ctx->lumasource; // ??
	ctx->gba->idleOptimization = IDLE_LOOP_IGNORE; // ??
	ctx->gba->realisticTiming = TRUE; // ??
	ctx->gba->keyCallback = &ctx->keysource; // ??

	ctx->keysource.readKeys = GetKeys;
	ctx->rotsource.sample = RotationCB;
	ctx->rotsource.readTiltX = GetX;
	ctx->rotsource.readTiltY = GetY;
	ctx->rotsource.readGyroZ = GetZ;
	ctx->lumasource.sample = LightCB;
	ctx->lumasource.readLuminance = GetLight;
	ctx->rtcsource.sample = TimeCB;
	ctx->rtcsource.unixTime = GetTime;

	if (bios)
	{
		memcpy(ctx->bios, bios, 16384);
		ctx->biosvf = VFileFromMemory(ctx->bios, 16384);
		/*if (!GBAIsBIOS(ctx->biosvf))
		{
			ctx->biosvf->close(ctx->biosvf);
			GBADestroy(&ctx->gba);
			free(ctx);
			return NULL;
		}*/
		ctx->core->loadBIOS(ctx->core, ctx->biosvf, 0);
	}

	if (dbinfo) // front end override
	{
		struct GBACartridgeOverride override;
		const struct GBACartridge* cart = (const struct GBACartridge*) ctx->gba->memory.rom;
		memcpy(override.id, &cart->id, sizeof(override.id));
		override.savetype = dbinfo->savetype;
		override.hardware = dbinfo->hardware;
		override.idleLoop = dbinfo->idleLoop;
		GBAOverrideApply(ctx->gba, &override);
	}

	resetinternal(ctx);
	return ctx;
}

EXP void BizReset(bizctx* ctx)
{
	resetinternal(ctx);
}

static void blit(void* dst_, const void* src_)
{
	// swap R&B, set top (alpha) byte
	const uint8_t* src = (const uint8_t*)src_;
	uint8_t* dst = (uint8_t*)dst_;

	uint8_t* dst_end = dst + VIDEO_HORIZONTAL_PIXELS * VIDEO_VERTICAL_PIXELS * BYTES_PER_PIXEL;

	while (dst < dst_end)
	{
		dst[2] = src[0] | src[0] >> 5;
		dst[1] = src[1] | src[1] >> 5;
		dst[0] = src[2] | src[2] >> 5;
		dst[3] = 0xff;
		dst += 4;
		src += 4;
		dst[2] = src[0] | src[0] >> 5;
		dst[1] = src[1] | src[1] >> 5;
		dst[0] = src[2] | src[2] >> 5;
		dst[3] = 0xff;
		dst += 4;
		src += 4;
		dst[2] = src[0] | src[0] >> 5;
		dst[1] = src[1] | src[1] >> 5;
		dst[0] = src[2] | src[2] >> 5;
		dst[3] = 0xff;
		dst += 4;
		src += 4;
		dst[2] = src[0] | src[0] >> 5;
		dst[1] = src[1] | src[1] >> 5;
		dst[0] = src[2] | src[2] >> 5;
		dst[3] = 0xff;
		dst += 4;
		src += 4;
	}
}

EXP int BizAdvance(bizctx* ctx, uint16_t keys, color_t* vbuff, int* nsamp, int16_t* sbuff,
	int64_t time, int16_t gyrox, int16_t gyroy, int16_t gyroz, uint8_t luma)
{
	ctx->keys = keys;
	ctx->light = luma;
	ctx->time = time;
	ctx->tiltx = gyrox;
	ctx->tilty = gyroy;
	ctx->tiltz = gyroz;
	ctx->lagged = TRUE;
	ctx->core->runFrame(ctx->core);

	blit(vbuff, ctx->vbuff);
	*nsamp = blip_samples_avail(ctx->core->getAudioChannel(ctx->core, 0));
	if (*nsamp > 1024)
		*nsamp = 1024;
	blip_read_samples(ctx->core->getAudioChannel(ctx->core, 0), sbuff, 1024, TRUE);
	blip_read_samples(ctx->core->getAudioChannel(ctx->core, 1), sbuff + 1, 1024, TRUE);
	return ctx->lagged;
}

struct MemoryAreas
{
	const void* bios;
	const void* wram;
	const void* iwram;
	const void* mmio;
	const void* palram;
	const void* vram;
	const void* oam;
	const void* rom;
	const void* sram;
	uint32_t sram_size;
};

EXP int BizGetSaveRamSize(bizctx* ctx)
{
	return GBASavedataSize(&ctx->gba->memory.savedata);
}

EXP void BizGetMemoryAreas(bizctx* ctx, struct MemoryAreas* dst)
{
	dst->bios = ctx->gba->memory.bios;
	dst->wram = ctx->gba->memory.wram;
	dst->iwram = ctx->gba->memory.iwram;
	dst->mmio = ctx->gba->memory.io;
	dst->palram = ctx->gba->video.palette;
	dst->vram = ctx->gba->video.renderer->vram;
	dst->oam = ctx->gba->video.oam.raw;
	dst->rom = ctx->gba->memory.rom;
	dst->sram = ctx->sram; //gba->memory.savedata.data;
	dst->sram_size = BizGetSaveRamSize(ctx);
}

EXP int BizGetSaveRam(bizctx* ctx, void* data, int size)
{
	ctx->sramvf->seek(ctx->sramvf, 0, SEEK_SET);
	return ctx->sramvf->read(ctx->sramvf, data, size);
}

EXP void BizPutSaveRam(bizctx* ctx, const void* data, int size)
{
	ctx->sramvf->seek(ctx->sramvf, 0, SEEK_SET);
	ctx->sramvf->write(ctx->sramvf, data, size);
}

// state sizes can vary!
EXP int BizStartGetState(bizctx* ctx, struct VFile** file, int* size)
{
	struct VFile* vf = VFileMemChunk(NULL, 0);
	if (!mCoreSaveStateNamed(ctx->core, vf, SAVESTATE_SAVEDATA))
	{
		vf->close(vf);
		return 0;
	}
	*file = vf;
	*size = vf->seek(vf, 0, SEEK_END);
	return 1;
}

EXP void BizFinishGetState(struct VFile* file, void* data, int size)
{
	file->seek(file, 0, SEEK_SET);
	file->read(file, data, size);
	file->close(file);
}

EXP int BizPutState(bizctx* ctx, const void* data, int size)
{
	struct VFile* vf = VFileFromConstMemory(data, size);
	int ret = mCoreLoadStateNamed(ctx->core, vf, SAVESTATE_SAVEDATA);
	vf->close(vf);
	return ret;
}

EXP void BizSetLayerMask(bizctx *ctx, int mask)
{
	struct GBAVideoRenderer *r = ctx->gba->video.renderer;
	r->disableBG[0] = !(mask & 1);
	r->disableBG[1] = !(mask & 2);
	r->disableBG[2] = !(mask & 4);
	r->disableBG[3] = !(mask & 8);
	r->disableOBJ = !(mask & 16);
}

EXP void BizSetSoundMask(bizctx* ctx, int mask)
{
	struct GBAAudio *a = &ctx->gba->audio;
	struct GBAudio *g = &a->psg;

	g->forceDisableCh[0] = !(mask & 1);
	g->forceDisableCh[0] = !(mask & 2);
	g->forceDisableCh[0] = !(mask & 4);
	g->forceDisableCh[0] = !(mask & 8);
	a->forceDisableChA = !(mask & 16);
	a->forceDisableChB = !(mask & 32);
}
