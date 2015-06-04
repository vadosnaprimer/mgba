#include <stdlib.h>
#include "util/common.h"
#include "gba/gba.h"
#include "gba/renderers/video-software.h"
#include "gba/serialize.h"
#include "gba/supervisor/overrides.h"
#include "gba/video.h"
#include "util/vfs.h"

#define EXP __declspec(dllexport)

void ARMDebuggerEnter(struct ARMDebugger* u1, enum DebuggerEntryReason u2, struct DebuggerEntryInfo* u3) { }
struct VFile* VFileOpen(const char* path, int flags) { return NULL; }

typedef struct
{
    struct GBA gba;
    struct ARMCore cpu;
    struct GBAVideoSoftwareRenderer renderer;
    struct GBAAVStream stream;
    color_t vbuff[VIDEO_HORIZONTAL_PIXELS * VIDEO_VERTICAL_PIXELS];
    void* rom;
    struct VFile* romfile;
} bizctx;

static void logdebug(struct GBAThread* thread, enum GBALogLevel level, const char* format, va_list args)
{

}

EXP void BizDestroy(bizctx* ctx)
{
    ctx->romfile->close(ctx->romfile);
    ctx->romfile = NULL;
    free(ctx->rom);
    ctx->rom = NULL;

    // TODO: this seems short.  is there anything else that needs to happen here?
    GBADestroy(&ctx->gba);
    free(ctx);
}

EXP bizctx* BizCreate(void)
{
    bizctx* ctx = calloc(1, sizeof(*ctx));
    if (ctx)
    {
        GBACreate(&ctx->gba);
        ARMSetComponents(&ctx->cpu, &ctx->gba.d, 0, NULL);
        ARMInit(&ctx->cpu);
        // TODO: configuration
        ctx->gba.logLevel = 0;
        ctx->gba.logHandler = logdebug;
        // ctx->gba.stream = &ctx->stream;
        ctx->gba.idleOptimization = IDLE_LOOP_IGNORE;

        GBAVideoSoftwareRendererCreate(&ctx->renderer);
        ctx->renderer.outputBuffer = ctx->vbuff;
        ctx->renderer.outputBufferStride = VIDEO_HORIZONTAL_PIXELS;
        GBAVideoAssociateRenderer(&ctx->gba.video, &ctx->renderer.d);

        GBAAudioResizeBuffer(&ctx->gba.audio, 2048);
        blip_set_rates(ctx->gba.audio.left, GBA_ARM7TDMI_FREQUENCY, 44100);
        blip_set_rates(ctx->gba.audio.right, GBA_ARM7TDMI_FREQUENCY, 44100);
    }
    return ctx;
}

EXP void BizReset(bizctx* ctx)
{
    ARMReset(&ctx->cpu);
}

EXP int BizLoad(bizctx* ctx, const void* data, int length)
{
    ctx->rom = malloc(length);
    if (!ctx->rom)
        return 0;

    memcpy(ctx->rom, data, length);
    ctx->romfile = VFileFromMemory(ctx->rom, length);

    if (!GBAIsROM(ctx->romfile))
    {
        ctx->romfile->close(ctx->romfile);
        ctx->romfile = NULL;
        free(ctx->rom);
        ctx->rom = NULL;
        return 0;
    }

    // TODO: savedata
    GBALoadROM(&ctx->gba, ctx->romfile, NULL, NULL);
    // TODO: what is GBAOverrideApply
    BizReset(ctx);
    return 1;
}

EXP void BizAdvance(bizctx* ctx, int keys, color_t **vbuff)
{
    ctx->gba.keySource = &keys;
    *vbuff = ctx->vbuff;
    int frameCount = ctx->gba.video.frameCounter;
    while (frameCount == ctx->gba.video.frameCounter)
    {
        ARMRunLoop(&ctx->cpu);
    }
}
