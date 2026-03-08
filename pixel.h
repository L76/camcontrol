#ifndef __PIXEL_H__
#define __PIXEL_H__

#include <stdint.h>
#include <GxIAPI.h>
#include "DxImageProc.h"

#define PIXFMT_CVT_FAIL         -1              ///< PixelFormatConvert fail
#define PIXFMT_CVT_SUCCESS      0

typedef struct {
    int64_t pixelFormat;
    int64_t frameSize;
    uint8_t * RBGimageBuf;
    uint8_t *Raw8Buf;
    uint8_t *MonoImageBuf;
}PixelProcState_T;

PixelProcState_T * PixelProcInit(int64_t frameSize, int64_t colorFilter);

int PixelFormatConvert(PixelProcState_T *state, GX_FRAME_CALLBACK_PARAM *pFrameBuffer);

#endif
