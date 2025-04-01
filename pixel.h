#ifndef __PIXEL_H__
#define __PIXEL_H__

#include <stdint.h>
#include <stdbool.h>
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

//int PixelFormatConvert(PixelProcState_T *state, PGX_FRAME_BUFFER pFrameBuffer);
int PixelFormatConvert(PixelProcState_T *state, GX_FRAME_CALLBACK_PARAM *pFrameBuffer);

void RGB24toGrayscale8(uint8_t *inputFrame, uint8_t *outputFrame, int h, int w);

void RGB24RedtoGrayscale8(uint8_t *inputFrame, uint8_t *outputFrame, int h, int w);

void RGB24GreentoGrayscale8(uint8_t *inputFrame, uint8_t *outputFrame, int h, int w);

void * savePngToMem(unsigned char *frame, uint32_t w, uint32_t h);

#endif
