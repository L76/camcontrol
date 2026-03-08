#include "pixel.h"
#include <GxIAPI.h>
#include "DxImageProc.h"
#include <stdio.h>

/**
\brief Convert frame date to suitable pixel format
\param pFrameBuffer[in]    FrameData from camera
\return void
*/
//-------------------------------------------------

bool g_bColorFilter = false;                        ///< Color filter support flag
int64_t g_i64ColorFilter = GX_COLOR_FILTER_NONE;    ///< Color filter of device

PixelProcState_T*
PixelProcInit(int64_t frameSize, int64_t colorFilter)
{
    PixelProcState_T *state = malloc(sizeof(PixelProcState_T));
    state->frameSize = frameSize;
    state->pixelFormat = colorFilter;
    state->MonoImageBuf = malloc((state->frameSize ));
    state->Raw8Buf = malloc((state->frameSize ));
    state->RBGimageBuf = malloc((state->frameSize ) * 3);
    return state;
}

/*


#define VERIFY_STATUS_RET(emStatus)\
if(emStatus != GX_STATUS_SUCCESS)\
{\
  return emStatus;\
}\
GX_STATUS emStatus = GX_STATUS_SUCCESS;

emStatus = GXSetBool(m_hDevice, GX_BOOL_REVERSE_Y, false);
VERIFY_STATUS_RET(emStatus);

bool bValue = true;
emStatus = GXGetBool(m_hDevice, GX_BOOL_REVERSE_Y, &bValue);
VERIFY_STATUS_RET(emStatus);

*/

int
PixelFormatConvert(PixelProcState_T *state, GX_FRAME_CALLBACK_PARAM * pFrameBuffer)
{
    VxInt32 emDXStatus = DX_OK;

    // Convert RAW8 or RAW16 image to RGB24 image
    switch (pFrameBuffer->nPixelFormat)
    {
        case GX_PIXEL_FORMAT_MONO8:
        {
            memcpy(state->MonoImageBuf, pFrameBuffer->pImgBuf, state->frameSize);
            break;
        }
        case GX_PIXEL_FORMAT_MONO10:
        case GX_PIXEL_FORMAT_MONO12:
        {
            //Convert to the Raw8 image
            emDXStatus = DxRaw16toRaw8((unsigned char*)pFrameBuffer->pImgBuf, state->MonoImageBuf, pFrameBuffer->nWidth, pFrameBuffer->nHeight, DX_BIT_2_9);
            if (emDXStatus != DX_OK)
            {
                printf("DxRaw16toRaw8 Failed, Error Code: %d\n", emDXStatus);
                return PIXFMT_CVT_FAIL;
            }
            break;
        }
        case GX_PIXEL_FORMAT_BAYER_GR8:
        case GX_PIXEL_FORMAT_BAYER_RG8:
        case GX_PIXEL_FORMAT_BAYER_GB8:
        case GX_PIXEL_FORMAT_BAYER_BG8:
        {
            // Convert to the RGB image
            emDXStatus = DxRaw8toRGB24((unsigned char*)pFrameBuffer->pImgBuf, state->RBGimageBuf, pFrameBuffer->nWidth, pFrameBuffer->nHeight,
                              RAW2RGB_NEIGHBOUR, (DX_PIXEL_COLOR_FILTER)state->pixelFormat, false);
            if (emDXStatus != DX_OK)
            {
                printf("DxRaw8toRGB24 Failed, Error Code: %d\n", emDXStatus);
                return PIXFMT_CVT_FAIL;
            }
            break;
        }
        case GX_PIXEL_FORMAT_BAYER_GR10:
        case GX_PIXEL_FORMAT_BAYER_RG10:
        case GX_PIXEL_FORMAT_BAYER_GB10:
        case GX_PIXEL_FORMAT_BAYER_BG10:
        case GX_PIXEL_FORMAT_BAYER_GR12:
        case GX_PIXEL_FORMAT_BAYER_RG12:
        case GX_PIXEL_FORMAT_BAYER_GB12:
        case GX_PIXEL_FORMAT_BAYER_BG12:
        {
            // Convert to the Raw8 image
            emDXStatus = DxRaw16toRaw8((unsigned char*)pFrameBuffer->pImgBuf, state->Raw8Buf, pFrameBuffer->nWidth, pFrameBuffer->nHeight, DX_BIT_2_9);
            if (emDXStatus != DX_OK)
            {
                printf("DxRaw16toRaw8 Failed, Error Code: %d\n", emDXStatus);
                return PIXFMT_CVT_FAIL;
            }
            // Convert to the RGB24 image
            emDXStatus = DxRaw8toRGB24(state->Raw8Buf, state->RBGimageBuf, pFrameBuffer->nWidth, pFrameBuffer->nHeight,
                              RAW2RGB_NEIGHBOUR, (DX_PIXEL_COLOR_FILTER) state->pixelFormat, false);
            if (emDXStatus != DX_OK)
            {
                printf("DxRaw8toRGB24 Failed, Error Code: %d\n", emDXStatus);
                return PIXFMT_CVT_FAIL;
            }
            break;
        }
        default:
        {
            printf("Error : PixelFormat of this camera is not supported\n");
            return PIXFMT_CVT_FAIL;
        }
    }
    return PIXFMT_CVT_SUCCESS;
}


