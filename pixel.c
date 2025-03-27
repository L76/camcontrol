#include "pixel.h"
#include <GxIAPI.h>
#include "DxImageProc.h"
#include <stdio.h>
#include <png.h>
#include "utils.h"

/**
\brief Convert frame date to suitable pixel format
\param pFrameBuffer[in]    FrameData from camera
\return void
*/
//-------------------------------------------------

bool g_bColorFilter = false;                        ///< Color filter support flag
int64_t g_i64ColorFilter = GX_COLOR_FILTER_NONE;    ///< Color filter of device

PixelProcState_T * PixelProcInit(int64_t frameSize, int64_t colorFilter) {
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

int PixelFormatConvert(PixelProcState_T *state, GX_FRAME_CALLBACK_PARAM * pFrameBuffer)
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

void RGB24toGrayscale8(uint8_t *inputFrame, uint8_t *outputFrame, int h, int w) {
    int bytesPerPixel = 3;
    for (int i=0; i<h; i++) {
        for (int j=0; j<w; j++) {
            int _gray = inputFrame[ (i *w + j)*bytesPerPixel] * 21268 + inputFrame[ (i *w + j)*bytesPerPixel + 1] * 71510
                + inputFrame[ (i *w + j)*bytesPerPixel + 2] * 7217;
            outputFrame[i *w + j] = _gray/100000;
        }
    }
}

void RGB24RedtoGrayscale8(uint8_t *inputFrame, uint8_t *outputFrame, int h, int w) {
    int bytesPerPixel = 3;
    for (int i=0; i<h; i++) {
        for (int j=0; j<w; j++) {
            uint8_t _gray = 255-inputFrame[ (i *w + j)*bytesPerPixel];
            outputFrame[i *w + j] = _gray;
        }
    }
}

static void PngWriteCallback(png_structp  png_ptr, png_bytep data, png_size_t length) {
    void * p = png_get_io_ptr(png_ptr);
    //printf("Insert png data into %p, data %p, size = %d\r\n", p, data, length);
    dynamicBufferPush(p, data, length);
}

void * savePngToMem(unsigned char *frame, uint32_t w, uint32_t h) {
    const size_t bytesPerPixel = 1;
    const int avgCompLevel = 3;
    void *mem_io = dynamicBufferCreate(h * w * bytesPerPixel / avgCompLevel);

    png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    if (!png) {
        fprintf(stderr, "Could not create png\r\n");
        abort();
    }
    png_infop info = png_create_info_struct(png);
    if (!info) {
        fprintf(stderr, "Could not create png info\r\n");
        abort();
    }
    if (setjmp(png_jmpbuf(png))) {
        fprintf(stderr, "setjump failed\r\n");
        abort();
    }
    uint8_t *rows[h];
    for (uint32_t i=0; i<h; i++) {
        rows[i] = (uint8_t *)frame + i*w * bytesPerPixel;
    }

    png_set_IHDR(png, info, w, h, 8,
            PNG_COLOR_TYPE_GRAY, //PNG_COLOR_TYPE_RGB
            PNG_INTERLACE_NONE,
            PNG_COMPRESSION_TYPE_DEFAULT,
            PNG_FILTER_TYPE_DEFAULT);

    // Optional significant bit (sBIT) chunk.
    png_color_8 sig_bit;
    // If we are dealing with a grayscale image then
    sig_bit.gray = 8;
    png_set_sBIT(png, info, &sig_bit);

    png_set_compression_level(png, 2);
    png_set_compression_strategy(png, 2); //0-2
    png_set_filter(png,  0, PNG_FILTER_SUB);

    png_set_rows(png, info, &rows[0]);
    png_set_write_fn(png, mem_io, PngWriteCallback, NULL);
    png_write_png(png, info, PNG_TRANSFORM_IDENTITY, NULL);
    png_destroy_write_struct(&png, NULL);

    return mem_io;
}


void rgb_bitmap_make_sample (unsigned char *bitmap, int width, int height)
{
    for (int j = 0; j < height; j++)
        for (int i = 0; i < width; i++)
        {
            unsigned char s = random() % 255;
            for (int k = 1; k <= 3; k++)
                bitmap[3*(j*width + i) + k] = s;
        }
}
