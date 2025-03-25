#include "utils.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

DynamicBuffer_T *dynamicBufferCreate(size_t initialSize) {
    DynamicBuffer_T *buf = malloc(sizeof(DynamicBuffer_T));
    buf->data = calloc(initialSize, 1);
    buf->alloc_size = initialSize;
    buf->count = 0;
    return buf;
}

int dynamicBufferPush(DynamicBuffer_T * buf, uint8_t *data, size_t len) {
    if (buf->count + len > buf->alloc_size) {
        //need realloc
        size_t newSize = MAX(2*buf->alloc_size, buf->count + len);
        buf->data = realloc(buf->data, newSize);
        buf->alloc_size = newSize;
        printf("newSize = %d\r\n", newSize);
    }
    memcpy(buf->data + buf->count, data, len);
    buf->count += len;
    return 0;
}

void dynamicBufferReset(DynamicBuffer_T * buf) {
    //memset(buf->data, 0, buf->count);
    buf->count = 0;
}

void dynamicBufferDestroy(DynamicBuffer_T * buf) {
    free(buf->data);
    free(buf);
}

uint8_t *makeGradientRGBA(const int w, const int h) {
    unsigned char *RGBData = malloc(4 * w * h);
    for (int i=0; i<h; i++) {
        for (int j=0; j<w; j++) {
            float grad = (float) (i+j)/(w + h);
            RGBData[i*w * 4 + 4*j] = 255 * grad;
            RGBData[i*w * 4 + 4*j+1] = 255 * grad;
            RGBData[i*w * 4 + 4*j+2] = 255 * grad;
            RGBData[i*w * 4 + 4*j+3] = 255; //opacity
        }
    }
    return RGBData;
}

uint8_t *makeNoiseRGBA(const int w, const int h) {
    unsigned char *RGBData = malloc(4 * w * h);
    for (int i=0; i<h; i++) {
        for (int j=0; j<w; j++) {
            RGBData[i*w * 4 + 4*j] = rand()%255;
            RGBData[i*w * 4 + 4*j+1] = rand()%255;
            RGBData[i*w * 4 + 4*j+2] = rand()%255;
            RGBData[i*w * 4 + 4*j+3] = 255; //opacity
        }
    }
    return RGBData;
}


uint8_t *makeGradientRGB(const int w, const int h) {
    unsigned char *RGBData = malloc(4 * w * h);
    const size_t pixelSize = 3;
    for (int i=0; i<h; i++) {
        for (int j=0; j<w; j++) {
            float grad = (float) (i+j)/(w + h);
            RGBData[(i*w + j) * pixelSize] = 255 * grad;
            RGBData[(i*w + j) * pixelSize + 1] = 255 * grad;
            RGBData[(i*w + j) * pixelSize + 2] = 255 * grad;
        }
    }
    return RGBData;
}

uint8_t *makeNoiseRGB(const int w, const int h) {
    unsigned char *RGBData = malloc(4 * w * h);
    const size_t pixelSize = 3;
    for (int i=0; i<h; i++) {
        for (int j=0; j<w; j++) {
            float grad = (float) (i+j)/(w + h);
            RGBData[(i*w + j) * pixelSize]     = rand()%255;
            RGBData[(i*w + j) * pixelSize + 1] = rand()%255;
            RGBData[(i*w + j) * pixelSize + 2] = rand()%255;
        }
    }
    return RGBData;
}
