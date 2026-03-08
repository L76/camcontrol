#include "utils.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

uint8_t*
makeGradientRGBA(const int w, const int h)
{
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
            RGBData[(i*w + j) * pixelSize]     = rand()%255;
            RGBData[(i*w + j) * pixelSize + 1] = rand()%255;
            RGBData[(i*w + j) * pixelSize + 2] = rand()%255;
        }
    }
    return RGBData;
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

void RGB24GreentoGrayscale8(uint8_t *inputFrame, uint8_t *outputFrame, int h, int w) {
    int bytesPerPixel = 3;
    for (int i=0; i<h; i++) {
        for (int j=0; j<w; j++) {
            uint8_t _gray = 255-inputFrame[ (i *w + j)*bytesPerPixel+1];
            outputFrame[i *w + j] = _gray;
        }
    }
}

void RGB24BluetoGrayscale8(uint8_t *inputFrame, uint8_t *outputFrame, int h, int w) {
    int bytesPerPixel = 3;
    for (int i=0; i<h; i++) {
        for (int j=0; j<w; j++) {
            uint8_t _gray = 255-inputFrame[ (i *w + j)*bytesPerPixel+2];
            outputFrame[i *w + j] = _gray;
        }
    }
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
