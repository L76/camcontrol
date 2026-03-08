#ifndef __UTILS_H__
#define __UTILS_H__

#include <stdint.h>
#include <stddef.h>

#define SWAP(a, b) do { typeof(a) temp = a; a = b; b = temp; } while (0)

void RGB24toGrayscale8(uint8_t *inputFrame, uint8_t *outputFrame, int h, int w);
void RGB24RedtoGrayscale8(uint8_t *inputFrame, uint8_t *outputFrame, int h, int w);
void RGB24GreentoGrayscale8(uint8_t *inputFrame, uint8_t *outputFrame, int h, int w);
void RGB24BluetoGrayscale8(uint8_t *inputFrame, uint8_t *outputFrame, int h, int w);

#endif
