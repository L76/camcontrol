#ifndef __UTILS_H__
#define __UTILS_H__

#include <stdint.h>
#include <stddef.h>

#define MAX(a, b) (((a)>=(b))?(a):(b))

#define MIN(a, b) (((a)<(b))?(a):(b))

#define SWAP(a, b) do { typeof(a) temp = a; a = b; b = temp; } while (0)

#define DYNAMIC_BUFFER_DEFAULT_SIZE 1024

typedef struct {
    uint8_t *data;
    size_t alloc_size;
    size_t count;
}DynamicBuffer_T;

DynamicBuffer_T *dynamicBufferCreate(size_t initialSize);

int dynamicBufferPush(DynamicBuffer_T * buf, uint8_t *data, size_t len);

void dynamicBufferDestroy(DynamicBuffer_T * buf);

void dynamicBufferReset(DynamicBuffer_T * buf);

#endif