#ifndef __SAVEIMAGE_H__
#define __SAVEIMAGE_H__

#include <stdint.h>
#include <stddef.h>

int savePngToFile(char* filename, uint8_t* data, uint32_t w, uint32_t h);

#endif
