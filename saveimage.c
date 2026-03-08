#include "png.h"
#include "saveimage.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <gmodule.h>

#include "utils.h"

static void
PngWriteCallback(png_structp  png_ptr, png_bytep data, png_size_t length)
{
    void* p = png_get_io_ptr(png_ptr);
    g_array_append_vals(p, data, length);
}

GArray*
savePngToMem(unsigned char *frame, uint32_t w, uint32_t h)
{
    const size_t bytesPerPixel = 1;
    GArray* mem_io = g_array_new(FALSE, TRUE, sizeof(uint8_t));

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

int
savePngToFile(char* filename, uint8_t* data, uint32_t w, uint32_t h)
{
    GArray* io = savePngToMem(data, w, h);
    FILE* out = fopen(filename, "wb");
    fwrite(io->data, sizeof(uint8_t), io->len, out);
    fflush(out);
    fclose(out);
    g_array_free(io, TRUE);
    return 0;
}
