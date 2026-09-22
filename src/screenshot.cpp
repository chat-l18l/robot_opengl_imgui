/**
 * @file screenshot.cpp
 * @brief Framebuffer readback and a minimal PNG encoder.
 */

#include "screenshot.h"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zlib.h>

/** Bytes per pixel written: PNG colour type 2 is 8-bit RGB. */
#define RBT_PNG_CHANNELS 3

/** @brief Write a big-endian 32-bit value. */
static void s_put_be32(unsigned char *out, uint32_t value)
{
    out[0] = (unsigned char)((value >> 24) & 0xFFu);
    out[1] = (unsigned char)((value >> 16) & 0xFFu);
    out[2] = (unsigned char)((value >> 8) & 0xFFu);
    out[3] = (unsigned char)(value & 0xFFu);
}

/**
 * @brief Write one PNG chunk: length, type, payload, CRC.
 *
 * The CRC covers the type and the payload but not the length, which is what
 * trips up most hand-written encoders.
 */
static bool s_write_chunk(FILE *file, const char type[4], const unsigned char *data, size_t length)
{
    assert(file != NULL);
    assert(type != NULL);

    unsigned char header[4];
    s_put_be32(header, (uint32_t)length);
    if (fwrite(header, 1, sizeof(header), file) != sizeof(header)) {
        return false;
    }
    if (fwrite(type, 1, 4, file) != 4) {
        return false;
    }
    if (length > 0 && fwrite(data, 1, length, file) != length) {
        return false;
    }

    uLong crc = crc32(0L, Z_NULL, 0);
    crc = crc32(crc, (const Bytef *)type, 4);
    if (length > 0) {
        crc = crc32(crc, (const Bytef *)data, (uInt)length);
    }

    unsigned char crc_bytes[4];
    s_put_be32(crc_bytes, (uint32_t)crc);
    return fwrite(crc_bytes, 1, sizeof(crc_bytes), file) == sizeof(crc_bytes);
}

/**
 * @brief Encode top-down RGB rows as a PNG file.
 *
 * Every row is prefixed with filter type 0, the minimum a decoder must accept.
 * Filtering would compress better; the images are small enough not to care.
 */
static bool s_write_png(const char *path, const unsigned char *rows, GLsizei width, GLsizei height)
{
    const size_t stride    = (size_t)width * RBT_PNG_CHANNELS;
    const size_t raw_size  = ((size_t)height) * (stride + 1u);
    unsigned char *raw     = (unsigned char *)malloc(raw_size);
    if (raw == NULL) {
        fprintf(stderr, "screenshot: out of memory staging %dx%d\n", width, height);
        return false;
    }
    for (GLsizei row = 0; row < height; row++) {
        unsigned char *dst = raw + (size_t)row * (stride + 1u);
        dst[0] = 0;  /* filter: none */
        memcpy(dst + 1, rows + (size_t)row * stride, stride);
    }

    uLongf packed_size = compressBound((uLong)raw_size);
    unsigned char *packed = (unsigned char *)malloc(packed_size);
    if (packed == NULL) {
        free(raw);
        fprintf(stderr, "screenshot: out of memory compressing\n");
        return false;
    }
    const int zip = compress2(packed, &packed_size, raw, (uLong)raw_size, Z_BEST_COMPRESSION);
    free(raw);
    if (zip != Z_OK) {
        free(packed);
        fprintf(stderr, "screenshot: deflate failed (%d)\n", zip);
        return false;
    }

    FILE *file = fopen(path, "wb");
    if (file == NULL) {
        free(packed);
        fprintf(stderr, "screenshot: cannot open %s for writing\n", path);
        return false;
    }

    static const unsigned char signature[8] = {0x89, 'P', 'N', 'G', '\r', '\n', 0x1A, '\n'};
    unsigned char ihdr[13];
    s_put_be32(ihdr + 0, (uint32_t)width);
    s_put_be32(ihdr + 4, (uint32_t)height);
    ihdr[8]  = 8;  /* bit depth */
    ihdr[9]  = 2;  /* colour type: truecolour */
    ihdr[10] = 0;  /* compression: deflate */
    ihdr[11] = 0;  /* filter method 0 */
    ihdr[12] = 0;  /* no interlacing */

    const bool ok = fwrite(signature, 1, sizeof(signature), file) == sizeof(signature)
                    && s_write_chunk(file, "IHDR", ihdr, sizeof(ihdr))
                    && s_write_chunk(file, "IDAT", packed, packed_size)
                    && s_write_chunk(file, "IEND", NULL, 0);

    free(packed);
    if (fclose(file) != 0 || !ok) {
        fprintf(stderr, "screenshot: writing %s failed\n", path);
        return false;
    }
    return true;
}

bool rbt_screenshot_capture(const char *path, GLint x, GLint y, GLsizei width, GLsizei height)
{
    assert(path != NULL);

    if (width < 1 || height < 1) {
        fprintf(stderr, "screenshot: refusing a %dx%d capture\n", width, height);
        return false;
    }

    const size_t stride = (size_t)width * RBT_PNG_CHANNELS;
    unsigned char *pixels = (unsigned char *)malloc(stride * (size_t)height);
    if (pixels == NULL) {
        fprintf(stderr, "screenshot: out of memory reading %dx%d\n", width, height);
        return false;
    }

    /* Rows are tightly packed; the default 4-byte alignment would pad them. */
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(x, y, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels);

    /* GL hands back the bottom row first; PNG wants the top row first. */
    unsigned char *flipped = (unsigned char *)malloc(stride * (size_t)height);
    if (flipped == NULL) {
        free(pixels);
        fprintf(stderr, "screenshot: out of memory flipping\n");
        return false;
    }
    for (GLsizei row = 0; row < height; row++) {
        memcpy(flipped + (size_t)row * stride,
               pixels + (size_t)(height - 1 - row) * stride,
               stride);
    }
    free(pixels);

    const bool ok = s_write_png(path, flipped, width, height);
    free(flipped);
    if (ok) {
        printf("screenshot: wrote %s (%dx%d)\n", path, width, height);
    }
    return ok;
}
