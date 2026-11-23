/* The FC never compresses SEDSnet payloads: telemetry.c sets
 * payload_compress_threshold = SIZE_MAX because zstd's one-shot compressor
 * needs a ~3.8 KiB stack context plus a >= 18.9 KiB heap workspace, more
 * than this heap has. SEDSnet still references ZSTD_compress, and that one
 * reference made the linker pull zstd's whole compressor (about 18 KB of
 * flash that never runs). Defining the entry point here satisfies it, so
 * zstd_compress.o and everything only it needs stay out of the image.
 * Decompression is untouched. A call returns zstd's generic error code,
 * which SEDSnet already treats as "send the payload uncompressed". */
#include <stddef.h>

size_t ZSTD_compress(void *dst, size_t dst_capacity, const void *src, size_t src_size,
                     int compression_level);

size_t ZSTD_compress(void *dst, size_t dst_capacity, const void *src, size_t src_size,
                     int compression_level)
{
    (void)dst; (void)dst_capacity; (void)src; (void)src_size; (void)compression_level;
    return (size_t)-1; /* -ZSTD_error_GENERIC */
}
