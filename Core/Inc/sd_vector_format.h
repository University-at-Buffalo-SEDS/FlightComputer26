#ifndef SD_VECTOR_FORMAT_H
#define SD_VECTOR_FORMAT_H
#include "sd_float_format.h"
#include <stddef.h>

static inline int sd_format_vector(char *dst, size_t capacity,
                                    const float *data, size_t count)
{
    if (dst == NULL || capacity == 0) return -1;
    dst[0] = '\0';
    if (data == NULL) return -1;
    size_t used = 0;
    for (size_t i = 0; i < count; ++i) {
        char item[24];
        sd_format_float(item, data[i]);
        const size_t length = strlen(item);
        const size_t separator = i != 0;
        if (length + separator >= capacity - used) { dst[0] = '\0'; return -1; }
        if (separator) dst[used++] = ' ';
        memcpy(dst + used, item, length);
        used += length;
        dst[used] = '\0';
    }
    return (int)used;
}
#endif
