#ifndef SD_FILE_WIRE_H
#define SD_FILE_WIRE_H
/* SD file download protocol v1 + CRC on the SEDSnet P2P stream port 4511.
 * Same opcodes and frames as the DAQ (DAQ-Board Core/Inc/sd_file_wire.h).
 * GroundStation opens the stream; each request gets one bounded answer:
 *   LIST   [0x21][dir u8][index u16]                -> one log file entry
 *   READ   [0x22][offset u32][dir u8][len u8][name] -> <=512 bytes in DATA frames
 *   CANCEL [0x23]                                   -> drops any pending answer
 * Responses carry opcode|0x80 and a status byte; frames stay within 117 bytes.
 * Little-endian. FC differences from the DAQ: one directory (0 = card root),
 * LIST shows and READ accepts only irec*.csv names, and the last DATA frame
 * of an OK READ that carries data sets SD_FILE_DATA_CRC and appends the zlib
 * CRC-32 of all data bytes of that READ answer (4 bytes LE, after the data). */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define SD_FILE_STREAM_PORT 4511U
#define SD_FILE_OP_LIST 0x21U
#define SD_FILE_OP_READ 0x22U
#define SD_FILE_OP_CANCEL 0x23U
#define SD_FILE_RESPONSE_FLAG 0x80U

#define SD_FILE_STATUS_OK 0U
#define SD_FILE_STATUS_BAD_MESSAGE 1U
#define SD_FILE_STATUS_BAD_STATE 2U
#define SD_FILE_STATUS_BUSY 3U
#define SD_FILE_STATUS_NOT_FOUND 4U
#define SD_FILE_STATUS_IO_ERROR 5U
#define SD_FILE_STATUS_END 6U
#define SD_FILE_STATUS_BAD_OFFSET 7U

#define SD_FILE_DIR_ROOT 0U        /* the only FC directory: card root */

#define SD_FILE_NAME_MAX 63U
#define SD_FILE_FRAME_MAX 117U
#define SD_FILE_BLOCK_BYTES 512U
#define SD_FILE_DATA_HEADER 11U
#define SD_FILE_DATA_MAX 100U
#define SD_FILE_CRC_BYTES 4U
#define SD_FILE_LIST_HEADER 12U

#define SD_FILE_ENTRY_OPEN 0x01U   /* still open for writing; size may grow */
#define SD_FILE_DATA_LAST 0x01U    /* last frame for this READ */
#define SD_FILE_DATA_CRC 0x02U     /* CRC-32 of the READ answer follows the data */

_Static_assert(SD_FILE_DATA_HEADER + SD_FILE_DATA_MAX + SD_FILE_CRC_BYTES <= SD_FILE_FRAME_MAX, "DATA frame bound");
_Static_assert(SD_FILE_LIST_HEADER + SD_FILE_NAME_MAX <= SD_FILE_FRAME_MAX, "LIST frame bound");
_Static_assert(7U + SD_FILE_NAME_MAX <= SD_FILE_FRAME_MAX, "READ request bound");

typedef struct {
    uint8_t op;
    uint8_t dir;
    uint8_t name_len;
    uint16_t index;
    uint32_t offset;
    char name[SD_FILE_NAME_MAX + 1U];
} sd_file_request_t;

static inline void sd_file_put_u32(uint8_t *p, uint32_t v)
{ p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); p[2]=(uint8_t)(v>>16); p[3]=(uint8_t)(v>>24); }
static inline uint32_t sd_file_get_u32(const uint8_t *p)
{ return (uint32_t)p[0] | ((uint32_t)p[1]<<8) | ((uint32_t)p[2]<<16) | ((uint32_t)p[3]<<24); }

/* Local flight states (fcstructs.h): Startup 0, Landed 9, Recovery 10.
 * Network FLIGHT_STATE: Startup 0, Idle 1. Reads are allowed after landing,
 * or on the ground before Postinit while the network is Startup/Idle. Never
 * in Postinit, Armed, Launch..Reefing, or with an unknown network state. */
static inline bool sd_file_fc_state_allowed(uint8_t local, bool network_known, uint8_t network)
{
    return local == 9U || local == 10U || (local == 0U && network_known && network <= 1U);
}

/* Plain short leaf names only: no separators, no dot-only names. */
static inline bool sd_file_name_valid(const char *name, size_t len)
{
    if (!name || len == 0U || len > SD_FILE_NAME_MAX) return false;
    bool other = false;
    for (size_t i = 0; i < len; ++i) {
        const char c = name[i];
        const bool ok = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
                        (c >= '0' && c <= '9') || c == '_' || c == '-' || c == '.';
        if (!ok) return false;
        if (c != '.') other = true;
    }
    return other;
}

/* FC logs: irec*.csv, case-insensitive (valid names only, so |0x20 is exact). */
static inline bool sd_file_fc_log_name(const char *name, size_t len)
{
    if (len < 8U || !sd_file_name_valid(name, len)) return false;
    for (size_t i = 0; i < 4U; ++i)
        if ((name[i] | 0x20) != "irec"[i] || (name[len - 4U + i] | 0x20) != ".csv"[i]) return false;
    return true;
}

/* Returns a status; on OK the request is fully validated. */
static inline uint8_t sd_file_parse_request(const uint8_t *p, size_t len, sd_file_request_t *out)
{
    if (!p || !out || len == 0U || len > SD_FILE_FRAME_MAX) return SD_FILE_STATUS_BAD_MESSAGE;
    sd_file_request_t r;
    memset(&r, 0, sizeof r);
    r.op = p[0];
    if (r.op == SD_FILE_OP_CANCEL) {
        if (len != 1U) return SD_FILE_STATUS_BAD_MESSAGE;
    } else if (r.op == SD_FILE_OP_LIST) {
        if (len != 4U || p[1] != SD_FILE_DIR_ROOT) return SD_FILE_STATUS_BAD_MESSAGE;
        r.index = (uint16_t)(p[2] | (p[3] << 8));
    } else if (r.op == SD_FILE_OP_READ) {
        if (len < 8U) return SD_FILE_STATUS_BAD_MESSAGE;
        r.offset = sd_file_get_u32(p + 1);
        r.name_len = p[6];
        if (p[5] != SD_FILE_DIR_ROOT || (size_t)r.name_len + 7U != len ||
            !sd_file_fc_log_name((const char *)(p + 7), r.name_len)) return SD_FILE_STATUS_BAD_MESSAGE;
        memcpy(r.name, p + 7, r.name_len);
    } else {
        return SD_FILE_STATUS_BAD_MESSAGE;
    }
    *out = r;
    return SD_FILE_STATUS_OK;
}

/* LIST answer: [0xA1][status][dir][index u16][size u32][flags][name_len][0][name]. */
static inline size_t sd_file_encode_list(uint8_t *dst, uint8_t status, uint16_t index, uint32_t file_size,
                                         uint8_t flags, const char *name, size_t name_len)
{
    if (status != SD_FILE_STATUS_OK) name_len = file_size = flags = 0U;
    if (name_len > SD_FILE_NAME_MAX) return 0U;
    dst[0] = SD_FILE_OP_LIST | SD_FILE_RESPONSE_FLAG; dst[1] = status; dst[2] = SD_FILE_DIR_ROOT;
    dst[3] = (uint8_t)index; dst[4] = (uint8_t)(index >> 8); sd_file_put_u32(dst + 5, file_size);
    dst[9] = flags; dst[10] = (uint8_t)name_len; dst[11] = 0U;
    if (name_len) memcpy(dst + SD_FILE_LIST_HEADER, name, name_len);
    return SD_FILE_LIST_HEADER + name_len;
}

/* DATA frame: [0xA2][status][flags][offset u32][file_size u32][data][crc u32 if flags&CRC].
 * dst must hold SD_FILE_FRAME_MAX bytes. */
static inline size_t sd_file_encode_data(uint8_t *dst, uint8_t status, uint8_t flags, uint32_t offset,
                                         uint32_t file_size, const uint8_t *data, size_t len, uint32_t crc)
{
    if (len > SD_FILE_DATA_MAX || (len && !data)) return 0U;
    dst[0] = SD_FILE_OP_READ | SD_FILE_RESPONSE_FLAG; dst[1] = status; dst[2] = flags;
    sd_file_put_u32(dst + 3, offset); sd_file_put_u32(dst + 7, file_size);
    if (len) memcpy(dst + SD_FILE_DATA_HEADER, data, len);
    len += SD_FILE_DATA_HEADER;
    if (flags & SD_FILE_DATA_CRC) { sd_file_put_u32(dst + len, crc); len += SD_FILE_CRC_BYTES; }
    return len;
}
#endif
