/*
 * Direct Memory Access header and API.
 */

#ifndef DMA_H
#define DMA_H

#include <stdint.h>


/* ------ DMA / sensor defs ------  */

#define SENSOR_BUF_SIZE 8

#define DMA_RX_NULL UINT8_MAX

#define RX0_DONE 0x07
#define RX1_DONE (RX0_DONE << 4)

#define FINISH_ACTIVE_TRANSFERS 15


/* ------ Containers ------ */

/// Discriminants must comply with decode_ptr() dev[].
typedef enum {
  BAROMETER = 0,
  GYROSCOPE = 1,
  ACCELEROMETER = 2,
} device_e;

/* Generic measurement containers */
typedef struct { float x, y, z; } coords_t;
typedef struct { float alt, temp; } baro_t;
typedef struct { int16_t x, y, z; } gyro_t;

/// Transferable raw data unit
typedef struct {
  baro_t baro;
  gyro_t gyro;
  coords_t accl;
} payload_t;

/// Global return status
typedef enum {
  DMA_OK,
  DMA_WAIT,
  DMA_BADARG
} dma_e;


/* ------ Public API ------ */

/// Tries to fetch data from DMA rxbuf into provided buffer.
/// Context: sensor task.
dma_e dma_try_fetch(payload_t *restrict buf);


#endif // DMA_H