/*
 * Direct Memory Access header and API.
 */

#ifndef DMA_H
#define DMA_H

#include <stdint.h>


/* ------ DMA / sensor defs ------  */

#define SENSOR_BUF_SIZE 8

#define DMA_RX_NULL UINT8_MAX

#define BARO_DONE 0x01
#define GYRO_DONE 0x02
#define ACCL_DONE 0x04

#define RX_DONE 0x07


/* ------ Containers ------ */

/// Discriminants must comply with decode_ptr() dev[].
enum device {
  BAROMETER = 0,
  GYROSCOPE = 1,
  ACCELEROMETER = 2,

  DEVICES = 3,
};

/* Generic measurement containers */
struct coords { float x, y, z; };
struct baro { float p, t, alt; };

/// Transferable raw data unit
struct measurement {
  struct baro baro;
  struct coords gyro, accl;
};


/* ------ Public API ------ */

/// Fetches whatever is available in a free (tm) DMA buffer.
/// Returns 1 when all 3 sensor buckets have been filled,
/// (accumulates acrosss calls), 0 otherwise, and -1 on bag argument.
int dma_try_fetch(struct measurement *buf);


#endif // DMA_H