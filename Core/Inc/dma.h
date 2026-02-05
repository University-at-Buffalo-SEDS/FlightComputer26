/*
 * Direct Memory Access configuration and API.
 */

#ifndef DMA_H
#define DMA_H


/* ------ DMA / sensor defs ------  */

#define SENSOR_BUF_SIZE 8

#define DMA_RX_NULL UINT8_MAX

#define BARO_DONE 0x01
#define GYRO_DONE 0x02
#define ACCL_DONE 0x04

#define RX_DONE 0x07


/* ------ Serial type attributes ------ */

#define serial __attribute__((packed, aligned(4)))


/* ------ Containers ------ */

/// Discriminants must comply with decode_ptr() dev[].
enum device {
  BAROMETER = 0,
  GYROSCOPE = 1,
  ACCELEROMETER = 2,

  DEVICES = 3,
};

/* Generic measurement containers */
struct serial coords { float x, y, z; }; /* Order matters */
struct serial baro { float alt, p, t; }; /* Order matters */

/// Transferable raw data unit
struct serial measurement { /* Order matters */
  struct coords gyro;
  struct baro baro;
  struct coords accl;
};


/* ------ Public API ------ */

/// Fetches whatever is available in a free (tm) DMA buffer.
/// Returns 1 when all 3 sensor buckets have been filled,
/// (accumulates acrosss calls), 0 otherwise, and -1 on bag argument.
int dma_try_fetch(struct measurement *buf);

/// Aggregates sensor compensation functions.
/// Run before reporting and publishing data.
void compensate(struct measurement *buf);


#endif // DMA_H