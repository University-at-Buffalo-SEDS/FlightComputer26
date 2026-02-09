/*
 * Direct Memory Access configuration and API.
 */

#ifndef DMA_H
#define DMA_H

#include "platform.h"


/* ------ DMA / sensor defs ------  */

#define SENSOR_BUF_SIZE 8

#define DMA_RX_NULL UINT8_MAX

#define BARO_DONE 0x01
#define GYRO_DONE 0x02
#define ACCL_DONE 0x04

#define RX_DONE 0x07


/* ------ Containers ------ */

/* Discriminants must comply with decode_ptr() dev[]. */

enum device {
  Sensor_Baro,
  Sensor_Gyro,
  Sensor_Accl,

  Sensors
};

struct serial coords { float x, y, z; };
struct serial baro { float alt, p, t; };

struct serial measurement {
  struct coords gyro;
  union {
    struct coords accl;
    struct coords gps;
  } d;
  struct baro baro;
};


/* ------ Public API ------ */

/// Fetches whatever is available in a free (tm) DMA buffer.
/// Returns 1 when all required sensor buckets have been filled,
/// (accumulates acrosss calls), 0 otherwise, and -1 on bag argument.
int dma_try_fetch(struct measurement *buf, fu8 skip_mask);

/// Aggregates sensor compensation functions.
/// Run before reporting and publishing data.
void compensate(struct measurement *buf, fu8 skip_mask);


#endif // DMA_H