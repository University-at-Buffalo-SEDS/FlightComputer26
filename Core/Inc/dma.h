/*
 * Direct Memory Access configuration and API.
 */

#ifndef DMA_H
#define DMA_H

#include "platform.h"


/* ------ Containers ------ */

/* Discriminants must comply with decode_ptr() dev[]. */

enum device {
  Sensor_Baro = 0,
  Sensor_Gyro = 1,
  Sensor_Accl = 2,

  Sensors
};

struct serial baro { float alt, p, t; };

struct serial measurement {
  struct coords gyro;
  union {
    struct coords accl;
    struct coords gps;
  } d;
  struct baro baro;
};


/* ------ DMA / sensor defs ------  */

#define SENSOR_BUF_SIZE 8

#define DMA_RX_NULL UINT8_MAX

#define RX_BARO (1u << Sensor_Baro)
#define RX_GYRO (1u << Sensor_Gyro)
#define RX_ACCL (1u << Sensor_Accl)

#define RX_DONE (RX_BARO | RX_GYRO | RX_ACCL)


/* ------ Public API ------ */

/*
 * Fetches whatever is available in a free (tm) DMA
 * buffer. Returns a relevancy mask, which has bits
 * set for devices whose data was fetched.
 */
fu8 dma_fetch(struct measurement *buf, fu8 skip_mask);

/*
 * Peeks at the IMU DMA buffers' statuses, and
 * fetches the latest pair of measurements
 * (accelerometer, gyroscope), if available.
 * Returns 1 on successful fetch and 0 otherwise. 
 */
fu8 dma_fetch_imu(struct coords *gyro, struct coords *accl);

/*
 * Peeks at the barometer DMA buffer status,
 * and fetches the latest pair of measurements
 * (temperature, pressure), if available.
 * Returns 1 on successful fetch and 0 otherwise. 
 */
fu8 dma_fetch_baro(struct baro *buf);


/* ------ Inline API ------ */

/*
 * Calls compensation functions from the barometer driver.
 */
static inline void compensate_baro(struct baro *buf)
{
  buf->t   = baro_comp_temp(buf->t);
  buf->p   = baro_comp_pres(buf->p);
  buf->alt = baro_calc_alt (buf->p);
}


/*
 * Performs correction on accelerometer measurements.
 */
static inline void compensate_accl(struct coords *buf)
{
  buf->x *= MG;
  buf->y *= MG;
  buf->z *= MG;
}

/*
 * Aggregates sensor compensation functions.
 */
static inline void compensate_all(struct measurement *buf)
{
  compensate_baro(&buf->baro);
  compensate_accl(&buf->d.accl);
}


#endif // DMA_H