// Includes consumer API

#include <stdint.h>
#include <stdatomic.h>

#define SENSOR_BUF_SIZE 8

/* Typedefs */

typedef enum {
  NONE = 0,
  BAROMETER = 1,
  GYROSCOPE = 2,
  ACCELEROMETER = 3,
} expected_e;

typedef struct {
  expected_e type;
  union {
    float baro[2];
    int16_t gyro[3];
    float accel[3];
  } data;
} payload_t;

typedef enum {
  DMA_OK,
  DMA_BUSY,
  DMA_EMPTY,
  DMA_GENERR
} dma_e;

typedef struct {
  atomic_uint_fast8_t w;
  atomic_uint_fast8_t r;
  expected_e type[2];
} dma_t;

/* Globals */

/*
 * Reads the latest payload entry.
 * Returns DMA_OK (0) on success and > 0 on failure.
 */
inline dma_e dma_read_latest(payload_t *buf);