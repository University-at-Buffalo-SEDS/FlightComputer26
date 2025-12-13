// Includes consumer API

#include <stdint.h>

/* Ring properties */

#define RING_SIZE 16
#define RING_MASK RING_SIZE - 1

/* Status reporting to consumers */

typedef enum {
  DMA_OK      = 0,
  RING_EMPTY  = -1,
} dma_e;

/* Packet queue typedefs */

typedef enum {
  NONE,
  BAROMETER,
  GYROSCOPE,
  ACCELEROMETER,
} expected_e;

typedef struct {
  expected_e type;
  union {
    float baro[3];
    int16_t gyro[3];
    float accel[3];
  } data;
} payload_t;

/* Globals */

/*
 * Dequeue the oldest element of the ring and advance tail.
 */
inline dma_e dma_ring_dequeue_oldest(payload_t *buf);

/*
 * Copy but not dequeue the oldest element of the ring.
 */
inline dma_e dma_ring_copy_oldest(payload_t *buf);