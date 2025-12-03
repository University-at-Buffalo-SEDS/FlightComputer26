// Includes consumer API

#include "stm32h5xx_hal.h"
#include "stm32h5xx_hal_gpio.h"

#include "barometer.h"
#include "gyro.h"
#include "accel.h"

/* DMA interrupt pins for each device */

#define ACCEL_INT_PIN_1 GPIO_PIN_4
#define ACCEL_INT_PIN_2 GPIO_PIN_5
#define GYRO_INT_PIN_1  GPIO_PIN_0
#define GYRO_INT_PIN_2  GPIO_PIN_1
#define BARO_INT_PIN    GPIO_PIN_7

/* Ring properties */

#define RING_SIZE 16
#define RING_MASK RING_SIZE - 1

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
inline bool dma_ring_dequeue_oldest(payload_t *buf);

/*
 * Copy but not dequeue the oldest element of the ring.
 */
inline bool dma_ring_copy_oldest(payload_t *buf);