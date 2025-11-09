/**
 * Compile-time definitions related to DMA and ring buffer.
 */

#include "stm32h5xx_hal.h"
#include "stm32h5xx_hal_gpio.h"

#include "barometer.h"
#include "gyro.h"

// DMA interrupt pins for each device
#define ACCEL_INT_PIN_1 GPIO_PIN_4
#define ACCEL_INT_PIN_2 GPIO_PIN_5
#define GYRO_INT_PIN_1  GPIO_PIN_0
#define GYRO_INT_PIN_2  GPIO_PIN_1
#define BARO_INT_PIN    GPIO_PIN_7

// Ring properties
#define RING_SIZE 16
#define RING_MASK RING_SIZE - 1

// Packet queue typedefs

typedef enum {
  NONE,
  BAROMETER,
  GYROSCOPE,
  ACCELEROMETER,
} expected_e;

typedef struct {
  expected_e type;
  union {
    baro_data_t baro;
    gyro_data_t gyro;
    // accel_data_t accel;
  } data;
} payload_t;

// Global functions

/**
 * @brief Single function to dequeue and pass data to the library.
 * @param None
 * @retval A SedsResult indicating success or specific error.
 */
inline SedsResult dequeue_and_send_next();