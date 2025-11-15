#pragma once
#include <stdint.h>
#include "main.h"
#include "stm32h5xx_hal_spi.h"

// REGISTER MAPPING
#define ACCEL_RESET         0x7E
#define ACCEL_CHIP_ADDR     0x00
#define ACCEL_CHIP_ID       0x1E
#define ACCEL_CONF          0x40
#define ACCEL_POWER_CTRL    0x7D
#define ACCEL_POWER_CONF    0x7C
#define ACCEL_RANGE         0x41

#define ACCEL_Z_MSB         0x17
#define ACCEL_Z_LSB         0x16
#define ACCEL_Y_MSB         0x15
#define ACCEL_Y_LSB         0x14
#define ACCEL_X_MSB         0x13
#define ACCEL_X_LSB         0x12

// ACCEL CONFIGS
#define ACCEL_RESET_VAL     0xB6
#define ACCEL_RANGE_VAL     0x03
#define ACCEL_POWER_VAL     0x00
#define ACCEL_CONF_VAL      ((0x0A << 4) | 0x0C)
#define ACCEL_BUF_SIZE      6

// Self-testing
#define ACC_SELF_TEST       0x6D
#define ACC_POS_POL         0x0D
#define ACC_NEG_POL         0x09
#define ACC_TEST_OFF        0x00
#define ACC_TEST_CONF       0xA7
#define ACC_TEST_WAIT_MS    50

#define MG ((float)(1u << (ACCEL_RANGE_VAL + 0x01)) / (float)(1 << 15) * 1.5f)

#define ACCEL_CMD_READ(reg)  ((uint8_t)((reg) | 0x80u))
#define ACCEL_CMD_WRITE(reg) ((uint8_t)((reg) & 0x7F))

#define ACCEL_CS_LOW()    { HAL_GPIO_WritePin(CS_ACCEL_GPIO_Port, CS_ACCEL_Pin, GPIO_PIN_RESET); }
#define ACCEL_CS_HIGH()   { HAL_GPIO_WritePin(CS_ACCEL_GPIO_Port, CS_ACCEL_Pin, GPIO_PIN_SET);   }

// Type definitions

typedef enum {
    POWER_ON = 0x04,
    POWER_OFF = 0x00
} accel_power;

typedef enum {
    ACCEL_RANGE_3g = 0x00,
    ACCEL_RANGE_6g = 0x01,
    ACCEL_RANGE_12g = 0x02,
    ACCEL_RANGE_24g = 0x03
} AccelRange;

typedef struct {
    float x;
    float y;
    float z;
} accel_data_t;

// Functions

/* Configure the accelerometer */
HAL_StatusTypeDef accel_init(SPI_HandleTypeDef *hspi);

/* Read X axis data */
HAL_StatusTypeDef accel_read(SPI_HandleTypeDef *hspi, accel_data_t *accelData);

/* Performs self-test, writes data to out, and reinitializes the device. */
HAL_StatusTypeDef accel_selftest(SPI_HandleTypeDef *hspi, accel_data_t *out);