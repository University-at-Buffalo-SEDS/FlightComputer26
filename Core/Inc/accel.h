/*
 * Synchronous accelerometer driver over SPI.
 */

#pragma once
#include <stdint.h>
#include "main.h"
#include "stm32h5xx_hal_spi.h"

// Type definitions

typedef enum {
    ACCEL_ON = 0x04,
    ACCEL_OFF = 0x00
} accel_pwr_ctrl_e;

typedef enum {
    ACTIVE_MODE = 0x00,
    SUSPEND_MODE = 0x03
} accel_pwr_conf_e;

typedef enum {
    OSR4_100HZ = ((0x08 << 4) | 0x08),
    OSR4_400HZ = ((0x08 << 4) | 0x0A),
    OSR2_50HZ = ((0x09 << 4) | 0x07),
    OSR2_100HZ = ((0x09 << 4) | 0x08),
    NORMAL_100HZ = ((0x0A << 4) | 0x08),
    NORMAL_200HZ = ((0x0A << 4) | 0x09),
    NORMAL_400HZ = ((0x0A << 4) | 0x0A),
    NORMAL_800HZ = ((0x0A << 4) | 0x0B),
    NORMAL_1600HZ = ((0x0A << 4) | 0x0C)
} accel_conf_e;

typedef enum {
    ACCEL_RANGE_3g = 0x00,
    ACCEL_RANGE_6g = 0x01,
    ACCEL_RANGE_12g = 0x02,
    ACCEL_RANGE_24g = 0x03
} accel_range_e;

typedef struct {
    float x;
    float y;
    float z;
} accel_data_t;

// Register maps
#define ACCEL_RESET         0x7E
#define ACCEL_CHIP_ADDR     0x00
#define ACCEL_CHIP_ID       0x1E
#define ACCEL_CONF          0x40
#define ACCEL_PWR_CTRL      0x7D
#define ACCEL_PWR_CONF      0x7C
#define ACCEL_RANGE         0x41

#define ACCEL_Z_MSB         0x17
#define ACCEL_Z_LSB         0x16
#define ACCEL_Y_MSB         0x15
#define ACCEL_Y_LSB         0x14
#define ACCEL_X_MSB         0x13
#define ACCEL_X_LSB         0x12

// Constants
#define ACCEL_RESET_VAL     0xB6
#define ACCEL_BUF_SIZE      8

// Self-test constants
#define ACCEL_SELF_TEST     0x6D
#define ACCEL_POS_POL       0x0D
#define ACCEL_NEG_POL       0x09
#define ACCEL_TEST_OFF      0x00
#define ACCEL_TEST_CONF     0xA7
#define ACCEL_TEST_WAIT_MS  50

// Helper definitions

#define MG ((float)(1 << (ACCEL_RANGE_24g + 0x01)) / 32768.0f * 1.5f)

#define ACCEL_CMD_READ(reg)  ((uint8_t)((reg) | 0x80u))
#define ACCEL_CMD_WRITE(reg) ((uint8_t)((reg) & 0x7F))

#define ACCEL_CS_LOW()    { HAL_GPIO_WritePin(CS_ACCEL_GPIO_Port, CS_ACCEL_Pin, GPIO_PIN_RESET); }
#define ACCEL_CS_HIGH()   { HAL_GPIO_WritePin(CS_ACCEL_GPIO_Port, CS_ACCEL_Pin, GPIO_PIN_SET);   }

// Functions

/* Configure the accelerometer */
HAL_StatusTypeDef accel_init(SPI_HandleTypeDef *hspi);

/* Read acceleration data and convert it to milligravity */
HAL_StatusTypeDef accel_read(SPI_HandleTypeDef *hspi, accel_data_t *accelData);

/* Perform self-test, write readings to out, and reinitialize accelerometer. */
HAL_StatusTypeDef accel_selftest(SPI_HandleTypeDef *hspi, accel_data_t *out);