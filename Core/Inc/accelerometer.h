/*
 * BMI088 Accelerometer configuration and API.
 */

#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

#include "main.h"


/* ------ Register maps ------ */

#define ACCL_RESET         0x7E
#define ACCL_CHIP_ADDR     0x00
#define ACCL_CHIP_ID       0x1E
#define ACCL_CONF          0x40
#define ACCL_PWR_CTRL      0x7D
#define ACCL_PWR_CONF      0x7C
#define ACCL_RANGE         0x41
#define ACCL_INT_CONF      0x53
#define ACCL_INT_MAP       0x58

#define ACCL_Z_MSB         0x17
#define ACCL_Z_LSB         0x16
#define ACCL_Y_MSB         0x15
#define ACCL_Y_LSB         0x14
#define ACCL_X_MSB         0x13
#define ACCL_X_LSB         0x12

/* 0000 1010 : INT1 to active high PP */
#define ACCL_INT_CONF_VAL  0x0A

/* 0000 0100 : INT1 mapped to data ready */
#define ACCL_INT_MAP_VAL   0x04

/* ------ Enumerated configuration ------ */

enum accl_pwr_ctrl {
  Accl_Off = 0x00,
  Accl_On = 0x04,
};

enum accl_pwr_conf {
  Active_Mode = 0x00,
  Suspend_Mode = 0x03,
};

enum accl_mode {
  OSR4_100Hz    = ((0x08 << 4) | 0x08),
  OSR4_400Hz    = ((0x08 << 4) | 0x0A),
  OSR2_50Hz     = ((0x09 << 4) | 0x07),
  OSR2_100Hz    = ((0x09 << 4) | 0x08),
  Normal_100Hz  = ((0x0A << 4) | 0x08),
  Normal_200Hz  = ((0x0A << 4) | 0x09),
  Normal_400Hz  = ((0x0A << 4) | 0x0A),
  Normal_800Hz  = ((0x0A << 4) | 0x0B),
  Normal_1600Hz = ((0x0A << 4) | 0x0C),
};

enum accl_range {
  Accl_Range_3g  = 0x00,
  Accl_Range_6g  = 0x01,
  Accl_Range_12g = 0x02,
  Accl_Range_24g = 0x03,
};

struct accl_config {
  enum accl_mode mode;
  enum accl_range rng;
};


/* ------ Constants ------ */

#define ACCL_RESET_VAL     0xB6
#define ACCL_DMA_BUF_SIZE  8

#define ACCL_SELF_TEST     0x6D
#define ACCL_POS_POL       0x0D
#define ACCL_NEG_POL       0x09
#define ACCL_TEST_OFF      0x00
#define ACCL_TEST_CONF     0xA7
#define ACCL_WAIT_MS       50


/* ------ Helper definitions ------ */

#define MG                                                             \
  ((float)(1u << (Accl_Range_24g + 0x01u)) / 32768.0f * 1.5f)

#define accl_cmd_read(reg)  ((uint8_t)((reg) | 0x80u))
#define accl_cmd_write(reg) ((uint8_t)((reg) & 0x7F))

#define accl_cs_low()                                                  \
  HAL_GPIO_WritePin(CS_ACCEL_GPIO_Port, CS_ACCEL_Pin, GPIO_PIN_RESET);
#define accl_cs_high()                                                 \
  HAL_GPIO_WritePin(CS_ACCEL_GPIO_Port, CS_ACCEL_Pin, GPIO_PIN_SET);


/* ------ Public API ------ */

struct coords;

HAL_StatusTypeDef accl_init(SPI_HandleTypeDef *hspi, const struct accl_config *conf);
HAL_StatusTypeDef accl_read(SPI_HandleTypeDef *hspi, struct coords *buf);
HAL_StatusTypeDef accl_test(SPI_HandleTypeDef *hspi, struct coords *buf, const struct accl_config *conf);


#endif // ACCELEROMETER_H