/*
 * BMI088 Gyroscope configuration and API.
 */

#ifndef GYROSCOPE_H
#define GYROSCOPE_H

#include "main.h"


/* ------ Register map ------ */

#define GYRO_CHIP_ID        0x00
#define GYRO_INT_STAT_1     0x0A
#define GYRO_FIFO_STATUS    0x0E
#define GYRO_RANGE          0x0F
#define GYRO_BANDWIDTH      0x10
#define GYRO_LPM1           0x11
#define GYRO_SOFTRESET      0x14
#define GYRO_FIFO_CONFIG_0  0x3D
#define GYRO_FIFO_CONFIG_1  0x3E
#define GYRO_FIFO_DATA      0x3F

#define GYRO_RATE_X_LSB     0x02
#define GYRO_RATE_X_MSB     0x03
#define GYRO_RATE_Y_LSB     0x04
#define GYRO_RATE_Y_MSB     0x05
#define GYRO_RATE_Z_LSB     0x06
#define GYRO_RATE_Z_MSB     0x07

#define GYRO_CHIP_ID_VALUE  0x0F


/* ------ Helper definitions ------ */

#define gyro_cmd_read(reg)  ((uint8_t)((reg) | 0x80u))
#define gyro_cmd_write(reg) ((uint8_t)((reg) & ~0x80u))

#define gyro_cs_low()                                               \
  HAL_GPIO_WritePin(CS_GYRO_GPIO_Port, CS_GYRO_Pin, GPIO_PIN_RESET)
#define gyro_cs_high()                                              \
  HAL_GPIO_WritePin(CS_GYRO_GPIO_Port, CS_GYRO_Pin, GPIO_PIN_SET)

#define GYRO_DMA_BUF_SIZE 8
#define GYRO_WAIT_MS      30

/* ------ Enumerated configuration ------ */

enum gyro_range {
  Gyro_Range_2000Dps = 0x00,
  Gyro_Range_1000Dps = 0x01,
  Gyro_Range_500Dps  = 0x02,
  Gyro_Range_250Dps  = 0x03,
  Gyro_Range_125Dps  = 0x04,

  Gyro_Range_Entries = 0x05
};

enum gyro_bandwidth {
  Gyro_532Hz_ODR_2000Hz = 0x00,
  Gyro_230Hz_ODR_2000Hz = 0x01,
  Gyro_116Hz_ODR_1000Hz = 0x02,
  Gyro_47Hz_ODR_400Hz   = 0x03,
  Gyro_23Hz_ODR_200Hz   = 0x04,
  Gyro_12Hz_ODR_100Hz   = 0x05,
  Gyro_64Hz_ODR_200Hz   = 0x06,
  Gyro_32Hz_ODR_100Hz   = 0x07,
};

struct gyro_config {
  enum gyro_range rng;
  enum gyro_bandwidth bw;
};

struct gyro_raw {
  int16_t x, y, z;
};


/* ------ Public API ------ */

struct coords;

HAL_StatusTypeDef gyro_init(SPI_HandleTypeDef *hspi, const struct gyro_config *conf);
HAL_StatusTypeDef gyro_read(SPI_HandleTypeDef *hspi, struct coords *buf);

HAL_StatusTypeDef gyro_read_raw(SPI_HandleTypeDef *hspi, struct gyro_raw *data);
void gyro_convert(const struct gyro_raw *raw, struct coords *buf);


#endif // GYROSCOPE_H