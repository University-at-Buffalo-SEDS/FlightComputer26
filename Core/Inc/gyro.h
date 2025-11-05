#pragma once
#include "main.h"
#include "stm32h5xx_hal.h"

/* --- BMI088 Gyro Register Addresses (datasheet) --- */
#define GYRO_CHIP_ID 0x00 // expect 0x0F
#define GYRO_RATE_X_LSB 0x02
#define GYRO_RATE_X_MSB 0x03
#define GYRO_RATE_Y_LSB 0x04
#define GYRO_RATE_Y_MSB 0x05
#define GYRO_RATE_Z_LSB 0x06
#define GYRO_RATE_Z_MSB 0x07
#define GYRO_INT_STAT_1 0x0A
#define GYRO_FIFO_STATUS 0x0E
#define GYRO_RANGE 0x0F
#define GYRO_BANDWIDTH 0x10
#define GYRO_LPM1 0x11
#define GYRO_SOFTRESET 0x14
#define GYRO_FIFO_CONFIG_0 0x3D
#define GYRO_FIFO_CONFIG_1 0x3E
#define GYRO_FIFO_DATA 0x3F
#define GYRO_RANGE_DPS GYRO_RANGE_2000DPS

/* WHO_AM_I value (gyro die) */
#define GYRO_CHIP_ID_VALUE 0x0F // BMI088 gyro WHOAMI
/* SPI command helpers (bit7=1 => read; bit7=0 => write) */
#define GYRO_CMD_READ(reg) ((uint8_t)((reg) | 0x80u))
#define GYRO_CMD_WRITE(reg) ((uint8_t)((reg) & ~0x80u))

/* Data buffer size */
#define GYRO_BUF_SIZE 8

typedef enum {
  GYRO_RANGE_2000DPS = 0x00,
  GYRO_RANGE_1000DPS = 0x01,
  GYRO_RANGE_500DPS = 0x02,
  GYRO_RANGE_250DPS = 0x03,
  GYRO_RANGE_125DPS = 0x04
} GyroRange;

typedef enum {
  GYRO_BW_523HZ_ODR_2000HZ = 0x00,
  GYRO_BW_230HZ_ODR_2000HZ = 0x01,
  GYRO_BW_116HZ_ODR_1000HZ = 0x02,
  GYRO_BW_47HZ_ODR_400HZ = 0x03,
  GYRO_BW_23HZ_ODR_200HZ = 0x04,
  GYRO_BW_12HZ_ODR_100HZ = 0x05,
  GYRO_BW_64HZ_ODR_200HZ = 0x06,
  GYRO_BW_32HZ_ODR_100HZ = 0x07,
} GyroBandwidth;

typedef struct {
  int16_t rate_x;
  int16_t rate_y;
  int16_t rate_z;
} gyro_data_t;

HAL_StatusTypeDef gyro_write_register(SPI_HandleTypeDef *hspi, uint8_t reg,
                                      uint8_t value);
HAL_StatusTypeDef gyro_read_register(SPI_HandleTypeDef *hspi, uint8_t reg,
                                     uint8_t *data);
HAL_StatusTypeDef gyro_read_buffer(SPI_HandleTypeDef *hspi, uint8_t start_reg,
                                   uint8_t *dst, uint16_t len);
HAL_StatusTypeDef gyro_init(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef gyro_read(SPI_HandleTypeDef *hspi, gyro_data_t *gyro_data);

float gyro_raw_to_dps(int16_t raw);
void  gyro_convert_to_dps(const gyro_data_t *raw, float *x_dps, float *y_dps, float *z_dps);