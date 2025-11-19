/*
 * Synchronous accelerometer driver over SPI.
 */

#include <stdint.h>
#include "accel.h"
#include "stdio.h"
#include "stm32h5xx_hal.h"
#include "stm32h5xx_hal_def.h"
#include "stm32h5xx_hal_spi.h"

/* Write 1 byte to a register address (the same as for gyroscope) */
static inline HAL_StatusTypeDef accel_write_reg(SPI_HandleTypeDef *hspi,
                                                uint8_t reg, uint8_t data) {
  uint8_t buf[2] = {ACCEL_CMD_WRITE(reg), data};
  ACCEL_CS_LOW();
  HAL_StatusTypeDef st = HAL_SPI_Transmit(hspi, buf, sizeof(buf), HAL_MAX_DELAY);
  ACCEL_CS_HIGH();
  return st;
}

/* Single byte read from given register, must ignore dummy byte (datasheet ch. 6.1.2) */
static inline HAL_StatusTypeDef accel_read_reg(SPI_HandleTypeDef *hspi,
                                               uint8_t reg, uint8_t *data) {
  if (!data) return HAL_ERROR;

  uint8_t tx[3] = {ACCEL_CMD_READ(reg), 0x00, 0x00};
  uint8_t rx[3];

  ACCEL_CS_LOW();
  HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(hspi, tx, rx, sizeof(tx), HAL_MAX_DELAY);
  ACCEL_CS_HIGH();

  if (st == HAL_OK) *data = rx[2];
  return st;
}

/* Initialize and configure the accelerometer (several datasheet chapters) */
HAL_StatusTypeDef accel_init(SPI_HandleTypeDef *hspi)
{
  HAL_StatusTypeDef status;
  uint8_t id = 0x00;

  ACCEL_CS_LOW();
  HAL_Delay(1);
  ACCEL_CS_HIGH();
  HAL_Delay(50);

  /* Soft reset (datasheet ch. 4.8) */
  status = accel_write_reg(hspi, ACCEL_RESET, ACCEL_RESET_VAL);
  if (status != HAL_OK) return status;
  HAL_Delay(50);

  /* Dummy read (result ignored) */ 
  status = accel_read_reg(hspi, ACCEL_CHIP_ADDR, &id);

  /* WHO_AM_I should be 0x1E at 0x00 (taken from gyroscope driver) */ 
  status = accel_read_reg(hspi, ACCEL_CHIP_ADDR, &id);
  if (status != HAL_OK) return status;
  if (id != ACCEL_CHIP_ID) {
    printf("Accel WHOAMI mismatch: 0x%02X (exp 0x1E)\n", id);
    return HAL_ERROR;
  }

  /* Bandwith of low pass filter config to normal and ODR set to 1600hz (datasheet ch. 5.3.10) */
  status = accel_write_reg(hspi, ACCEL_CONF, NORMAL_1600HZ);
  if (status != HAL_OK) return status;

  /* Enable active mode (datasheet ch. 5.3.20) */
  status = accel_write_reg(hspi, ACCEL_PWR_CONF, ACTIVE_MODE);
  if (status != HAL_OK) return status;
  HAL_Delay(50);

  /* Power on (enter normal mode) (datasheet ch. 3 & 4.1.1) */
  status = accel_write_reg(hspi, ACCEL_PWR_CTRL, ACCEL_ON);
  if (status != HAL_OK) return status;
  HAL_Delay(450);

  /* Set range to ±24g (datasheet 5.3.11) */
  status = accel_write_reg(hspi, ACCEL_RANGE, ACCEL_RANGE_24g);
  if (status != HAL_OK) return status;
  HAL_Delay(30);

  return HAL_OK;
}

/* Read the accelermoter axes data (datasheet ch. 5.3.4 & 6.1.2) */
HAL_StatusTypeDef accel_read(SPI_HandleTypeDef *hspi, accel_data_t *data) {
  uint8_t tx[ACCEL_BUF_SIZE] = {[0] = ACCEL_CMD_READ(ACCEL_X_LSB),
                                     0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t rx[ACCEL_BUF_SIZE];

  ACCEL_CS_LOW();
  HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(hspi, tx, rx, sizeof(rx), HAL_MAX_DELAY);
  ACCEL_CS_HIGH();
  
  if (st == HAL_OK) {
    data->x = (float)((rx[3] << 8) | rx[2]) * MG;
    data->y = (float)((rx[5] << 8) | rx[4]) * MG;
    data->z = (float)((rx[7] << 8) | rx[6]) * MG;
  }
  return st;
}

/* Perform self-test, write raw data to out, and reinitialize accelerometer (datasheet ch. 4.6.1) */
HAL_StatusTypeDef accel_selftest(SPI_HandleTypeDef *hspi, accel_data_t *out) {
  HAL_StatusTypeDef st;
  accel_data_t data_p;
  accel_data_t data_n;

  st = accel_write_reg(hspi, ACCEL_CONF, ACCEL_TEST_CONF);
  if (st != HAL_OK) return st;
  HAL_Delay(5);

  st = accel_write_reg(hspi, ACCEL_SELF_TEST, ACCEL_POS_POL);
  if (st != HAL_OK) return st;
  HAL_Delay(55);

  st = accel_read(hspi, &data_p);
  if (st != HAL_OK) return st;

  st = accel_write_reg(hspi, ACCEL_SELF_TEST, ACCEL_NEG_POL);
  if (st != HAL_OK) return st;
  HAL_Delay(55);

  st = accel_read(hspi, &data_n);
  if (st != HAL_OK) return st;

  st = accel_write_reg(hspi, ACCEL_SELF_TEST, ACCEL_TEST_OFF);
  if (st != HAL_OK) return st;

  out->x = (float)(data_p.x - data_n.x);
  out->y = (float)(data_p.y - data_n.y);
  out->z = (float)(data_p.z - data_n.z);
  
  return accel_init(hspi);
}