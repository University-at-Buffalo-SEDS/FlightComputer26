#include "stm32h5xx_hal.h"
#include "accel.h"
#include "stdio.h"
#include "stm32h5xx_hal_def.h"
#include "stm32h5xx_hal_spi.h"
#include <stdint.h>

/* Write 1 byte to a register address */
static inline HAL_StatusTypeDef accel_write_reg(SPI_HandleTypeDef *hspi,
                                                uint8_t reg, uint8_t data) {
  uint8_t buf[2] = {ACCEL_CMD_WRITE(reg), data};
  ACCEL_CS_LOW();
  HAL_StatusTypeDef st = HAL_SPI_Transmit(hspi, buf, sizeof(buf), HAL_MAX_DELAY);
  ACCEL_CS_HIGH();
  return st;
}

/* Single Byte Read from given register, must ignore dummy byte */
static inline HAL_StatusTypeDef accel_read_reg(SPI_HandleTypeDef *hspi,
                                               uint8_t reg, uint8_t *data) {
  if (!data) return HAL_ERROR;

  uint8_t tx[3] = {ACCEL_CMD_READ(reg), 0x00, 0x00};
  uint8_t rx[3] = {0x00, 0x00, 0x00};

  ACCEL_CS_LOW();
  HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(hspi, tx, rx, sizeof(tx), HAL_MAX_DELAY);
  ACCEL_CS_HIGH();

  if (st == HAL_OK) *data = rx[2];
  return st;
}

/* Configure the accelerometer */
HAL_StatusTypeDef accel_init(SPI_HandleTypeDef *hspi)
{
  HAL_StatusTypeDef status;
  uint8_t id = 0x00;

  ACCEL_CS_LOW();
  HAL_Delay(1);
  ACCEL_CS_HIGH();
  HAL_Delay(50);

  /* Soft reset */
  status = accel_write_reg(hspi, ACCEL_RESET, ACCEL_RESET_VAL);
  if (status != HAL_OK) return status;
  HAL_Delay(50);

  /* Dummy read */ 
  status = accel_read_reg(hspi, ACCEL_CHIP_ADDR, &id);

  /* WHO_AM_I should be 0x1E at 0x00 (taken directly from Gyro Driver) */ 
  status = accel_read_reg(hspi, ACCEL_CHIP_ADDR, &id);
  if (status != HAL_OK) return status;
  if (id != ACCEL_CHIP_ID) {
    printf("Accel WHOAMI mismatch: 0x%02X (exp 0x1E)\n", id);
    return HAL_ERROR;
  }

  /* Bandwith of low pass filter config to normal and ODR set to 1600hz */
  status = accel_write_reg(hspi, ACCEL_CONF, ACCEL_CONF_VAL);
  if (status != HAL_OK) return status;

  /* Enable active mode */
  status = accel_write_reg(hspi, ACCEL_POWER_CONF, ACCEL_POWER_VAL);
  if (status != HAL_OK) return status;
  HAL_Delay(50);

  /* Power on (enter normal mode) */
  status = accel_write_reg(hspi, ACCEL_POWER_CTRL, POWER_ON);
  if (status != HAL_OK) return status;
  HAL_Delay(450);

  /* Set range to ±24g */
  status = accel_write_reg(hspi, ACCEL_RANGE, ACCEL_RANGE_VAL);
  if (status != HAL_OK) return status;
  HAL_Delay(30);

  return HAL_OK;
}

/* Read the accelermoter axis data */
HAL_StatusTypeDef accel_read(SPI_HandleTypeDef *hspi, accel_data_t *data) {
  uint8_t tx[ACCEL_BUF_SIZE + 1] = {[0] = ACCEL_CMD_READ(ACCEL_X_LSB),
                                     0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t rx[ACCEL_BUF_SIZE + 1];

  ACCEL_CS_LOW();
  HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(hspi, tx, rx, sizeof(rx), HAL_MAX_DELAY);
  ACCEL_CS_HIGH();
  
  if (st == HAL_OK) {
    data->x = (float)((rx[2] << 8) | rx[1]) * MG;
    data->y = (float)((rx[4] << 8) | rx[3]) * MG;
    data->z = (float)((rx[6] << 8) | rx[5]) * MG;
  }
  return st;
}

/* Performs self-test, writes raw data to out, and reinitializes the device. */
HAL_StatusTypeDef accel_selftest(SPI_HandleTypeDef *hspi, accel_data_t *out) {
  accel_data_t data_p;
  accel_data_t data_n;

  accel_write_reg(hspi, ACCEL_CONF, ACC_TEST_CONF);
  HAL_Delay(5);

  accel_write_reg(hspi, ACC_SELF_TEST, ACC_POS_POL);
  HAL_Delay(55);
  accel_read(hspi, &data_p);

  accel_write_reg(hspi, ACC_SELF_TEST, ACC_NEG_POL);
  HAL_Delay(55);
  accel_read(hspi, &data_n);

  accel_write_reg(hspi, ACC_SELF_TEST, ACC_TEST_OFF);
  out->x = (float)(data_p.x - data_n.x);
  out->y = (float)(data_p.y - data_n.y);
  out->z = (float)(data_p.z - data_n.z);
  
  return accel_init(hspi);
}