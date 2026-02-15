/*
 * BMI088 Accelerometer driver over SPI.
 */

#include "platform.h"
#include "accelerometer.h"


/* ------ Synchronous SPI helpers ------ */

/*
 * Synchronously read one value at the specified register.
 * Discards first dummy byte (datasheet section 6.1.2).
 */
static inline HAL_StatusTypeDef
accl_read_reg(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t *data)
{
  if (!data) {
    return HAL_ERROR;
  }

  HAL_StatusTypeDef st;
  uint8_t tx[3] = { accl_cmd_read(reg), 0x00, 0x00 };
  uint8_t rx[3];

  accl_cs_low();
  st = HAL_SPI_TransmitReceive(hspi, tx, rx, sizeof tx, HAL_MAX_DELAY);
  accl_cs_high();

  if (st == HAL_OK) {
    *data = rx[2];
  }

  return st;
}

/*
 * Synchronously write new value to a specified writeable register. 
 */
static inline HAL_StatusTypeDef
accl_write_reg(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t data)
{
  HAL_StatusTypeDef st;
  uint8_t buf[2] = { accl_cmd_write(reg), data };

  accl_cs_low();
  st = HAL_SPI_Transmit(hspi, buf, sizeof buf, HAL_MAX_DELAY);
  accl_cs_high();

  return st;
}

/*
 * Read the accelermoter axes data (datasheet section 5.3.4 & 6.1.2).
 */
HAL_StatusTypeDef
accl_read(SPI_HandleTypeDef *hspi, struct coords *data)
{
  HAL_StatusTypeDef st;
  uint8_t tx[ACCL_BUF_SIZE] = {[0] = accl_cmd_read(ACCL_X_LSB),
                               [1 ... 7] = 0x00};
  uint8_t rx[ACCL_BUF_SIZE];

  accl_cs_low();
  st = HAL_SPI_TransmitReceive(hspi, tx, rx, sizeof rx, HAL_MAX_DELAY);
  accl_cs_high();
  
  if (st == HAL_OK) {
    data->x = (float)((rx[3] << 8) | rx[2]) * MG;
    data->y = (float)((rx[5] << 8) | rx[4]) * MG;
    data->z = (float)((rx[7] << 8) | rx[6]) * MG;
  }

  return st;
}


/* ------ Initialization and testing ------ */

/*
 * Initializes accelerometer with specified parameters.
 * Mode and range values can be found in accelerometer.h.
 * Do not call from interrupts or callbacks. Idempotent.
 */
HAL_StatusTypeDef
accl_init(SPI_HandleTypeDef *hspi, const struct accl_config *conf)
{
  HAL_StatusTypeDef st;
  uint8_t id = 0x00;

  accl_cs_low();
  HAL_Delay(1);
  accl_cs_high();
  HAL_Delay(ACCL_WAIT_MS);

  /* Soft reset (datasheet 4.8) */
  st = accl_write_reg(hspi, ACCL_RESET, ACCL_RESET_VAL);
  if (st != HAL_OK) {
    return st;
  }

  HAL_Delay(ACCL_WAIT_MS);

  /* Dummy read (result ignored) */ 
  st = accl_read_reg(hspi, ACCL_CHIP_ADDR, &id);

  /* Match chip ID */ 
  st = accl_read_reg(hspi, ACCL_CHIP_ADDR, &id);
  if (st != HAL_OK) {
    return st;
  }
  if (id != ACCL_CHIP_ID) {
    log_err("ACCL: CHIP_ID=0x%02X (expected 0x1E)", id);
    return HAL_ERROR;
  }

  /* Configuration setup with default fallbacks */
  struct accl_config valid = {
    .mode = Normal_1600Hz,
    .rng  = Accl_Range_24g,
  };

  if (conf) {
    if (conf->mode >= OSR4_100Hz && conf->mode <= Normal_1600Hz) {
      valid.mode = conf->mode;
    }
    if (conf->rng >= Accl_Range_3g && conf->rng <= Accl_Range_24g) {
      valid.rng = conf->rng;
    }
  }

  /* Bandwith of low pass filter (datasheet 5.3.10) */
  st = accl_write_reg(hspi, ACCL_CONF, valid.mode);
  if (st != HAL_OK) {
    return st;
  }

  /* Set range to ±24g (datasheet 5.3.11) */
  st = accl_write_reg(hspi, ACCL_RANGE, valid.rng);
  if (st != HAL_OK) {
    return st;
  }

  /* Enable active mode (datasheet 5.3.20) */
  st = accl_write_reg(hspi, ACCL_PWR_CONF, Active_Mode);
  if (st != HAL_OK) {
    return st;
  }

  HAL_Delay(ACCL_WAIT_MS);

  /* Power on (enter normal mode) (datasheet 3 & 4.1.1) */
  st = accl_write_reg(hspi, ACCL_PWR_CTRL, Accl_On);
  if (st != HAL_OK) {
    return st;
  }

  HAL_Delay(1);

  return HAL_OK;
}

/*
 * Perform self-test, write raw data to the argument struct,
 * and reinitialize accelerometer (datasheet ch. 4.6.1).
 * Do not call from interrupts or callbacks. Idempotent.
 */
HAL_StatusTypeDef
accl_test(SPI_HandleTypeDef *hspi, struct coords *buf,
          const struct accl_config *conf)
{
  HAL_StatusTypeDef st;
  struct coords data_p;
  struct coords data_n;

  st = accl_write_reg(hspi, ACCL_CONF, ACCL_TEST_CONF);
  if (st != HAL_OK) {
    return st;
  }

  HAL_Delay(5);

  st = accl_write_reg(hspi, ACCL_SELF_TEST, ACCL_POS_POL);
  if (st != HAL_OK) {
    return st;
  }

  HAL_Delay(ACCL_WAIT_MS);

  st = accl_read(hspi, &data_p);
  if (st != HAL_OK) {
    return st;
  }

  st = accl_write_reg(hspi, ACCL_SELF_TEST, ACCL_NEG_POL);
  if (st != HAL_OK) {
    return st;
  }

  HAL_Delay(ACCL_WAIT_MS);

  st = accl_read(hspi, &data_n);
  if (st != HAL_OK) {
    return st;
  }

  st = accl_write_reg(hspi, ACCL_SELF_TEST, ACCL_TEST_OFF);
  if (st != HAL_OK) {
    return st;
  }

  buf->x = (float)(data_p.x - data_n.x);
  buf->y = (float)(data_p.y - data_n.y);
  buf->z = (float)(data_p.z - data_n.z);
  
  return accl_init(hspi, conf);
}