/*
 * BMI088 Gyroscope driver over SPI.
 */

#include "platform.h"
#include "gyroscope.h"


/* ------ Synchronous SPI helpers ------ */

/*
 * Synchronously read one value at the specified register.
 */
static inline HAL_StatusTypeDef
gyro_read_reg(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t *val)
{
  if (!val) {
    return HAL_ERROR;
  }

  HAL_StatusTypeDef st;
  uint8_t cmd = gyro_cmd_read(reg);

  gyro_cs_low();
  st = HAL_SPI_TransmitReceive(hspi, &cmd, val, 1, HAL_MAX_DELAY);
  gyro_cs_high();

  return st;
}

/*
 * Synchronously write new value to a specified writeable register. 
 */
static inline HAL_StatusTypeDef
gyro_write_reg(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t val)
{
  HAL_StatusTypeDef st;
  uint8_t tx[2] = { gyro_cmd_write(reg), val };

  gyro_cs_low();
  st = HAL_SPI_Transmit(hspi, tx, sizeof tx, HAL_MAX_DELAY);
  gyro_cs_high();

  return st;
}

/*
 * Read the gyroscope axes data.
 */
static inline HAL_StatusTypeDef
gyro_burst_read(SPI_HandleTypeDef *hspi, uint8_t start, uint8_t *buf, uint16_t len)
{
  if (!buf || !len) {
    return HAL_ERROR;
  }

  HAL_StatusTypeDef st;
  uint8_t cmd = gyro_cmd_read(start);

  gyro_cs_low();
  st = HAL_SPI_Transmit(hspi, &cmd, 1, HAL_MAX_DELAY);

  if (st == HAL_OK) {
    st = HAL_SPI_Receive(hspi, buf, len, HAL_MAX_DELAY);  
  }
  gyro_cs_high();

  return st;
}


/* ------ Utilities for raw integer and float (DPS) formats ------ */

/*
 * Read XYZ (LSB first then MSB) — datasheet §5.5.2.
 */
HAL_StatusTypeDef
gyro_read_raw(SPI_HandleTypeDef *hspi, struct gyro_raw *data)
{
    HAL_StatusTypeDef st;
    uint8_t buf[6];

    st = gyro_burst_read(hspi, GYRO_RATE_X_LSB, buf, sizeof buf);
    if (st != HAL_OK) {
      return st;
    }

    data->x = (int16_t)((uint16_t)buf[1] << 8 | buf[0]);
    data->y = (int16_t)((uint16_t)buf[3] << 8 | buf[2]);
    data->z = (int16_t)((uint16_t)buf[5] << 8 | buf[4]);

    return HAL_OK;
}

/* Set by gyro_init */
static enum gyro_range init_rng = Gyro_Range_Entries;

/* Inverted to avoid division at runtime */
static const float inv_sensivity[Gyro_Range_Entries] = {
  1.0f/16.384f, 1.0f/32.768f, 1.0f/65.536f, 1.0f/131.072f, 1.0f/262.144f
};

/*
 * Convert raw axes readings to degrees per second.
 */
void
gyro_convert(const struct gyro_raw *raw, struct coords *buf)
{
    if (!raw || !buf) {
      return;
    }

    /* Pre-initialization fallback */
    if (init_rng == Gyro_Range_Entries) {
      init_rng = Gyro_Range_2000Dps;
    }

    buf->x = (float)raw->x * inv_sensivity[init_rng];
    buf->y = (float)raw->y * inv_sensivity[init_rng];
    buf->z = (float)raw->z * inv_sensivity[init_rng];
}

/*
 * Read raw values and convert them to degrees per second.
 */
HAL_StatusTypeDef
gyro_read(SPI_HandleTypeDef *hspi, struct coords *buf)
{
  HAL_StatusTypeDef st;
  struct gyro_raw data;

  st = gyro_read_raw(hspi, &data);
  if (st != HAL_OK) {
    return st;
  }

  gyro_convert(&data, buf);
  
  return HAL_OK;
}


/* ------ Initialization ------ */

/*
 * Initializes gyroscope with specified parameters.
 * Bandwidth and range values can be found in gyroscope.h.
 * Do not call from interrupts or callbacks. Idempotent.
 */
HAL_StatusTypeDef
gyro_init(SPI_HandleTypeDef *hspi, const struct gyro_config *conf)
{
  HAL_StatusTypeDef st;

  HAL_Delay(GYRO_WAIT_MS);

  /* Match chip ID */
  uint8_t id = 0;
  st = gyro_read_reg(hspi, GYRO_CHIP_ID, &id);

  if (st != HAL_OK) {
    return st;
  }
  if (id != GYRO_CHIP_ID_VALUE) {
      log_err("ACCL: CHIP_ID=0x%02X (expected 0x0F)", id);
      return HAL_ERROR;
  }

  /* Soft reset (0xB6 → 0x14), then wait ≥30ms */
  st = gyro_write_reg(hspi, GYRO_SOFTRESET, 0xB6);
  if (st != HAL_OK) {
    return st;
  }
  
  HAL_Delay(GYRO_WAIT_MS);

  /* Normal mode (write 0x00 to GYRO_LPM1 per §4.1.2) */
  st = gyro_write_reg(hspi, GYRO_LPM1, 0x00);
  if (st != HAL_OK) {
    return st;
  }
  
  HAL_Delay(GYRO_WAIT_MS);

  /* Configuration setup with default fallbacks */
  struct gyro_config valid = {
    .rng = Gyro_Range_2000Dps,
    .bw  = Gyro_532Hz_ODR_2000Hz,
  };

  if (conf) {
    if (conf->rng >= Gyro_Range_2000Dps &&
        conf->rng <= Gyro_Range_125Dps)
    {
      valid.rng = conf->rng;
    }
    if (conf->bw >= Gyro_532Hz_ODR_2000Hz &&
        conf->bw <= Gyro_32Hz_ODR_100Hz)
    {
      valid.rng = conf->rng;
    }
  }

  /* Set range */
  st = gyro_write_reg(hspi, GYRO_RANGE, (uint8_t)valid.rng);
  if (st != HAL_OK) {
    return st;
  }

  /* Set bandwidth and ODR */
  st = gyro_write_reg(hspi, GYRO_BANDWIDTH, (uint8_t)valid.bw);
  if (st != HAL_OK) {
    return st;
  }

  /* Set interrupt pin INT3 (PB0) to raise on new data */
  st = gyro_write_reg(hspi, GYRO_INT_CONF, GYRO_INT_CONF_VAL);
  if (st != HAL_OK) {
    return st;
  }

  st = gyro_write_reg(hspi, GYRO_INT_MAP, GYRO_INT_MAP_VAL);
  if (st != HAL_OK) {
    return st;
  }

  return HAL_OK;
}