#include "barometer.h"
#include "stm32h5xx_hal_def.h"
#include "stm32h5xx_hal_spi.h"
#include "telemetry.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include <stdatomic.h>

// ---- Globals ----
BMP390_calib_data_t calib_data = {0};
float ground_level_pressure = 0.0f;

// ---- Private helpers ----
static inline uint32_t u24(uint8_t b0, uint8_t b1, uint8_t b2) {
  return ((uint32_t)b2 << 16) | ((uint32_t)b1 << 8) | (uint32_t)b0;
}

// ---- Basic SPI helpers ----
HAL_StatusTypeDef baro_read_reg(SPI_HandleTypeDef *hspi, uint8_t reg,
                                uint8_t *out, uint16_t len) {
  if (!out || len == 0 || len > 255)
    return HAL_ERROR;

  uint8_t head[2] = {(uint8_t)(reg | BMP390_SPI_READ_BIT), 0x00};

  BARO_CS_LOW();
  HAL_StatusTypeDef st =
      HAL_SPI_Transmit(hspi, head, sizeof(head), BMP390_SPI_TIMEOUT_MS);
  if (st != HAL_OK) {
    BARO_CS_HIGH();
    return st;
  }

  st = HAL_SPI_Receive(hspi, out, len, BMP390_SPI_TIMEOUT_MS);
  BARO_CS_HIGH();
  return st;
}

HAL_StatusTypeDef baro_write_reg(SPI_HandleTypeDef *hspi, uint8_t reg,
                                 const uint8_t *data, uint16_t len) {
  if (len > 255)
    return HAL_ERROR;

  uint8_t tx[1 + 255];
  tx[0] = (uint8_t)(reg & BMP390_SPI_WRITE_MASK);
  if (len && data)
    memcpy(&tx[1], data, len);

  BARO_CS_LOW();
  HAL_StatusTypeDef st =
      HAL_SPI_Transmit(hspi, tx, (uint16_t)(len + 1), BMP390_SPI_TIMEOUT_MS);
  BARO_CS_HIGH();
  return st;
}

HAL_StatusTypeDef baro_read_u8(SPI_HandleTypeDef *hspi, uint8_t reg,
                               uint8_t *val) {
  return baro_read_reg(hspi, reg, val, 1);
}

HAL_StatusTypeDef baro_write_u8(SPI_HandleTypeDef *hspi, uint8_t reg,
                                uint8_t val) {
  return baro_write_reg(hspi, reg, &val, 1);
}

// ---- Trim/NVM read ----
static HAL_StatusTypeDef read_trim_pars(SPI_HandleTypeDef *hspi) {
  uint8_t raw[21];
  HAL_StatusTypeDef st = baro_read_reg(hspi, BARO_NVM_PAR_T1, raw, sizeof(raw));
  if (st != HAL_OK)
    return st;

  // Integer coefficients (datasheet §8.4)
  uint16_t nvm_par_t1 = (uint16_t)((raw[1] << 8) | raw[0]); // U16
  int16_t nvm_par_t2 = (int16_t)((raw[3] << 8) | raw[2]);   // S16
  int8_t nvm_par_t3 = (int8_t)raw[4];

  int16_t nvm_par_p1 = (int16_t)((raw[6] << 8) | raw[5]); // S16
  int16_t nvm_par_p2 = (int16_t)((raw[8] << 8) | raw[7]); // S16
  int8_t nvm_par_p3 = (int8_t)raw[9];
  int8_t nvm_par_p4 = (int8_t)raw[10];
  uint16_t nvm_par_p5 = (uint16_t)((raw[12] << 8) | raw[11]); // U16
  uint16_t nvm_par_p6 = (uint16_t)((raw[14] << 8) | raw[13]); // U16
  int8_t nvm_par_p7 = (int8_t)raw[15];
  int8_t nvm_par_p8 = (int8_t)raw[16];
  int16_t nvm_par_p9 = (int16_t)((raw[18] << 8) | raw[17]); // S16
  int8_t nvm_par_p10 = (int8_t)raw[19];
  int8_t nvm_par_p11 = (int8_t)raw[20];

  // Convert to float (matches your Arduino code)
  calib_data.par_t1 = (float)nvm_par_t1 / powf(2.0f, -8.0f); // nvm * 256
  calib_data.par_t2 = (float)nvm_par_t2 / powf(2.0f, 30.0f);
  calib_data.par_t3 = (float)nvm_par_t3 / powf(2.0f, 48.0f);

  calib_data.par_p1 =
      ((float)nvm_par_p1 - powf(2.0f, 14.0f)) / powf(2.0f, 20.0f);
  calib_data.par_p2 =
      ((float)nvm_par_p2 - powf(2.0f, 14.0f)) / powf(2.0f, 29.0f);
  calib_data.par_p3 = (float)nvm_par_p3 / powf(2.0f, 32.0f);
  calib_data.par_p4 = (float)nvm_par_p4 / powf(2.0f, 37.0f);
  calib_data.par_p5 = (float)nvm_par_p5 / powf(2.0f, -3.0f); // nvm * 8
  calib_data.par_p6 = (float)nvm_par_p6 / powf(2.0f, 6.0f);
  calib_data.par_p7 = (float)nvm_par_p7 / powf(2.0f, 8.0f);
  calib_data.par_p8 = (float)nvm_par_p8 / powf(2.0f, 15.0f);
  calib_data.par_p9 = (float)nvm_par_p9 / powf(2.0f, 48.0f);
  calib_data.par_p10 = (float)nvm_par_p10 / powf(2.0f, 48.0f);
  calib_data.par_p11 = (float)nvm_par_p11 / powf(2.0f, 65.0f);

  return HAL_OK;
}

// --- helpers ---
static HAL_StatusTypeDef baro_wait_idle(SPI_HandleTypeDef *hspi) {
  for (int i = 0; i < 20; ++i) {
    uint8_t s = 0;
    if (baro_read_u8(hspi, BARO_STATUS, &s) != HAL_OK)
      return HAL_ERROR;
    HAL_Delay(2);
  }
  return HAL_OK;
}

// Enter NORMAL mode with sensors enabled
static HAL_StatusTypeDef baro_try_enter_normal(SPI_HandleTypeDef *hspi) {
  uint8_t rb;

  if (baro_write_u8(hspi, BARO_PWR_CTRL, (uint8_t)BMP390_PWR_NORMAL_WITH_SENSORS) !=
      HAL_OK)
    return HAL_ERROR;
  HAL_Delay(BMP390_MODE_SWITCH_DELAY_MS);

  if (baro_read_u8(hspi, BARO_PWR_CTRL, &rb) != HAL_OK)
    return HAL_ERROR;
  if (((rb & BMP390_PWR_MODE_MASK) == BMP390_PWR_MODE_NORMAL) &&
      ((rb & BMP390_PWR_ENABLE_SENSORS) == BMP390_PWR_ENABLE_SENSORS))
    return HAL_OK;

  // Retry from sleep
  (void)baro_wait_idle(hspi);
  (void)baro_write_u8(hspi, BARO_PWR_CTRL, (uint8_t)0x00);
  HAL_Delay(BMP390_ENABLE_DELAY_MS);
  (void)baro_write_u8(hspi, BARO_PWR_CTRL, (uint8_t)BMP390_PWR_ENABLE_SENSORS);
  HAL_Delay(BMP390_ENABLE_DELAY_MS);
  (void)baro_write_u8(hspi, BARO_PWR_CTRL, (uint8_t)BMP390_PWR_NORMAL_WITH_SENSORS);
  HAL_Delay(BMP390_MODE_SWITCH_DELAY_MS);

  if (baro_read_u8(hspi, BARO_PWR_CTRL, &rb) != HAL_OK)
    return HAL_ERROR;
  return (((rb & BMP390_PWR_MODE_MASK) == BMP390_PWR_MODE_NORMAL) &&
          ((rb & BMP390_PWR_ENABLE_SENSORS) == BMP390_PWR_ENABLE_SENSORS))
             ? HAL_OK
             : HAL_ERROR;
}

// Wait for both temp+press ready (STATUS bit6|bit5)
static HAL_StatusTypeDef baro_wait_drdy(SPI_HandleTypeDef *hspi,
                                        uint32_t extra_ms) {
  uint8_t odr_sel = 0;
  if (baro_read_u8(hspi, BARO_ODR, &odr_sel) != HAL_OK)
    return HAL_ERROR;
  odr_sel &= BMP390_ODR_SEL_MASK;

  uint32_t period_ms = BMP390_PERIOD_MS_FROM_ODRSEL(odr_sel);
  uint32_t timeout_ms = 3u * period_ms + extra_ms;
  if (timeout_ms < 50u)
    timeout_ms = 50u;

  uint32_t t0 = HAL_GetTick();
  for (;;) {
    uint8_t s = 0;
    if (baro_read_u8(hspi, BARO_STATUS, &s) != HAL_OK)
      return HAL_ERROR;
    if ((s & BMP390_STATUS_BOTH_DRDY) == BMP390_STATUS_BOTH_DRDY)
      return HAL_OK;
    if ((HAL_GetTick() - t0) >= timeout_ms)
      return HAL_TIMEOUT;
    HAL_Delay(1);
  }
}

// Non-blocking: just peek STATUS
static HAL_StatusTypeDef baro_check_drdy(SPI_HandleTypeDef *hspi) {
  uint8_t s = 0;
  if (baro_read_u8(hspi, BARO_STATUS, &s) != HAL_OK)
    return HAL_ERROR;
  return ((s & BMP390_STATUS_BOTH_DRDY) == BMP390_STATUS_BOTH_DRDY) ? HAL_OK
                                                                    : HAL_BUSY;
}

// ---- Device init ----
HAL_StatusTypeDef init_barometer(SPI_HandleTypeDef *hspi) {
  HAL_StatusTypeDef st;
  uint8_t v;

  // Reset
  {
    uint8_t cmd = BMP390_SOFTRESET_CMD;
    st = baro_write_reg(hspi, BARO_CMD, &cmd, 1);
    if (st != HAL_OK)
      die("baro: failed to issue soft reset\r\n");
    HAL_Delay(BMP390_RESET_DELAY_MS);
  }

  // Check ID
  st = baro_read_u8(hspi, BARO_CHIP_ID, &v);
  if (st != HAL_OK)
    die("baro: failed to read CHIP_ID\r\n");
  if (v != BMP390_CHIP_ID_VALUE)
    die("baro: CHIP_ID=0x%02X (expected 0x60)\r\n", v);

  // Force 4-wire
  v = 0x00;
  (void)baro_write_reg(hspi, BARO_IF_CONF, &v, 1);
  HAL_Delay(BMP390_ENABLE_DELAY_MS);

  // Enable sensors in SLEEP, then NORMAL
  (void)baro_write_u8(hspi, BARO_PWR_CTRL, 0x00); // sleep
  HAL_Delay(BMP390_ENABLE_DELAY_MS);
  if (baro_write_u8(hspi, BARO_PWR_CTRL, BMP390_PWR_ENABLE_SENSORS) != HAL_OK)
    die("baro: enable sensors failed\r\n");
  HAL_Delay(BMP390_ENABLE_DELAY_MS);

  // OSR/ODR
  if (baro_write_u8(hspi, BARO_OSR, BMP390_DEFAULT_OSR) != HAL_OK)
    die("baro: write OSR failed\r\n"); // Tx1, Px2
  if (baro_write_u8(hspi, BARO_ODR, BMP390_DEFAULT_ODR_SEL) != HAL_OK)
    die("baro: write ODR failed\r\n"); // 12.5Hz
  HAL_Delay(BMP390_ENABLE_DELAY_MS);

  // NORMAL
  if (baro_try_enter_normal(hspi) != HAL_OK) {
    uint8_t pwr = 0, status = 0, err = 0;
    (void)baro_read_u8(hspi, BARO_PWR_CTRL, &pwr);
    (void)baro_read_u8(hspi, BARO_STATUS, &status);
    (void)baro_read_u8(hspi, BARO_ERR_REG, &err);
    die("baro: NORMAL failed, PWR_CTRL=0x%02X STATUS=0x%02X ERR=0x%02X\r\n",
        pwr, status, err);
  }

  // Read calibration
  if (read_trim_pars(hspi) != HAL_OK)
    die("baro: read calibration failed\r\n");

  // Let measurements settle (~2 frames)
  uint8_t odr_sel = 0;
  (void)baro_read_u8(hspi, BARO_ODR, &odr_sel);
  uint32_t period_ms = BMP390_PERIOD_MS_FROM_ODRSEL(odr_sel);
  HAL_Delay(2u * period_ms);

  // Prime baseline
  float p = 0.0f, t = 0.0f;
  for (int i = 0; i < 5; ++i) {
    if (baro_wait_drdy(hspi, 5) == HAL_OK &&
        get_temperature_pressure(hspi, &t, &p) == HAL_OK) {
      if (p >= BMP390_BARO_VALID_MIN_PA && p <= BMP390_BARO_VALID_MAX_PA)
        break;
    }
    HAL_Delay(10);
  }
  if (p < BMP390_BARO_VALID_MIN_PA || p > BMP390_BARO_VALID_MAX_PA)
    die("baro: baseline pressure invalid: %.1f Pa\r\n", p);

  ground_level_pressure = p;
  return HAL_OK;
}

void baro_rezero(SPI_HandleTypeDef *hspi) {
  float p, t;
  if (get_temperature_pressure(hspi, &t, &p) == HAL_OK &&
      p >= BMP390_BARO_VALID_MIN_PA && p <= BMP390_BARO_VALID_MAX_PA) {
    ground_level_pressure = p;
  }
}

// ---- Compensation math (datasheet §8.5/§8.6) ----
float compensate_temperature(uint32_t uncomp_temp) {
  float partial_data1;
  float partial_data2;
  partial_data1 = (float)(uncomp_temp - calib_data.par_t1);
  partial_data2 = (float)(partial_data1 * calib_data.par_t2);
  /* Update the compensated temperature in calib structure since this is
   * needed for pressure calculation */
  calib_data.t_lin =
      partial_data2 + (partial_data1 * partial_data1) * calib_data.par_t3;
  /* Returns compensated temperature */
  return calib_data.t_lin;
}

float compensate_pressure(uint32_t uncomp_press) {
  /* Variable to store the compensated pressure */
  float comp_press;
  /* Temporary variables used for compensation */
  float partial_data1;
  float partial_data2;
  float partial_data3;
  float partial_data4;
  float partial_out1;
  float partial_out2;
  /* Calibration data */
  partial_data1 = calib_data.par_p6 * calib_data.t_lin;
  partial_data2 = calib_data.par_p7 * (calib_data.t_lin * calib_data.t_lin);
  partial_data3 = calib_data.par_p8 *
                  (calib_data.t_lin * calib_data.t_lin * calib_data.t_lin);
  partial_out1 =
      calib_data.par_p5 + partial_data1 + partial_data2 + partial_data3;
  partial_data1 = calib_data.par_p2 * calib_data.t_lin;
  partial_data2 = calib_data.par_p3 * (calib_data.t_lin * calib_data.t_lin);
  partial_data3 = calib_data.par_p4 *
                  (calib_data.t_lin * calib_data.t_lin * calib_data.t_lin);
  partial_out2 = (float)uncomp_press * (calib_data.par_p1 + partial_data1 +
                                        partial_data2 + partial_data3);
  partial_data1 = (float)uncomp_press * (float)uncomp_press;
  partial_data2 = calib_data.par_p9 + calib_data.par_p10 * calib_data.t_lin;
  partial_data3 = partial_data1 * partial_data2;
  partial_data4 = partial_data3 + ((float)uncomp_press * (float)uncomp_press *
                                   (float)uncomp_press) *
                                      calib_data.par_p11;
  comp_press = partial_out1 + partial_out2 + partial_data4;
  return comp_press;
}

// ---- Public getters ----
HAL_StatusTypeDef get_temperature(SPI_HandleTypeDef *hspi,
                                  float *temperature_c) {
  uint8_t tbuf[3];
  if (baro_wait_drdy(hspi, 5) != HAL_OK)
    return HAL_TIMEOUT;
  HAL_StatusTypeDef st = baro_read_reg(hspi, BARO_DATA_3, tbuf, sizeof(tbuf));
  if (st != HAL_OK)
    return st;

  uint32_t adc_t = u24(tbuf[0], tbuf[1], tbuf[2]);
  float t_c = compensate_temperature(adc_t);
  if (temperature_c)
    *temperature_c = t_c;
  return HAL_OK;
}

HAL_StatusTypeDef get_temperature_pressure(SPI_HandleTypeDef *hspi,
                                           float *temperature_c,
                                           float *pressure_pa) {
  uint8_t buf[6];
  if (baro_wait_drdy(hspi, 5) != HAL_OK)
    return HAL_TIMEOUT;

  HAL_StatusTypeDef st = baro_read_reg(hspi, BARO_DATA_0, buf, sizeof(buf));
  if (st != HAL_OK)
    return st;

  uint32_t adc_p = u24(buf[0], buf[1], buf[2]);
  uint32_t adc_t = u24(buf[3], buf[4], buf[5]);

  float t_c = compensate_temperature(adc_t);
  float p_pa = compensate_pressure(adc_p);

  if (temperature_c)
    *temperature_c = t_c;
  if (pressure_pa)
    *pressure_pa = p_pa;
  return HAL_OK;
}

HAL_StatusTypeDef get_temperature_pressure_altitude(SPI_HandleTypeDef *hspi,
                                                    float *temperature_c,
                                                    float *pressure_pa,
                                                    float *altitude_m) {
  float t, p;
  HAL_StatusTypeDef st = get_temperature_pressure(hspi, &t, &p);
  if (st != HAL_OK)
    return st;
  if (temperature_c)
    *temperature_c = t;
  if (pressure_pa)
    *pressure_pa = p;
  if (altitude_m)
    *altitude_m = compute_relative_altitude(p);
  return HAL_OK;
}

HAL_StatusTypeDef get_pressure(SPI_HandleTypeDef *hspi, float *pressure_pa) {
  float temp;
  return get_temperature_pressure(hspi, &temp, pressure_pa);
}

// ---- Altitude helpers ----
static inline float altitude_from_pressures(float p, float p0) {
  return 44330.0f * (1.0f - powf(p / p0, BMP390_HYPSOMETRIC_EXPONENT));
}

static inline float pressure_for_altitude(float p, float alt) {
  // compute baseline p0 that makes this pressure correspond to given alt
  const float k = BMP390_HYPSOMETRIC_EXPONENT;
  float denom = 1.0f - (alt / 44330.0f);
  if (denom <= 0.0f)
    denom = 1e-6f;
  return p / powf(denom, 1.0f / k);
}

float compute_relative_altitude(float pressure) {
  static float prev_rel_alt = 0.0f;
  static float last_checkpoint_alt = 0.0f;

  if (ground_level_pressure <= 0.0f || !isfinite(pressure)) {
    ground_level_pressure = pressure;
    prev_rel_alt = 0.0f;
    last_checkpoint_alt = 0.0f;
    return 0.0f;
  }

  float rel_alt = altitude_from_pressures(pressure, ground_level_pressure);

  uint8_t entered_zero_band = (fabsf(rel_alt) <= HYST_DEADBAND) &&
                              (fabsf(prev_rel_alt) > HYST_DEADBAND);
  uint8_t moved_step = (fabsf(rel_alt - last_checkpoint_alt) >= RESET_STEP_M);

  if (entered_zero_band) {
    // Reanchor to true 0
    ground_level_pressure = pressure;
    prev_rel_alt = 0.0f;
    last_checkpoint_alt = 0.0f;

    return 0.0f;
  }

  if (moved_step) {
    // Compute the baseline pressure that keeps this altitude consistent,
    ground_level_pressure = pressure_for_altitude(pressure, rel_alt);
    last_checkpoint_alt = rel_alt;

    // no change to rel_bias — continuous by construction
  }

  prev_rel_alt = rel_alt;
  return rel_alt;
}

float get_relative_altitude(SPI_HandleTypeDef *hspi) {
  float p;
  if (get_pressure(hspi, &p) != HAL_OK)
    return -1.0f;
  return compute_relative_altitude(p);
}

float get_relative_altitude_non_blocking(SPI_HandleTypeDef *hspi) {
  float p;
  if (get_pressure(hspi, &p) != HAL_OK)
    return -1.0f;
  return compute_relative_altitude(p);
}

// ---- Non-blocking convenience ----
static float last_temp = 0.0f;
static float last_press = 0.0f;

HAL_StatusTypeDef get_temperature_pressure_non_blocking(SPI_HandleTypeDef *hspi,
                                                        float *temperature_c,
                                                        float *pressure_pa) {
  if (baro_check_drdy(hspi) != HAL_OK) {
    if (temperature_c)
      *temperature_c = last_temp;
    if (pressure_pa)
      *pressure_pa = last_press;
    return HAL_OK;
  }

  uint8_t buf[6];
  HAL_StatusTypeDef st = baro_read_reg(hspi, BARO_DATA_0, buf, sizeof(buf));
  if (st != HAL_OK)
    return st;

  uint32_t adc_p = u24(buf[0], buf[1], buf[2]);
  uint32_t adc_t = u24(buf[3], buf[4], buf[5]);

  float t_c = compensate_temperature(adc_t);
  float p_pa = compensate_pressure(adc_p);
  last_temp = t_c;
  last_press = p_pa;

  if (temperature_c)
    *temperature_c = t_c;
  if (pressure_pa)
    *pressure_pa = p_pa;
  return HAL_OK;
}

HAL_StatusTypeDef get_temperature_pressure_altitude_non_blocking(
    SPI_HandleTypeDef *hspi, float *temperature_c, float *pressure_pa,
    float *altitude_m) {
  if (baro_check_drdy(hspi) != HAL_OK) {
    if (temperature_c)
      *temperature_c = last_temp;
    if (pressure_pa)
      *pressure_pa = last_press;
    if (altitude_m)
      *altitude_m = compute_relative_altitude(last_press);
    return HAL_OK;
  }

  uint8_t buf[6];
  HAL_StatusTypeDef st = baro_read_reg(hspi, BARO_DATA_0, buf, sizeof(buf));
  if (st != HAL_OK)
    return st;

  uint32_t adc_p = u24(buf[0], buf[1], buf[2]);
  uint32_t adc_t = u24(buf[3], buf[4], buf[5]);

  float t_c = compensate_temperature(adc_t);
  float p_pa = compensate_pressure(adc_p);
  last_temp = t_c;
  last_press = p_pa;

  if (temperature_c)
    *temperature_c = t_c;
  if (pressure_pa)
    *pressure_pa = p_pa;
  if (altitude_m)
    *altitude_m = compute_relative_altitude(p_pa);
  return HAL_OK;
}

HAL_StatusTypeDef get_pressure_non_blocking(SPI_HandleTypeDef *hspi,
                                            float *pressure_pa) {
  float temp;
  return get_temperature_pressure_non_blocking(hspi, &temp, pressure_pa);
}


HAL_StatusTypeDef get_temperature_non_blocking(SPI_HandleTypeDef *hspi, float *temperature_c){
  if (baro_check_drdy(hspi) != HAL_OK) {
    if (temperature_c)
      *temperature_c = last_temp;
    return HAL_OK;
  }

  uint8_t buf[3];
  HAL_StatusTypeDef st = baro_read_reg(hspi, BARO_DATA_3, buf, sizeof(buf));
  if (st != HAL_OK)
    return st;

  uint32_t adc_t = u24(buf[0], buf[1], buf[2]);

  float t_c = compensate_temperature(adc_t);
  last_temp = t_c;

  if (temperature_c)
    *temperature_c = t_c;
  return HAL_OK;
}