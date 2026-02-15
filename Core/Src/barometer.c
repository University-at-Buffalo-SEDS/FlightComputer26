/*
 * BMP390 Barometer driver over SPI.
 */

#include "platform.h"
#include "barometer.h"


/* ------ Static ------ */

static struct baro_calibration cd = {0};
static float gnd_lvl_pressure = 0.0f;


/* ------ Synchronous SPI helpers ------ */

/*
 * Synchronously read one value at the specified register.
 */
static HAL_StatusTypeDef
baro_read_reg(SPI_HandleTypeDef *hspi, uint8_t reg,
              uint8_t *out, uint16_t len)
{
  if (!out || len == 0 || len > 255) {
    return HAL_ERROR;
  }

  HAL_StatusTypeDef st;
  uint8_t head[2] = {(uint8_t)(reg | BARO_SPI_READ_BIT), 0x00};

  baro_cs_low();
  st = HAL_SPI_Transmit(hspi, head, sizeof head, BARO_SPI_TIMEOUT_MS);

  if (st != HAL_OK) {
    baro_cs_high();
    return st;
  }

  st = HAL_SPI_Receive(hspi, out, len, BARO_SPI_TIMEOUT_MS);
  baro_cs_high();

  return st;
}

/*
 * Synchronously write new value to a specified writeable register. 
 */
static HAL_StatusTypeDef
baro_write_reg(SPI_HandleTypeDef *hspi, uint8_t reg,
               const uint8_t *data, uint16_t len)
{
  if (len > 255) {
    return HAL_ERROR;
  }

  HAL_StatusTypeDef st;
  uint8_t tx[1 + 255];
  tx[0] = (uint8_t)(reg & BARO_SPI_WRITE_MASK);

  if (len && data) {
    memcpy(&tx[1], data, len);
  }

  baro_cs_low();
  st = HAL_SPI_Transmit(hspi, tx, (uint16_t)(len + 1), BARO_SPI_TIMEOUT_MS);
  baro_cs_high();

  return st;
}

/* ------ Helper wrappers for byte operations ------ */

static inline HAL_StatusTypeDef
baro_read_u8(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t *val)
{
  return baro_read_reg(hspi, reg, val, 1);
}

static inline HAL_StatusTypeDef
baro_write_u8(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t val)
{
  return baro_write_reg(hspi, reg, &val, 1);
}


/* ------ Trim/NVM read ------ */

/* Constant-folded when exp is literal (>= O2) */
#define invpowf2(exp) (1.0f / powf(2.0f, (exp)))

/*
 * Reads initial calibration coefficients from barometer registers.
 */
static inline HAL_StatusTypeDef
read_trim_pars(SPI_HandleTypeDef *hspi)
{
  HAL_StatusTypeDef st;
  uint8_t raw[21];

  st = baro_read_reg(hspi, BARO_NVM_PAR_T1, raw, sizeof raw);

  if (st != HAL_OK) {
    return st;
  }

  // Integer coefficients (datasheet §8.4)
  uint16_t nvm_par_t1 = (uint16_t)((raw[1] << 8) | raw[0]);
  int16_t  nvm_par_t2 = (int16_t) ((raw[3] << 8) | raw[2]);
  int8_t   nvm_par_t3 = (int8_t)    raw[4];

  int16_t  nvm_par_p1  = (int16_t) ((raw[6] << 8) | raw[5]);
  int16_t  nvm_par_p2  = (int16_t) ((raw[8] << 8) | raw[7]);
  int8_t   nvm_par_p3  = (int8_t)    raw[9];
  int8_t   nvm_par_p4  = (int8_t)    raw[10];
  uint16_t nvm_par_p5  = (uint16_t)((raw[12] << 8) | raw[11]);
  uint16_t nvm_par_p6  = (uint16_t)((raw[14] << 8) | raw[13]);
  int8_t   nvm_par_p7  = (int8_t)    raw[15];
  int8_t   nvm_par_p8  = (int8_t)    raw[16];
  int16_t  nvm_par_p9  = (int16_t) ((raw[18] << 8) | raw[17]);
  int8_t   nvm_par_p10 = (int8_t)    raw[19];
  int8_t   nvm_par_p11 = (int8_t)    raw[20];

  /* Conversion to float */
  cd.par_t1 = (float)nvm_par_t1 * invpowf2(-8.0f);
  cd.par_t2 = (float)nvm_par_t2 * invpowf2(30.0f);
  cd.par_t3 = (float)nvm_par_t3 * invpowf2(48.0f);

  const float exp_p1_2 = powf(2.0f, 14.0f);

  cd.par_p1  = ((float)nvm_par_p1 - exp_p1_2) * invpowf2(20.0f);
  cd.par_p2  = ((float)nvm_par_p2 - exp_p1_2) * invpowf2(29.0f);
  cd.par_p3  = (float)nvm_par_p3  * invpowf2(32.0f);
  cd.par_p4  = (float)nvm_par_p4  * invpowf2(37.0f);
  cd.par_p5  = (float)nvm_par_p5  * invpowf2(-3.0f);
  cd.par_p6  = (float)nvm_par_p6  * invpowf2(6.0f);
  cd.par_p7  = (float)nvm_par_p7  * invpowf2(8.0f);
  cd.par_p8  = (float)nvm_par_p8  * invpowf2(15.0f);
  cd.par_p9  = (float)nvm_par_p9  * invpowf2(48.0f);
  cd.par_p10 = (float)nvm_par_p10 * invpowf2(48.0f);
  cd.par_p11 = (float)nvm_par_p11 * invpowf2(65.0f);

  return HAL_OK;
}


/* ------ Wait and mode helpers ------ */

/*
 * Waiting function that repeatedly polls barometer status for 40 ms.
 */
static inline HAL_StatusTypeDef
baro_wait_idle(SPI_HandleTypeDef *hspi)
{
  uint8_t st = 0;

  for (int i = 0; i < 20; ++i)
  {
    if (baro_read_u8(hspi, BARO_STATUS, &st) != HAL_OK) {
      return HAL_ERROR;
    }
    HAL_Delay(2);
  }

  return HAL_OK;
}

/*
 * Enter NORMAL mode with sensors enabled.
 */
static inline HAL_StatusTypeDef
baro_try_enter_normal(SPI_HandleTypeDef *hspi)
{
  HAL_StatusTypeDef st;
  uint8_t rb;

  st = baro_write_u8(hspi, BARO_PWR_CTRL, BARO_PWR_NORMAL_WITH_SENSORS);
  if (st != HAL_OK) {
    return HAL_ERROR;
  }

  HAL_Delay(BARO_MODE_SWITCH_DELAY_MS);

  st = baro_read_u8(hspi, BARO_PWR_CTRL, &rb);

  if (st != HAL_OK) {
    return HAL_ERROR;
  }

  if (is_in_normal_mode(rb)) {
    return HAL_OK;
  }

  /* Retry from sleep */
  baro_wait_idle(hspi);

  baro_write_u8(hspi, BARO_PWR_CTRL, (uint8_t)0x00);
  HAL_Delay(BARO_ENABLE_DELAY_MS);

  baro_write_u8(hspi, BARO_PWR_CTRL, BARO_PWR_ENABLE_SENSORS);
  HAL_Delay(BARO_ENABLE_DELAY_MS);

  baro_write_u8(hspi, BARO_PWR_CTRL, BARO_PWR_NORMAL_WITH_SENSORS);
  HAL_Delay(BARO_MODE_SWITCH_DELAY_MS);

  st = baro_read_u8(hspi, BARO_PWR_CTRL, &rb);
  if (st != HAL_OK) {
    return HAL_ERROR;
  }

  return is_in_normal_mode(rb) ? HAL_OK : HAL_ERROR;
}

/*
 * Wait for both temp+press ready (STATUS bit6|bit5)
 */
static HAL_StatusTypeDef
baro_wait_drdy(SPI_HandleTypeDef *hspi, uint32_t extra_ms)
{
  HAL_StatusTypeDef st;
  uint8_t odr_sel = 0;

  st = baro_read_u8(hspi, BARO_ODR, &odr_sel);
  if (st != HAL_OK) {
    return HAL_ERROR;
  }

  odr_sel &= BARO_ODR_SEL_MASK;

  uint32_t period_ms = BARO_PERIOD_MS_FROM_ODRSEL(odr_sel);
  uint32_t timeout_ms = 3u * period_ms + extra_ms;

  if (timeout_ms < 50u) {
    timeout_ms = 50u;
  }

  uint32_t t0 = HAL_GetTick();
  uint8_t s = 0;

  for (;;)
  {
    st = baro_read_u8(hspi, BARO_STATUS, &s);
    if (st != HAL_OK) {
      return HAL_ERROR;
    }

    if ((s & BARO_STATUS_BOTH_DRDY) == BARO_STATUS_BOTH_DRDY) {
      return HAL_OK;
    }

    if ((HAL_GetTick() - t0) >= timeout_ms) {
      return HAL_TIMEOUT;
    }

    HAL_Delay(1);
  }
}

/*
 * Non-spinning call that performs a one-time status check.
 */
static HAL_StatusTypeDef
baro_check_drdy(SPI_HandleTypeDef *hspi)
{
  HAL_StatusTypeDef st;
  uint8_t s = 0;

  st = baro_read_u8(hspi, BARO_STATUS, &s);
  if (st != HAL_OK) {
    return HAL_ERROR;
  }
  
  return ((s & BARO_STATUS_BOTH_DRDY) == BARO_STATUS_BOTH_DRDY) ? HAL_OK
                                                                : HAL_BUSY;
}


/* ------ Compensation helpers ------ */

// FIXME this compiles to powf calls that end up on hot paths
// (distribution task). O3

static inline float
altitude_from_pressures(float p, float p0)
{
  return 44330.0f * (1.0f - powf(p / p0, BARO_HYPSOMETRIC_EXPONENT));
}

/*
 * Compute baseline p0 that makes it correspond to given altitude
 */
static inline float
pressure_for_altitude(float p, float alt)
{
  const float k = BARO_HYPSOMETRIC_EXPONENT;
  float denom = 1.0f - (alt / 44330.0f);

  if (denom <= 0.0f) {
    denom = 1e-6f;
  }

  return p / powf(denom, 1.0f / k);
}


/* ------ Public API ------ */

/*
 * Pressure -> altitude map provided by BMP390 datasheet.
 */
float baro_relative_alt(float pressure)
{
  static float prev_rel_alt = 0.0f;
  static float checkpoint_alt = 0.0f;

  if (gnd_lvl_pressure <= 0.0f || !isfinite(pressure))
  {
    gnd_lvl_pressure = pressure;
    prev_rel_alt = 0.0f;
    checkpoint_alt = 0.0f;
    return 0.0f;
  }

  float rel_alt = altitude_from_pressures(pressure, gnd_lvl_pressure);

  uint8_t entered_zero_band = (fabsf(rel_alt) <= BARO_HYST_DEADBAND) &&
                              (fabsf(prev_rel_alt) > BARO_HYST_DEADBAND);
                              
  uint8_t moved_step = fabsf(rel_alt - checkpoint_alt) >= BARO_RESET_STEP_M;

  if (entered_zero_band) {
    // Reanchor to true 0
    gnd_lvl_pressure = pressure;
    prev_rel_alt = 0.0f;
    checkpoint_alt = 0.0f;

    return 0.0f;
  }

  if (moved_step) {
    // Compute baseline pressure that keeps this altitude consistent.
    // No change to rel_bias — continuous by construction.
    gnd_lvl_pressure = pressure_for_altitude(pressure, rel_alt);
    checkpoint_alt = rel_alt;
  }

  prev_rel_alt = rel_alt;
  return rel_alt;
}

/* ------ Compensation math (datasheet §8.5/§8.6) ------ */

float baro_compensate_temp(uint32_t uncomp_temp)
{
  float part_dat_1 = (float)uncomp_temp - cd.par_t1;
  float part_dat_2 = part_dat_1 * cd.par_t2;

  /* Update the compensated temperature in calib structure since
   * this is needed for pressure calculation */
  cd.t_lin = part_dat_2 + (part_dat_1*part_dat_1) * cd.par_t3;

  return cd.t_lin;
}

float baro_compensate_pres(uint32_t uncomp_press)
{
  float pres = (float)uncomp_press;

  /* Temporary variables used for compensation */
  float part_dat_1, part_dat_2;
  float part_dat_3, part_dat_4;

  float part_out_1, part_out_2;

  /* Precomputed powers of t_lin */
  float t_lin_sq = cd.t_lin * cd.t_lin;
  float t_lin_cb = t_lin_sq * cd.t_lin;

  /* Calibration data */
  part_dat_1 = cd.par_p6 * cd.t_lin;
  part_dat_2 = cd.par_p7 * t_lin_sq;
  part_dat_3 = cd.par_p8 * t_lin_cb;
  part_out_1 = cd.par_p5 + part_dat_1 + part_dat_2 + part_dat_3;

  part_dat_1 = cd.par_p2 * cd.t_lin;
  part_dat_2 = cd.par_p3 * t_lin_sq;
  part_dat_3 = cd.par_p4 * t_lin_cb;
  part_out_2 = pres * (cd.par_p1 + part_dat_1 + part_dat_2 + part_dat_3);

  part_dat_1 = pres * pres;
  part_dat_2 = cd.par_p9 + cd.par_p10 * cd.t_lin;
  part_dat_3 = part_dat_1 * part_dat_2;
  part_dat_4 = part_dat_3 + (pres * part_dat_1) * cd.par_p11;

  return part_out_1 + part_out_2 + part_dat_4;
}


/* ------ Public synchronous getters ------ */

HAL_StatusTypeDef
baro_fetch_temp(SPI_HandleTypeDef *hspi, float *temperature_c)
{
  HAL_StatusTypeDef st;
  uint8_t tbuf[3];

  st = baro_wait_drdy(hspi, 5);
  if (st != HAL_OK) {
    return HAL_TIMEOUT;
  }

  st = baro_read_reg(hspi, BARO_DATA_3, tbuf, sizeof tbuf);
  if (st != HAL_OK) {
    return st;
  }

  uint32_t adc_t = U24(tbuf[0], tbuf[1], tbuf[2]);
  float t_c = baro_compensate_temp(adc_t);
  if (temperature_c) {
    *temperature_c = t_c;
  }

  return HAL_OK;
}

HAL_StatusTypeDef
baro_fetch_temp_pres(SPI_HandleTypeDef *hspi, float *temp_c, float *pres_pa)
{
  HAL_StatusTypeDef st;
  uint8_t buf[6];

  if (baro_wait_drdy(hspi, 5) != HAL_OK) {
    return HAL_TIMEOUT;
  }

  st = baro_read_reg(hspi, BARO_DATA_0, buf, sizeof(buf));
  if (st != HAL_OK) {
    return st;
  }

  uint32_t adc_p = U24(buf[0], buf[1], buf[2]);
  uint32_t adc_t = U24(buf[3], buf[4], buf[5]);

  float t_c = baro_compensate_temp(adc_t);
  float p_pa = baro_compensate_pres(adc_p);

  if (temp_c) {
    *temp_c = t_c;
  }
  if (pres_pa) {
    *pres_pa = p_pa;
  }

  return HAL_OK;
}

HAL_StatusTypeDef
baro_fetch_all(SPI_HandleTypeDef *hspi, float *temp_c,
               float *pres_pa, float *alt_m)
{
  HAL_StatusTypeDef st;
  float t, p;
  
  st = baro_fetch_temp_pres(hspi, &t, &p);
  if (st != HAL_OK) {
    return st;
  }

  if (temp_c) {
    *temp_c = t;
  }
  if (pres_pa) {
    *pres_pa = p;
  }
  if (alt_m) {
    *alt_m = baro_relative_alt(p);
  }

  return HAL_OK;
}

HAL_StatusTypeDef
baro_fetch_pres(SPI_HandleTypeDef *hspi, float *pres_pa)
{
  float temp;
  return baro_fetch_temp_pres(hspi, &temp, pres_pa);
}

float baro_get_alt(SPI_HandleTypeDef *hspi)
{
  float p;

  if (baro_fetch_pres(hspi, &p) != HAL_OK) {
    return -1.0f;
  }

  return baro_relative_alt(p);
}


/* ------ Try-get (non-spinning) getter variants ------ */

static float last_temp = 0.0f;
static float last_pres = 0.0f;

HAL_StatusTypeDef
baro_try_fetch_temp_pres(SPI_HandleTypeDef *hspi, float *temp_c, float *pres_pa)
{
  HAL_StatusTypeDef st;

  st = baro_check_drdy(hspi);
  if (st != HAL_OK) {
    if (temp_c) {
      *temp_c = last_temp;
    }
    if (pres_pa) {
      *pres_pa = last_pres;
    }
    return HAL_OK;
  }

  uint8_t buf[6];

  st = baro_read_reg(hspi, BARO_DATA_0, buf, sizeof buf);
  if (st != HAL_OK) {
    return st;
  }

  uint32_t adc_p = U24(buf[0], buf[1], buf[2]);
  uint32_t adc_t = U24(buf[3], buf[4], buf[5]);

  float t_c = baro_compensate_temp(adc_t);
  float p_pa = baro_compensate_pres(adc_p);
  last_temp = t_c;
  last_pres = p_pa;

  if (temp_c) {
    *temp_c = t_c;
  }
  if (pres_pa) {
    *pres_pa = p_pa;
  }

  return HAL_OK;
}

HAL_StatusTypeDef
baro_try_fetch_all(SPI_HandleTypeDef *hspi, float *temp_c,
                   float *press_pa, float *alt_m)
{
  HAL_StatusTypeDef st;

  st = baro_check_drdy(hspi);
  if (st != HAL_OK) {
    if (temp_c) {
      *temp_c = last_temp;
    }
    if (press_pa) {
      *press_pa = last_pres;
    }
    if (alt_m) {
      *alt_m = baro_relative_alt(last_pres);
    }
    return HAL_OK;
  }

  uint8_t buf[6];

  st = baro_read_reg(hspi, BARO_DATA_0, buf, sizeof buf);
  if (st != HAL_OK) {
    return st;
  }

  uint32_t adc_p = U24(buf[0], buf[1], buf[2]);
  uint32_t adc_t = U24(buf[3], buf[4], buf[5]);

  float t_c = baro_compensate_temp(adc_t);
  float p_pa = baro_compensate_pres(adc_p);
  last_temp = t_c;
  last_pres = p_pa;

  if (temp_c) {
    *temp_c = t_c;
  }
  if (press_pa) {
    *press_pa = p_pa;
  }
  if (alt_m) {
    *alt_m = baro_relative_alt(p_pa);
  }

  return HAL_OK;
}

HAL_StatusTypeDef
baro_try_fetch_pres(SPI_HandleTypeDef *hspi, float *pres_pa)
{
  float temp;
  return baro_try_fetch_temp_pres(hspi, &temp, pres_pa);
}

HAL_StatusTypeDef
baro_try_fetch_temp(SPI_HandleTypeDef *hspi, float *temp_c)
{
  HAL_StatusTypeDef st;

  st = baro_check_drdy(hspi);
  if (st != HAL_OK) {
    if (temp_c) {
      *temp_c = last_temp;
    }
    return HAL_OK;
  }

  uint8_t buf[3];

  st = baro_read_reg(hspi, BARO_DATA_3, buf, sizeof buf);
  if (st != HAL_OK) {
    return st;
  }

  uint32_t adc_t = U24(buf[0], buf[1], buf[2]);

  float t_c = baro_compensate_temp(adc_t);
  last_temp = t_c;

  if (temp_c) {
    *temp_c = t_c;
  }

  return HAL_OK;
}


/* ------ Initialization ------ */

/*
 * Initializes barometer with specified parameters.
 * OSR, ODR, and IIR values can be found in barometer.h.
 * Do not call from interrupts or callbacks. Idempotent.
 */
HAL_StatusTypeDef
baro_init(SPI_HandleTypeDef *hspi, const struct baro_config *conf)
{
  HAL_StatusTypeDef st;

  /* Soft reset */
  uint8_t cmd = BARO_SOFTRESET_CMD;

  st = baro_write_reg(hspi, BARO_CMD, &cmd, 1);
  if (st != HAL_OK) {
    log_err("BARO: failed to issue soft reset");
    return HAL_ERROR;
  }

  HAL_Delay(BARO_RESET_DELAY_MS);

  /* Match chip ID */
  uint8_t val;

  st = baro_read_u8(hspi, BARO_CHIP_ID, &val);
  if (st != HAL_OK) {
    log_err("BARO: failed to read CHIP_ID");
    return HAL_ERROR;
  }

  if (val != BARO_CHIP_ID_VALUE) {
    log_err("BARO: CHIP_ID=0x%02X (expected 0x60)", val);
    return HAL_ERROR;
  }

  /* Force 4-wire */
  val = 0x00;

  baro_write_reg(hspi, BARO_IF_CONF, &val, 1);
  HAL_Delay(BARO_ENABLE_DELAY_MS);

  /* Enable sensors in SLEEP, then NORMAL */
  baro_write_u8(hspi, BARO_PWR_CTRL, 0x00);
  HAL_Delay(BARO_ENABLE_DELAY_MS);

  st = baro_write_u8(hspi, BARO_PWR_CTRL, BARO_PWR_ENABLE_SENSORS); 
  if (st != HAL_OK) {
    log_err("BARO: failed to enable sensors");
    return HAL_ERROR;
  }

  HAL_Delay(BARO_ENABLE_DELAY_MS);

  /* Configuration setup with default fallbacks */
  struct baro_config valid = {
    .osr_t = BARO_OSR_X1,
    .osr_p = BARO_OSR_X2,
    .odr = BARO_DEFAULT_ODR_SEL,
    .iir_coef = BARO_IIR_COEF_0,
  };

  if (conf) {
    if (conf->osr_t <= BARO_OSR_X32) {
      valid.osr_t = conf->osr_t;
    }
    if (conf->osr_p <= BARO_OSR_X32) {
      valid.osr_p = conf->osr_p;
    }
    if (conf->odr <= BARO_ODR_0P0015) {
      valid.odr = conf->odr;
    }
    if (conf->iir_coef <= BARO_IIR_COEF_127) {
      valid.iir_coef = conf->iir_coef;
    }
  }

  /* Oversampling and data rates (C-f OSR / ODR) */
  st = baro_write_u8(hspi, BARO_OSR,
                     baro_osr_make(valid.osr_t, valid.osr_p));
  if (st != HAL_OK) {
    log_err("BARO: failed to write OSR");
    return HAL_ERROR;
  }

  st = baro_write_u8(hspi, BARO_ODR, valid.odr);
  if (st != HAL_OK) {
    log_err("BARO: failed to write ODR");
    return HAL_ERROR;
  }

  HAL_Delay(BARO_ENABLE_DELAY_MS);

  /* IIR filter coefficient */
  st = baro_write_u8(hspi, BARO_CONFIG, valid.iir_coef);
  if (st != HAL_OK) {
    log_err("BARO: failed to write IIR coef");
    return HAL_ERROR;
  }

  HAL_Delay(BARO_ENABLE_DELAY_MS);

  /* Try to enter normal mode */
  st = baro_try_enter_normal(hspi);
  if (st != HAL_OK)
  {
    uint8_t pwr = 0, status = 0, err = 0;
    baro_read_u8(hspi, BARO_PWR_CTRL, &pwr);
    baro_read_u8(hspi, BARO_STATUS, &status);
    baro_read_u8(hspi, BARO_ERR_REG, &err);

    log_err("BARO: Could not enter normal mode.\n"
            "PWR_CTRL=0x%02X STATUS=0x%02X ERR=0x%02X",
            pwr, status, err);
    return HAL_ERROR;
  }

  /* Read calibration parameters from barometer registers */
  st = read_trim_pars(hspi);
  if (st != HAL_OK) {
    log_err("BARO: failed to read calibration params");
    return HAL_ERROR;
  }

  /* Let measurements settle (~2 frames) */
  uint8_t odr_sel = 0;
  baro_read_u8(hspi, BARO_ODR, &odr_sel);
  uint32_t period_ms = BARO_PERIOD_MS_FROM_ODRSEL(odr_sel);
  HAL_Delay(2u * period_ms);

  /* Try to validate first mesurements */
  float p = 0.0f, t = 0.0f;

  for (uint8_t i = 0; i < 5; ++i)
  {
    st = baro_wait_drdy(hspi, 5);
    if (st != HAL_OK) {
      HAL_Delay(5);
      continue;
    }

    st = baro_fetch_temp_pres(hspi, &t, &p);
    if (st == HAL_OK && p >= BARO_MIN_PA && p <= BARO_MAX_PA)
    {
      break; /* Validation successful */
    }

    HAL_Delay(10);
  }

  if (p < BARO_MIN_PA || p > BARO_MAX_PA) {
    log_err("BARO: baseline pressure invalid: %.1f Pa", p);
    return HAL_ERROR;
  }

  gnd_lvl_pressure = p;

  return HAL_OK;
}

/*
 * Obtain and set new ground level pressure.
 */
void baro_rezero(SPI_HandleTypeDef *hspi)
{
  HAL_StatusTypeDef st;
  float p = 0.0f, t = 0.0f;

  st = baro_fetch_temp_pres(hspi, &t, &p);

  if (st == HAL_OK && p >= BARO_MIN_PA && p <= BARO_MAX_PA)
  {
    gnd_lvl_pressure = p;
  }
}