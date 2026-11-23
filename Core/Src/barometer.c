/*
 * BMP390 Barometer driver over SPI.
 */

#include "platform.h"
#include "fc_watchdog_recovery.h"
#include "barometer.h"


/* ------ Static ------ */

static struct baro_calibration cd = {0};

static float   prev_rel_alt     = 0.0f;  /* last returned altitude (m)               */
static uint32_t last_tick       = 0;     /* tick of the last live sample (ST-Link viewer) */

/* Last compensated sample from this file's read paths */
static float   last_temp  = 0.0f;
static float   last_pres  = 0.0f;
static uint8_t last_fresh = 0;

#if BOARD_WATCHDOG_ENABLE
/* The ground reference first: the resume check requires a positive pressure. */
const fc_resume_region fc_resume_barometer[] = {
  FC_RESUME_REGION(baro_ref),
  FC_RESUME_REGION(prev_rel_alt),
  { NULL, 0U }
};
#endif


/* ------ Synchronous SPI helpers ------ */

/*
 * Synchronously read len bytes starting at the specified register.
 */
static inline HAL_StatusTypeDef baro_read_reg(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t *out, uint16_t len)
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
 * Synchronously write new value(s) to a specified writeable register.
 */
static inline HAL_StatusTypeDef baro_write_reg(SPI_HandleTypeDef *hspi, uint8_t reg, const uint8_t *data, uint16_t len) {
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

static inline HAL_StatusTypeDef baro_read_u8(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t *val) {
  return baro_read_reg(hspi, reg, val, 1);
}

static inline HAL_StatusTypeDef baro_write_u8(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t val)  {
  return baro_write_reg(hspi, reg, &val, 1);
}


/* ------ Trim/NVM read ------ */

/* Constant-folded when exp is literal (>= O2) */
#define invpowf2(exp) (1.0f / powf(2.0f, (exp)))

/*
 * Reads initial calibration coefficients from barometer registers.
 */
static inline HAL_StatusTypeDef read_trim_pars(SPI_HandleTypeDef *hspi) {
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
 * Wait (up to 40 ms) for the command interface to be ready (STATUS.cmd_rdy).
 */
static inline HAL_StatusTypeDef baro_wait_idle(SPI_HandleTypeDef *hspi) {
  uint8_t s = 0;
  uint32_t t0 = HAL_GetTick();

  do {
    if (baro_read_u8(hspi, BARO_STATUS, &s) != HAL_OK) {
      return HAL_ERROR;
    }

    if (s & BARO_STATUS_CMD_RDY) {
      return HAL_OK;
    }

    HAL_Delay(1);
  } while ((HAL_GetTick() - t0) < 40u);

  return HAL_TIMEOUT;
}

/*
 * Enter NORMAL mode with sensors enabled.
 */
static inline HAL_StatusTypeDef baro_try_enter_normal(SPI_HandleTypeDef *hspi) {
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
static inline HAL_StatusTypeDef baro_wait_drdy(SPI_HandleTypeDef *hspi, uint32_t extra_ms) {
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
static inline HAL_StatusTypeDef baro_check_drdy(SPI_HandleTypeDef *hspi) {
  HAL_StatusTypeDef st;
  uint8_t s = 0;
 
  st = baro_read_u8(hspi, BARO_STATUS, &s);
  if (st != HAL_OK) {
    return HAL_ERROR;
  }

  return ((s & BARO_STATUS_BOTH_DRDY) == BARO_STATUS_BOTH_DRDY) ? HAL_OK : HAL_BUSY;
}


/* ------ Altitude helpers ------ */

/* Sample is plausible for the sensor itself (works at apogee pressures) */
static inline uint8_t pressure_in_sensor_range(float p) {
  return isfinite(p) && p >= BARO_SENSOR_MIN_PA && p <= BARO_SENSOR_MAX_PA;
}
/* Sample is plausible as a pad/ground baseline */
static inline uint8_t pressure_is_valid_baseline(float p) {
  return isfinite(p) && p >= BARO_MIN_PA && p <= BARO_MAX_PA;
}

/*
 * Re-anchor so the given pressure reads exactly 0 m.
 */
static void baro_reset_baseline(float pressure) {
  baro_ref_reset(pressure, HAL_GetTick());
  prev_rel_alt = 0.0f;
}


/* ------ Public API: altitude ------ */

/*
 * Pressure -> altitude above the ground reference (baro_agl: hypsometric
 * equation with the ISA pad temperature, right at any site elevation).
 * Does not move the reference.
 */
float baro_relative_alt(float pressure) {
  if (!pressure_in_sensor_range(pressure)) {
    return prev_rel_alt;
  }

  if (!(baro_ref.ref > 0.0f)) {
    baro_reset_baseline(pressure);
    return 0.0f;
  }

  float rel_alt = baro_agl(pressure, baro_ref.ref);

  if (!isfinite(rel_alt)) {
    return prev_rel_alt;
  }

  prev_rel_alt = rel_alt;
  return rel_alt;
}

/*
 * Live sample path: track the ground reference against weather and sensor
 * drift while still on the pad (on_pad: before Launch), freeze it from
 * Launch on (baro_reference.h), then convert. Call once per NEW sample.
 */
float baro_track_alt(float pressure, bool on_pad) {
  last_tick = HAL_GetTick();
  if (pressure_in_sensor_range(pressure) && baro_ref.ref > 0.0f) {
    baro_ref_update(pressure, last_tick, on_pad);
  }
  return baro_relative_alt(pressure);
}

/*
 * Returns 1 if the most recent try-fetch returned a new sample, 0 if it
 * returned cached data.
 */
uint8_t baro_sample_was_fresh(void) {
  return last_fresh;
}


/* ------ Compensation math (datasheet §8.5/§8.6) ------ */

float baro_compensate_temp(uint32_t uncomp_temp) {
  float part_dat_1 = (float)uncomp_temp - cd.par_t1;
  float part_dat_2 = part_dat_1 * cd.par_t2;

  /* Update the compensated temperature in calib structure since
   * this is needed for pressure calculation */
  cd.t_lin = part_dat_2 + (part_dat_1*part_dat_1) * cd.par_t3;

  return cd.t_lin;
}

float baro_compensate_pres(uint32_t uncomp_press) {
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


/* ------ Sample read (shared by the getters below) ------ */

/*
 * Burst-read pressure+temperature, compensate, and update the cache.
 * Temperature is compensated first because pressure uses t_lin.
 */
static HAL_StatusTypeDef baro_read_sample(SPI_HandleTypeDef *hspi, float *temp_c, float *pres_pa) {
  uint8_t buf[6];

  HAL_StatusTypeDef st = baro_read_reg(hspi, BARO_DATA_0, buf, sizeof buf);
  if (st != HAL_OK) {
    return st;
  }

  uint32_t adc_p = U24(buf[0], buf[1], buf[2]);
  uint32_t adc_t = U24(buf[3], buf[4], buf[5]);

  float t_c  = baro_compensate_temp(adc_t);
  float p_pa = baro_compensate_pres(adc_p);

  last_temp  = t_c;
  last_pres  = p_pa;
  last_fresh = 1;

  if (temp_c) {
    *temp_c = t_c;
  }
  if (pres_pa) {
    *pres_pa = p_pa;
  }

  return HAL_OK;
}


/* ------ Public synchronous getters ------ */

HAL_StatusTypeDef baro_fetch_temp(SPI_HandleTypeDef *hspi, float *temperature_c) {
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
  last_temp = t_c;

  if (temperature_c) {
    *temperature_c = t_c;
  }

  return HAL_OK; 
}

HAL_StatusTypeDef baro_fetch_temp_pres(SPI_HandleTypeDef *hspi, float *temp_c, float *pres_pa) {
  if (baro_wait_drdy(hspi, 5) != HAL_OK) {
    return HAL_TIMEOUT;
  }

  return baro_read_sample(hspi, temp_c, pres_pa);
}

HAL_StatusTypeDef baro_fetch_all(SPI_HandleTypeDef *hspi, float *temp_c, float *pres_pa, float *alt_m) {
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

HAL_StatusTypeDef baro_fetch_pres(SPI_HandleTypeDef *hspi, float *pres_pa) {
  float temp;
  return baro_fetch_temp_pres(hspi, &temp, pres_pa);
}

float baro_get_alt(SPI_HandleTypeDef *hspi) {
  float p;

  if (baro_fetch_pres(hspi, &p) != HAL_OK) {
    return -1.0f;
  }

  return baro_relative_alt(p);
}


 /* ------ Try-get (non-spinning) getter variants ------ */

/*
 * Returns HAL_OK with cached values if no new sample is ready; use
 * baro_sample_was_fresh() to tell the difference.
 */
HAL_StatusTypeDef baro_try_fetch_temp_pres(SPI_HandleTypeDef *hspi, float *temp_c, float *pres_pa) {
  if (baro_check_drdy(hspi) != HAL_OK) {
    last_fresh = 0;

    if (temp_c) {
      *temp_c = last_temp;
    }
    if (pres_pa) {
      *pres_pa = last_pres;
    }
    return HAL_OK;
  }

  return baro_read_sample(hspi, temp_c, pres_pa);
}

/*
 * Altitude is only recomputed from a fresh sample. On cached data the previous
 * altitude is returned unchanged, so the drift logic never sees repeated
 * identical samples (or a zero pressure before the first read).
 */
HAL_StatusTypeDef baro_try_fetch_all(SPI_HandleTypeDef *hspi, float *temp_c, float *press_pa, float *alt_m) {
  HAL_StatusTypeDef st;
 
  st = baro_try_fetch_temp_pres(hspi, temp_c, press_pa);
  if (st != HAL_OK) {
    return st;
  }
 
  if (alt_m) {
    *alt_m = last_fresh ? baro_relative_alt(last_pres) : prev_rel_alt;
  }

  return HAL_OK;
}

HAL_StatusTypeDef baro_try_fetch_pres(SPI_HandleTypeDef *hspi, float *pres_pa) {
  float temp;
  return baro_try_fetch_temp_pres(hspi, &temp, pres_pa);
}
 
HAL_StatusTypeDef baro_try_fetch_temp(SPI_HandleTypeDef *hspi, float *temp_c) {
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

static __attribute__((noinline)) HAL_StatusTypeDef baro_init_failed(const char *operation)
{
  log_err("BARO: %s failed", operation);
  return HAL_ERROR;
}

/*
 * Initializes barometer with specified parameters.
 * OSR, ODR, and IIR values can be found in barometer.h.
 * Do not call from interrupts or callbacks. Idempotent.
 *
 * conf->rezero > 0 : measure and store a fresh ground baseline.
 * conf->rezero == 0: keep the existing baseline (e.g. after a watchdog
 *                    resume, so the altitude reference survives).
 */
HAL_StatusTypeDef baro_init(SPI_HandleTypeDef *hspi, const struct baro_config *conf) {
  HAL_StatusTypeDef st;
 
  /* Soft reset */
  uint8_t cmd = BARO_SOFTRESET_CMD;
 
  st = baro_write_reg(hspi, BARO_CMD, &cmd, 1);
  if (st != HAL_OK) {
    return baro_init_failed("reset");
  }

  HAL_Delay(BARO_RESET_DELAY_MS);

  /* Best effort: the CHIP_ID check below is the real gate */
  baro_wait_idle(hspi);

  /* Match chip ID */
  uint8_t val;

  st = baro_read_u8(hspi, BARO_CHIP_ID, &val);
  if (st != HAL_OK) {
    return baro_init_failed("ID read");
  }

  if (val != BARO_CHIP_ID_VALUE) {
    log_err("BARO: ID %02X, expected 60", val);
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
    return baro_init_failed("enable");
  }

  HAL_Delay(BARO_ENABLE_DELAY_MS);

  /* Configuration setup with default fallbacks */
  struct baro_config valid = {
    .osr_t = Baro_OSR_x1,
    .osr_p = Baro_OSR_x8,
    .odr = Baro_ODR_100,
    .iir_coef = Baro_IIR_Coef_3,
    .rezero = 1,
  };

  if (conf) {
    if (conf->osr_t <= Baro_OSR_x32) {
      valid.osr_t = conf->osr_t;
    }
    if (conf->osr_p <= Baro_OSR_x32) {
      valid.osr_p = conf->osr_p;
    }
    if (conf->odr <= Baro_ODR_0P0015) {
      valid.odr = conf->odr;
    }
    if (conf->iir_coef <= Baro_IIR_Coef_127) {
      valid.iir_coef = conf->iir_coef;
    }
    valid.rezero = conf->rezero;
  }

  /* Oversampling and data rates (C-f OSR / ODR) */
  st = baro_write_u8(hspi, BARO_OSR, baro_osr_make(valid.osr_t, valid.osr_p));
  if (st != HAL_OK) {
    return baro_init_failed("OSR");
  }

  st = baro_write_u8(hspi, BARO_ODR, valid.odr);
  if (st != HAL_OK) {
    return baro_init_failed("ODR");
  }

  HAL_Delay(BARO_ENABLE_DELAY_MS);

  /* IIR filter coefficient */
  st = baro_write_u8(hspi, BARO_CONFIG, valid.iir_coef);
  if (st != HAL_OK) {
    return baro_init_failed("IIR");
  }

  /* EXTI */
  st = baro_write_u8(hspi, BARO_INT_CTRL, BARO_INT_CTRL_VAL);
  if (st != HAL_OK) {
    return baro_init_failed("INT");
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

    log_err("BARO: normal mode failed.\n"
            "PWR=%02X STATUS=%02X ERR=%02X",
            pwr, status, err);
    return HAL_ERROR;
  }

  /* Read calibration parameters from barometer registers */
  st = read_trim_pars(hspi);
  if (st != HAL_OK) {
    return baro_init_failed("calibration read");
  }

  /* Cached samples from before the reset are no longer meaningful */
  last_fresh = 0;
  last_pres  = 0.0f;
  last_temp  = 0.0f;

  /* Measure a validated ground baseline if requested */
  if (valid.rezero > 0)
  {
    /* Let measurements settle (~2 frames) */
    uint8_t odr_sel = 0;
    baro_read_u8(hspi, BARO_ODR, &odr_sel);
    odr_sel &= BARO_ODR_SEL_MASK;
    uint32_t period_ms = BARO_PERIOD_MS_FROM_ODRSEL(odr_sel);
    HAL_Delay(2u * period_ms);

    float p = 0.0f, t = 0.0f;
    uint8_t ok = 0;

    for (uint8_t i = 0; i < 5; ++i)
    {
      /* baro_fetch_temp_pres() waits for data-ready itself */
      st = baro_fetch_temp_pres(hspi, &t, &p);
      if (st == HAL_OK && pressure_is_valid_baseline(p))
      {
        ok = 1;
        break; /* Validation successful */
      }

      HAL_Delay(10);
    }

    if (!ok) {
      log_err("BARO: baseline pressure invalid: %.1f Pa", p);
      return HAL_ERROR;
    }

    baro_reset_baseline(p);
  }

  return HAL_OK;
}

/*
 * Obtain and set new ground level pressure. Do not call in flight.
 */
void baro_rezero(SPI_HandleTypeDef *hspi) {
  HAL_StatusTypeDef st;
  float p = 0.0f, t = 0.0f;

  st = baro_fetch_temp_pres(hspi, &t, &p);

  if (st == HAL_OK && pressure_is_valid_baseline(p))
  {
    baro_reset_baseline(p);
  }
}