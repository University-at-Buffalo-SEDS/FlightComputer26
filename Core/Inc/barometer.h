/*
 * BMP390 Barometer configuration and API.
 */

#ifndef BAROMETER_H
#define BAROMETER_H

#include "main.h"


// ==============================
// BMP390 register map
// ==============================

#define BARO_CHIP_ID 0x00
#define BARO_REV_ID 0x01
#define BARO_ERR_REG 0x02
#define BARO_STATUS 0x03
#define BARO_DATA_0 0x04
#define BARO_DATA_1 0x05
#define BARO_DATA_2 0x06
#define BARO_DATA_3 0x07
#define BARO_DATA_4 0x08
#define BARO_DATA_5 0x09
#define BARO_SENSORTIME_0 0x0C
#define BARO_SENSORTIME_1 0x0D
#define BARO_SENSORTIME_2 0x0E
#define BARO_EVENT 0x10
#define BARO_INT_STATUS 0x11
#define BARO_FIFO_LENGTH_0 0x12
#define BARO_FIFO_LENGTH_1 0x13
#define BARO_FIFO_DATA 0x14
#define BARO_FIFO_WTM_0 0x15
#define BARO_FIFO_WTM_1 0x16
#define BARO_FIFO_CONFIG_0 0x17
#define BARO_FIFO_CONFIG_1 0x18
#define BARO_INT_CTRL 0x19
#define BARO_IF_CONF 0x1A
#define BARO_PWR_CTRL 0x1B
#define BARO_OSR 0x1C
#define BARO_ODR 0x1D
#define BARO_CONFIG 0x1F
#define BARO_CMD 0x7E

// Trim (NVM) parameters (0x31..0x45)
#define BARO_NVM_PAR_T1 0x31
#define BARO_NVM_PAR_T2 0x33
#define BARO_NVM_PAR_T3 0x35
#define BARO_NVM_PAR_P1 0x36
#define BARO_NVM_PAR_P2 0x38
#define BARO_NVM_PAR_P3 0x3A
#define BARO_NVM_PAR_P4 0x3B
#define BARO_NVM_PAR_P5 0x3C
#define BARO_NVM_PAR_P6 0x3E
#define BARO_NVM_PAR_P7 0x40
#define BARO_NVM_PAR_P8 0x41
#define BARO_NVM_PAR_P9 0x42
#define BARO_NVM_PAR_P10 0x44
#define BARO_NVM_PAR_P11 0x45


// ==============================
// Chip constants & helpers
// ==============================

// Chip ID
#define BARO_CHIP_ID_VALUE 0x60

// SPI
#define BARO_SPI_READ_BIT 0x80u
#define BARO_SPI_WRITE_MASK 0x7Fu
#define BARO_SPI_TIMEOUT_MS 100u

// GPIO (chip select)
#define BARO_GPIO_PIN  CS_BARO_Pin
#define BARO_GPIO_PORT CS_BARO_GPIO_Port

#define baro_cs_low()                                               \
  HAL_GPIO_WritePin(BARO_GPIO_PORT, BARO_GPIO_PIN, GPIO_PIN_RESET)
#define baro_cs_high()                                              \
  HAL_GPIO_WritePin(BARO_GPIO_PORT, BARO_GPIO_PIN, GPIO_PIN_SET)

// Soft reset & delays
#define BARO_SOFTRESET_CMD 0xB6
#define BARO_RESET_DELAY_MS 50u
#define BARO_MODE_SWITCH_DELAY_MS 10u
#define BARO_ENABLE_DELAY_MS 5u

// STATUS bits (0x03)
#define BARO_STATUS_TEMP_DRDY   (1u << 6)
#define BARO_STATUS_PRESS_DRDY  (1u << 5)
#define BARO_STATUS_BOTH_DRDY                                       \
  (BARO_STATUS_TEMP_DRDY | BARO_STATUS_PRESS_DRDY)

// PWR_CTRL (0x1B)
#define BARO_PWR_EN_PRESS (1u << 0)
#define BARO_PWR_EN_TEMP  (1u << 1)
#define BARO_PWR_ENABLE_SENSORS                                     \
  (uint8_t)(BARO_PWR_EN_PRESS | BARO_PWR_EN_TEMP)

#define BARO_PWR_MODE_SHIFT   4
#define BARO_PWR_MODE_MASK    (0x3u << BARO_PWR_MODE_SHIFT)
#define BARO_PWR_MODE_SLEEP   (0x0u << BARO_PWR_MODE_SHIFT)
#define BARO_PWR_MODE_FORCED  (0x1u << BARO_PWR_MODE_SHIFT)
#define BARO_PWR_MODE_NORMAL  (0x3u << BARO_PWR_MODE_SHIFT)

#define BARO_PWR_NORMAL_WITH_SENSORS                                \
  (uint8_t)(BARO_PWR_ENABLE_SENSORS | BARO_PWR_MODE_NORMAL)

// OSR (0x1C)
#define BARO_OSR_T_SHIFT 0
#define BARO_OSR_P_SHIFT 3

#define baro_osr_make(osr_t, osr_p)                                 \
  (((osr_t) << BARO_OSR_T_SHIFT) | ((osr_p) << BARO_OSR_P_SHIFT))

#define BARO_OSR_X1 0u
#define BARO_OSR_X2 1u
#define BARO_OSR_X4 2u
#define BARO_OSR_X8 3u
#define BARO_OSR_X16 4u
#define BARO_OSR_X32 5u

// ODR 
#define BARO_ODR_200    0x00
#define BARO_ODR_100    0x01
#define BARO_ODR_50     0x02
#define BARO_ODR_25     0x03
#define BARO_ODR_12P5   0x04
#define BARO_ODR_6P25   0x05
#define BARO_ODR_3P1    0x06
#define BARO_ODR_1P5    0x07
#define BARO_ODR_0P78   0x08
#define BARO_ODR_0P39   0x09
#define BARO_ODR_0P2    0x0A
#define BARO_ODR_0P1    0x0B
#define BARO_ODR_0P05   0x0C
#define BARO_ODR_0P02   0x0D
#define BARO_ODR_0P01   0x0E
#define BARO_ODR_0P006  0x0F
#define BARO_ODR_0P003  0x10
#define BARO_ODR_0P0015 0x11

// IIR filter coefficients
#define BARO_IIR_COEF_0    0x00   /* Bypass, default */
#define BARO_IIR_COEF_1    0x01
#define BARO_IIR_COEF_3    0x02
#define BARO_IIR_COEF_7    0x03
#define BARO_IIR_COEF_15   0x04
#define BARO_IIR_COEF_31   0x05
#define BARO_IIR_COEF_63   0x06
#define BARO_IIR_COEF_127  0x07

// Default OSR/ODR
#define BARO_DEFAULT_OSR                                            \
  baro_osr_make(BARO_OSR_X1, BARO_OSR_X2)   // T x1, P x2  -> 0x0D
#define BARO_ODR_SEL_MASK 0x1Fu
#define BARO_DEFAULT_ODR_SEL 0x04u          // 12.5 Hz
#define BARO_PERIOD_MS_FROM_ODRSEL(s)                               \
  (5u << ((s) & BARO_ODR_SEL_MASK))         // approx

// Valid pressure sanity range (Pa)
#define BARO_MIN_PA 70000.0f
#define BARO_MAX_PA 110000.0f

// Hypsometric helpers
#define BARO_SEA_LEVEL_PRESSURE_PA  101325.0f
#define BARO_HYPSOMETRIC_EXPONENT   0.1903f

// Tunables to prevent drift

// +/- band around 0 m to avoid chattering
#define BARO_HYST_DEADBAND 0.5f 
// recalibrate every ~5 m from the last baseline
#define BARO_RESET_STEP_M 5.0f

// Size of one DMA record
#define BARO_DMA_BUF_SIZE 8

// Condition to check if normal mode was entered
#define is_in_normal_mode(rb)                                     \
  (((rb) & BARO_PWR_MODE_MASK) == BARO_PWR_MODE_NORMAL) &&        \
  (((rb) & BARO_PWR_ENABLE_SENSORS) == BARO_PWR_ENABLE_SENSORS)


// ==============================
// Types
// ==============================
struct baro_calibration {
  /* Scaled, floating-point coefficients (datasheet §8.4) */
  float par_t1, par_t2, par_t3;

  float par_p1, par_p2, par_p3;
  float par_p4, par_p5, par_p6;
  float par_p7, par_p8, par_p9;
  float par_p10, par_p11;

  /* Computed by temperature compensation */
  float t_lin;
};

struct baro_config {
  uint8_t osr_t, osr_p, odr, iir_coef;
};


// ==============================
// API
// ==============================

/* Compensation functions (datasheet §8.5/§8.6) */
float baro_compensate_temp(uint32_t uncomp_temp);
float baro_compensate_pres(uint32_t uncomp_press);

float baro_relative_alt(float pressure);
float baro_get_alt(SPI_HandleTypeDef *hspi);

/* Public synchronous getters */
HAL_StatusTypeDef baro_fetch_temp(SPI_HandleTypeDef *hspi, float *temperature_c);
HAL_StatusTypeDef baro_fetch_temp_pres(SPI_HandleTypeDef *hspi, float *temp_c, float *pres_pa);
HAL_StatusTypeDef baro_fetch_all(SPI_HandleTypeDef *hspi, float *temp_c, float *pres_pa, float *alt_m);
HAL_StatusTypeDef baro_fetch_pres(SPI_HandleTypeDef *hspi, float *pres_pa);

/* Try-get (non-spinning) getter variants */
HAL_StatusTypeDef baro_try_fetch_temp_pres(SPI_HandleTypeDef *hspi, float *temp_c, float *pres_pa);
HAL_StatusTypeDef baro_try_fetch_all(SPI_HandleTypeDef *hspi, float *temp_c, float *press_pa, float *alt_m);
HAL_StatusTypeDef baro_try_fetch_pres(SPI_HandleTypeDef *hspi, float *pres_pa);
HAL_StatusTypeDef baro_try_fetch_temp(SPI_HandleTypeDef *hspi, float *temp_c);

/* Initialization */
HAL_StatusTypeDef baro_init(SPI_HandleTypeDef *hspi, const struct baro_config *conf);
void baro_rezero(SPI_HandleTypeDef *hspi);


#endif // BAROMETER_H