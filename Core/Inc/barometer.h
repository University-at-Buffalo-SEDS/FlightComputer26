#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stdatomic.h>
#include <math.h>
#include "main.h"
#include "stm32h5xx_hal.h"

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
#define BMP390_CHIP_ID_VALUE 0x60

// SPI
#define BMP390_SPI_READ_BIT 0x80u
#define BMP390_SPI_WRITE_MASK 0x7Fu
#define BMP390_SPI_TIMEOUT_MS 100u

// GPIO (chip select)
#define BAROMETER_GPIO_PIN CS_BARO_Pin
#define BAROMETER_GPIO_PORT CS_BARO_GPIO_Port

#define BARO_CS_LOW()                                                          \
  HAL_GPIO_WritePin(BAROMETER_GPIO_PORT, BAROMETER_GPIO_PIN, GPIO_PIN_RESET)
#define BARO_CS_HIGH()                                                         \
  HAL_GPIO_WritePin(BAROMETER_GPIO_PORT, BAROMETER_GPIO_PIN, GPIO_PIN_SET)

// Soft reset & delays
#define BMP390_SOFTRESET_CMD 0xB6
#define BMP390_RESET_DELAY_MS 50u
#define BMP390_MODE_SWITCH_DELAY_MS 10u
#define BMP390_ENABLE_DELAY_MS 5u

// STATUS bits (0x03)
#define BMP390_STATUS_TEMP_DRDY (1u << 6)
#define BMP390_STATUS_PRESS_DRDY (1u << 5)
#define BMP390_STATUS_BOTH_DRDY                                                \
  (BMP390_STATUS_TEMP_DRDY | BMP390_STATUS_PRESS_DRDY)

// PWR_CTRL (0x1B)
#define BMP390_PWR_EN_PRESS (1u << 0)
#define BMP390_PWR_EN_TEMP (1u << 1)
#define BMP390_PWR_ENABLE_SENSORS (BMP390_PWR_EN_PRESS | BMP390_PWR_EN_TEMP)

#define BMP390_PWR_MODE_SHIFT 4
#define BMP390_PWR_MODE_MASK (0x3u << BMP390_PWR_MODE_SHIFT)
#define BMP390_PWR_MODE_SLEEP (0x0u << BMP390_PWR_MODE_SHIFT)
#define BMP390_PWR_MODE_FORCED (0x1u << BMP390_PWR_MODE_SHIFT)
#define BMP390_PWR_MODE_NORMAL (0x3u << BMP390_PWR_MODE_SHIFT)

#define BMP390_PWR_NORMAL_WITH_SENSORS                                         \
  (BMP390_PWR_ENABLE_SENSORS | BMP390_PWR_MODE_NORMAL)

// OSR (0x1C)
#define BMP390_OSR_T_SHIFT 0
#define BMP390_OSR_P_SHIFT 3
#define BMP390_OSR_MAKE(osr_t, osr_p)                                          \
  (((osr_t) << BMP390_OSR_T_SHIFT) | ((osr_p) << BMP390_OSR_P_SHIFT))
#define BMP390_OSR_X1 0u
#define BMP390_OSR_X2 1u
#define BMP390_OSR_X4 2u
#define BMP390_OSR_X8 3u
#define BMP390_OSR_X16 4u
#define BMP390_OSR_X32 5u

// Default OSR/ODR
#define BMP390_DEFAULT_OSR                                                     \
  BMP390_OSR_MAKE(BMP390_OSR_X1, BMP390_OSR_X2) // T x1, P x2  -> 0x0D
#define BMP390_ODR_SEL_MASK 0x1Fu
#define BMP390_DEFAULT_ODR_SEL 0x04u // 12.5 Hz
#define BMP390_PERIOD_MS_FROM_ODRSEL(s)                                        \
  (5u << ((s) & BMP390_ODR_SEL_MASK)) // approx

// Valid pressure sanity range (Pa)
#define BMP390_BARO_VALID_MIN_PA 70000.0f
#define BMP390_BARO_VALID_MAX_PA 110000.0f

// Hypsometric helpers
#define BMP390_SEA_LEVEL_PRESSURE_PA 101325.0f
#define BMP390_HYPSOMETRIC_EXPONENT 0.1903f

// Tunables to prevent drift
#define HYST_DEADBAND 0.5f // +/- band around 0 m to avoid chattering
#define RESET_STEP_M 5.0f  // recalibrate every ~5 m from the last baseline

// Size of one DMA record (DATA 0-5)
#define BMP390_BUF_SIZE 8

// ==============================
// Types
// ==============================
typedef struct {
  // scaled, floating-point coefficients (datasheet §8.4)
  float par_t1;
  float par_t2;
  float par_t3;

  float par_p1;
  float par_p2;
  float par_p3;
  float par_p4;
  float par_p5;
  float par_p6;
  float par_p7;
  float par_p8;
  float par_p9;
  float par_p10;
  float par_p11;

  float t_lin; // computed by temperature compensation
} BMP390_calib_data_t;

typedef struct {
  float pres;
  float temp;
  float alt;
} baro_data_t;

// Extern variables (defined in barometer.c)
extern BMP390_calib_data_t calib_data;
extern float ground_level_pressure;

// ==============================
// API
// ==============================
HAL_StatusTypeDef init_barometer(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef get_temperature_pressure(SPI_HandleTypeDef *hspi,
                                           float *temperature_c,
                                           float *pressure_pa);
HAL_StatusTypeDef get_temperature_pressure_altitude(SPI_HandleTypeDef *hspi,
                                                    float *temperature_c,
                                                    float *pressure_pa,
                                                    float *altitude_m);
HAL_StatusTypeDef get_pressure(SPI_HandleTypeDef *hspi, float *pressure_pa);
HAL_StatusTypeDef get_temperature(SPI_HandleTypeDef *hspi,
                                  float *temperature_c);

// Non-blocking convenience
HAL_StatusTypeDef get_temperature_pressure_non_blocking(SPI_HandleTypeDef *hspi,
                                                        float *temperature_c,
                                                        float *pressure_pa);
HAL_StatusTypeDef get_temperature_pressure_altitude_non_blocking(
    SPI_HandleTypeDef *hspi, float *temperature_c, float *pressure_pa,
    float *altitude_m);
HAL_StatusTypeDef get_pressure_non_blocking(SPI_HandleTypeDef *hspi,
                                            float *pressure_pa);

// Low-level helpers
HAL_StatusTypeDef baro_read_u8(SPI_HandleTypeDef *hspi, uint8_t reg,
                               uint8_t *val);
HAL_StatusTypeDef baro_write_u8(SPI_HandleTypeDef *hspi, uint8_t reg,
                                uint8_t val);
HAL_StatusTypeDef baro_read_reg(SPI_HandleTypeDef *hspi, uint8_t reg,
                                uint8_t *out, uint16_t len);
HAL_StatusTypeDef baro_write_reg(SPI_HandleTypeDef *hspi, uint8_t reg,
                                 const uint8_t *data, uint16_t len);
HAL_StatusTypeDef get_temperature_non_blocking(SPI_HandleTypeDef *hspi,
                                               float *temperature_c);

// Data helpers
void baro_rezero(SPI_HandleTypeDef *hspi);
float compute_relative_altitude(float pressure);
float get_relative_altitude(SPI_HandleTypeDef *hspi);
float get_relative_altitude_non_blocking(SPI_HandleTypeDef *hspi);
float compensate_temperature(uint32_t uncomp_temp);
float compensate_pressure(uint32_t uncomp_press);

inline uint32_t u24(uint8_t b0, uint8_t b1, uint8_t b2) {
  return ((uint32_t)b2 << 16) | ((uint32_t)b1 << 8) | (uint32_t)b0;
}