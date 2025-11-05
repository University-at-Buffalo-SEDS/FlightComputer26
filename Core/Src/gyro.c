#include "gyro.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

static inline void gyro_cs_low(void)  { HAL_GPIO_WritePin(CS_GYRO_GPIO_Port, CS_GYRO_Pin, GPIO_PIN_RESET); }
static inline void gyro_cs_high(void) { HAL_GPIO_WritePin(CS_GYRO_GPIO_Port, CS_GYRO_Pin, GPIO_PIN_SET);   }

/* Single-byte write: [addr(bit7=0)] [data] */
HAL_StatusTypeDef gyro_write_register(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = { GYRO_CMD_WRITE(reg), value };
    gyro_cs_low();
    HAL_StatusTypeDef st = HAL_SPI_Transmit(hspi, tx, sizeof(tx), HAL_MAX_DELAY);
    gyro_cs_high();
    return st;
}

/* Single-byte read: TX addr(bit7=1), then RX data */
HAL_StatusTypeDef gyro_read_register(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t *data)
{
    if (!data) return HAL_ERROR;
    uint8_t cmd = GYRO_CMD_READ(reg);
    gyro_cs_low();
    HAL_StatusTypeDef st = HAL_SPI_Transmit(hspi, &cmd, 1, HAL_MAX_DELAY);
    if (st == HAL_OK) st = HAL_SPI_Receive(hspi, data, 1, HAL_MAX_DELAY);
    gyro_cs_high();
    return st;
}

/* Burst read: send addr(bit7=1), then receive N bytes (auto-increment is implicit for BMI088) */
HAL_StatusTypeDef gyro_read_buffer(SPI_HandleTypeDef *hspi, uint8_t start_reg, uint8_t *dst, uint16_t len)
{
    if (!dst || !len) return HAL_ERROR;
    uint8_t cmd = GYRO_CMD_READ(start_reg);
    gyro_cs_low();
    HAL_StatusTypeDef st = HAL_SPI_Transmit(hspi, &cmd, 1, HAL_MAX_DELAY);
    if (st == HAL_OK) st = HAL_SPI_Receive(hspi, dst, len, HAL_MAX_DELAY);
    gyro_cs_high();
    return st;
}

/* BMI088 gyro init (datasheet §3, §4.1.2, §5.5.x) */
HAL_StatusTypeDef gyro_init(SPI_HandleTypeDef *hspi)
{
    /* Wait for gyro startup (tsu ~30ms) after power-up or before first access) */
    HAL_Delay(30);

    /* WHO_AM_I should be 0x0F at 0x00 */
    uint8_t id = 0;
    HAL_StatusTypeDef st = gyro_read_register(hspi, GYRO_CHIP_ID, &id);
    if (st != HAL_OK) return st;
    if (id != GYRO_CHIP_ID_VALUE) {
        printf("Gyro WHOAMI mismatch: 0x%02X (exp 0x0F)\n", id);
        return HAL_ERROR;
    }

    /* Soft reset (0xB6 → 0x14), then wait ≥30ms */
    st = gyro_write_register(hspi, GYRO_SOFTRESET, 0xB6);
    if (st != HAL_OK) return st;
    HAL_Delay(30);

    /* Normal mode (write 0x00 to GYRO_LPM1 per §4.1.2) */
    st = gyro_write_register(hspi, GYRO_LPM1, 0x00);
    if (st != HAL_OK) return st;
    HAL_Delay(30);

    /* Range & bandwidth (examples) */
    st = gyro_write_register(hspi, GYRO_RANGE,     GYRO_RANGE_DPS);
    if (st != HAL_OK) return st;
    st = gyro_write_register(hspi, GYRO_BANDWIDTH, GYRO_BW_523HZ_ODR_2000HZ);
    if (st != HAL_OK) return st;

    return HAL_OK;
}

/* Read XYZ (LSB first then MSB) — datasheet §5.5.2 */
HAL_StatusTypeDef gyro_read(SPI_HandleTypeDef *hspi, gyro_data_t *g)
{
    uint8_t buf[6];
    HAL_StatusTypeDef st = gyro_read_buffer(hspi, GYRO_RATE_X_LSB, buf, sizeof(buf));
    if (st != HAL_OK) return st;

    g->rate_x = (int16_t)((uint16_t)buf[1] << 8 | buf[0]);
    g->rate_y = (int16_t)((uint16_t)buf[3] << 8 | buf[2]);
    g->rate_z = (int16_t)((uint16_t)buf[5] << 8 | buf[4]);
    return HAL_OK;
}

/* Get sensitivity (LSB per °/s) for given range */
static float gyro_sensitivity_lsb_per_dps(GyroRange range)
{
    switch (range) {
    case GYRO_RANGE_125DPS:  return 262.144f;
    case GYRO_RANGE_250DPS:  return 131.072f;
    case GYRO_RANGE_500DPS:  return 65.536f;
    case GYRO_RANGE_1000DPS: return 32.768f;
    case GYRO_RANGE_2000DPS: return 16.384f;
    default:                 return 16.384f; // safe fallback
    }
}

/* Convert one raw axis reading to degrees per second */
float gyro_raw_to_dps(int16_t raw)
{
    return (float)raw / gyro_sensitivity_lsb_per_dps(GYRO_RANGE_DPS);
}

/* Convert full struct to degrees per second */
void gyro_convert_to_dps(const gyro_data_t *raw, float *x_dps, float *y_dps, float *z_dps)
{
    if (!raw) return;
    if (x_dps) *x_dps = gyro_raw_to_dps(raw->rate_x);
    if (y_dps) *y_dps = gyro_raw_to_dps(raw->rate_y);
    if (z_dps) *z_dps = gyro_raw_to_dps(raw->rate_z);
}