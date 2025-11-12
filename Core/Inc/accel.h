#include <stdint.h>
#include "main.h"
#include "stm32h5xx_hal_gpio.h"

// REGISTER MAPPING
#define ACCEL_RESET         0x7E
#define ACCEL_CHIP_ID       0x00
#define ACCEL_CONF          0x40
#define ACCEL_POWER_CTRL    0x7D
#define ACCEL_RANGE         0x41    

#define ACCEL_Z_MSB         0x17
#define ACCEL_Z_LSB         0x16
#define ACCEL_Y_MSB         0x15
#define ACCEL_Y_LSB         0x14
#define ACCEL_X_MSB         0x13
#define ACCEL_X_LSB         0x12

// ACCEL CONFIGS
#define ACCEL_RESET_VAL     0xB6
#define ACCEL_RANGE_VAL     0x03
#define ACCEL_CONF_VAL      0x28
#define ACCEL_BUF_SIZE      6

#define ACCEL_CMD_READ(reg)  ((uint8_t)((reg) | 0x80u))
#define ACCEL_CMD_WRITE(reg) ((uint8_t)((reg) & ~0x80u))

#define ACCEL_CS_LOW()    { HAL_GPIO_WritePin(CS_ACCEL_GPIO_Port, CS_ACCEL_Pin, GPIO_PIN_RESET); }
#define ACCEL_CS_HIGH()   { HAL_GPIO_WritePin(CS_ACCEL_GPIO_Port, CS_ACCEL_Pin, GPIO_PIN_SET);   }

typedef enum {
    POWER_ON = 0x04,
    POWER_OFF = 0x00
} accel_power;

typedef enum {
    ACCEL_RANGE_3g = 0x00,
    ACCEL_RANGE_6g = 0x01,
    ACCEL_RANGE_12g = 0x02,
    ACCEL_RANGE_24g = 0x03
} AccelRange;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} accelData_t;

//write 1 byte to a register address
HAL_StatusTypeDef accel_write_reg(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t data);

//read 1 byte from a register address
HAL_StatusTypeDef accel_read_reg(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t *data);

//Read from multiple registers using auto increment
HAL_StatusTypeDef accel_read_buffer(SPI_HandleTypeDef *hspi, uint8_t start_reg, uint8_t *dst, uint16_t len);

//configure the accelerometer
HAL_StatusTypeDef accel_init(SPI_HandleTypeDef *hspi);

//Read X axis data
HAL_StatusTypeDef accel_read(SPI_HandleTypeDef *hspi, accelData_t *accelData);

//Convert raw accelerometer data to mg
void convert_raw_accel_to_mg(accelData_t *data, float *x, float *y, float *z);