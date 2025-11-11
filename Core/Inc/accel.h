//REGISTER MAPPING
#include <stdint.h>
#define accel_reset_addr    0x7E
#define accel_chip_id_addr  0x00
#define accel_conf_addr     0x40
#define accel_pwr_ctrl      0x00
#define accel_range_addr    0x41    


#define accel_z_msb         0x17
#define accel_z_lsb         0x16

#define accel_y_msb         0x15
#define accel_y_lsb         0x14

#define accel_x_msb         0x13
#define accel_x_lsb         0x12

//ACCEL CONFIGS
#define accel_reset_val     0xB6
#define accel_range_val     0x03
#define accel_conf_val      0x28


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