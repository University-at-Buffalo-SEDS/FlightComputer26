#include "main.h"
#include "stm32h5xx_hal.h"
#include "accel.h"
#include "stdio.h"

//write 1 byte to a register address
HAL_StatusTypeDef accel_write_reg(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t data){
  uint8_t buffer[2] = {((reg) & ~0x80u), data};
  ACCEL_CS_LOW(); //select accel chip
  HAL_StatusTypeDef status = HAL_SPI_Transmit(hspi, buffer, sizeof(buffer), 100);
  ACCEL_CS_HIGH();
  return status;
}

/* Single Byte Read from given register, must ignore dummy byte */
HAL_StatusTypeDef accel_read_reg(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t *data){
  if (!data) return HAL_ERROR;
  uint8_t reg_addr = reg | 0x80; //adding read bit to register address
  uint8_t tx_buffer[3] = {reg_addr, 0x00, 0x00};
  uint8_t rx_buffer[3];
  ACCEL_CS_LOW();
  HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(hspi, tx_buffer, rx_buffer, 3, 100); 
  ACCEL_CS_HIGH();
  if (status == HAL_ERROR) return status;
  *data = rx_buffer[2]; //Select last byte as actual info, then store it where the parameters point to
  return status;
}

//Burst read function using auto-increment for BMI088
HAL_StatusTypeDef accel_read_buffer(SPI_HandleTypeDef *hspi, uint8_t start_reg, uint8_t *dst, uint16_t len){
  if (!dst || !len) return HAL_ERROR;
  uint8_t reg_addr = start_reg | 0x80;
  uint8_t dummy = 0x00;
  ACCEL_CS_LOW();
  HAL_StatusTypeDef status = HAL_SPI_Transmit(hspi, &reg_addr, 1, HAL_MAX_DELAY);

  //DUMMY BYTE HANDLING
    if (status == HAL_OK) {
    status = HAL_SPI_TransmitReceive(hspi, &dummy, &dummy, 1, HAL_MAX_DELAY);
  }

  if (status == HAL_OK) status = HAL_SPI_Receive(hspi, dst, len, HAL_MAX_DELAY);
  ACCEL_CS_HIGH();
  return status;
}

//configure the accelerometer
HAL_StatusTypeDef accel_init(SPI_HandleTypeDef *hspi)
{
  HAL_Delay(30);
  HAL_StatusTypeDef status;

  uint8_t data = 0;
  /* WHO_AM_I should be 0x1E at 0x00 (Taken directly from Gyro Driver)*/ 
  status = accel_read_reg(hspi, accel_chip_id_addr, &data);
  if (status != HAL_OK) return status;
  if (data != 0x1E) {
    printf("Accel WHOAMI mismatch: 0x%02X (exp 0x1E)\n", data);
    return HAL_ERROR;
    }

  status = accel_write_reg(hspi, accel_pwr_ctrl, POWER_ON); //power on
  if (status != HAL_OK) return status;
  HAL_Delay(30); 



  //soft reset
  status = accel_write_reg(hspi, accel_reset_addr, 0xB6);
  if (status != HAL_OK) return status;
  HAL_Delay(2);

  //range set to ±24g
  status = accel_write_reg(hspi, accel_range_addr, 0x03); 
  if (status != HAL_OK) return status;
  HAL_Delay(1);

  //Bandwith of low pass filter config to normal and ODR set to 1600hz 
  status = accel_write_reg(hspi, accel_conf_addr, ((0x0A << 4) | 0x0C)); //
  if (status != HAL_OK) return status;
  HAL_Delay(5);

  return HAL_OK;
}


//read the accelermoter axis data
HAL_StatusTypeDef accel_read(SPI_HandleTypeDef *hspi, accelData_t *accelData){
  uint8_t rxBuffer[6];
  HAL_StatusTypeDef status = accel_read_buffer(hspi, 0x12, rxBuffer, 6);
  if (status != HAL_OK){
    return status;
  }
  accelData->x = (int16_t)((uint16_t)(rxBuffer[1] << 8) | rxBuffer[0]); //set struct values to raw data from accel
  accelData->y = (int16_t)((uint16_t)(rxBuffer[3] << 8) | rxBuffer[2]);
  accelData->z = (int16_t)((uint16_t)(rxBuffer[5] << 8) | rxBuffer[4]);
  return HAL_OK;
}

/*
static float accel_sensitivity_mg_per_lsb(AccelRange range)
{
    switch (range) {
    case ACCEL_RANGE_3g:  return 10.923f;
    case ACCEL_RANGE_6g:  return 5.461f;
    case ACCEL_RANGE_12g:  return 2.730f;
    case ACCEL_RANGE_24g: return 1.365f;
    default:                 return 1.365f; // safe fallback
    }
}
*/

void convert_raw_accel_to_mg(accelData_t *data, float *x, float *y, float *z){
  float mg_per_lsb = 24000.0f / 32768.0f;
  *x = data->x * mg_per_lsb;
  *y = data->y * mg_per_lsb;
  *z = data->z * mg_per_lsb;
}



//USE CALLBACK INTERRUPT FOR READING DATA FROM ACCEL IN MAIN.c
//DMA IS FASTER, BYPASSES CPU TO WRITE AND READ DATA FROM MEMORY
//UART WRITE
//ADC READ
//volatile variables for interrupt routines, It means other process like a callback can change the var at anytime.
//INTERRUPTS ARE STORED OUTSIDE OF MAIN FUNCTION AND NEED SPECIAL VARS TO BE USED^