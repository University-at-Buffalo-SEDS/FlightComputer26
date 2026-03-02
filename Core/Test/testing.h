/*
 * Flight Computer 26 testing definitions and API.
 */

#ifndef TESTING_H
#define TESTING_H


#define SENSOR_SYNC_STEPS 128

#define MAX_FETCH_INTERVAL_MS 50

#define MAX_ISR_CYCLES 16
#define MAX_SYNC_CYCLES 16

#define random_wait HAL_Delay(rand() % MAX_FETCH_INTERVAL_MS);

#define noreturn __attribute__ ((noreturn))


void test_baro_sync(SPI_HandleTypeDef *hspi, int precise);
void test_gyro_sync(SPI_HandleTypeDef *hspi, int lowpower);
void test_accl_sync(SPI_HandleTypeDef *hspi, int lowpower);
void noreturn test_sensors_sync(void);

void baro_dma_isr(int precise);
void imu_dma_isr(int lowpower);
void combination_dma_isr(int precise, int lowpower);
void baro_dma_sync(int precise);
void imu_dma_sync(int lowpower);
void noreturn dma_sensor_test(void);


#endif // TESTING_H