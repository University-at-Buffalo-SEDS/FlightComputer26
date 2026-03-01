/*
 * Flight Computer 26 testing definitions and API.
 */

#ifndef TESTING_H
#define TESTING_H

#define SENSOR_SYNC_STEPS 128

#define noreturn __attribute__ ((noreturn))

void test_baro_sync(SPI_HandleTypeDef *hspi, int precise);
void test_gyro_sync(SPI_HandleTypeDef *hspi, int lowpower);
void test_accl_sync(SPI_HandleTypeDef *hspi, int lowpower);
void noreturn test_sensors_sync(void);

#endif // TESTING_H