/*
 * Flight Computer 26 synchronous sensor tests.
 */

#include <assert.h>

#include "platform.h"
#include "testing.h"

extern SPI_HandleTypeDef hspi1;


static struct baro_config baro_conf = {
    .osr_t = BARO_OSR_X1,
    .osr_p = BARO_OSR_X2,
    .odr = BARO_DEFAULT_ODR_SEL,
    .iir_coef = BARO_IIR_COEF_0,
};

static struct gyro_config gyro_conf = {
    .rng = Gyro_Range_2000Dps,
    .bw = Gyro_532Hz_ODR_2000Hz,
};

static struct accl_config accl_conf = {
    .mode = Normal_1600Hz,
    .rng = Accl_Range_24g,
};


void test_baro_sync(SPI_HandleTypeDef *hspi, int precise)
{
	if (precise)
	{
		baro_conf.osr_p = BARO_OSR_X8;
  	baro_conf.iir_coef = BARO_IIR_COEF_15;
	}

	assert(init_baro(&baro_conf) == HAL_OK);

	HAL_StatusTypeDef st;
	/* x - temp, y - pres, z - alt */
	struct coords q = {0};

	for (int k = 0; k < SENSOR_SYNC_STEPS; ++k)
	{
		st = baro_fetch_all(hspi, &q.x, &q.y, &q.z);

		if (st == HAL_OK)
		{
			log_measurement(SEDS_DT_BAROMETER_DATA, &q);
		}
		else
		{
			log_err("Baro: sync fetch failed: %d", st);
		}
	}
}


void test_gyro_sync(SPI_HandleTypeDef *hspi, int lowpower)
{
	if (lowpower)
	{
		gyro_conf.bw = Gyro_47Hz_ODR_400Hz;
	}

	assert(init_gyro(&gyro_conf) == HAL_OK);

	HAL_StatusTypeDef st;
	struct coords q = {0};

	for (int k = 0; k < SENSOR_SYNC_STEPS; ++k)
	{
		st = gyro_read(hspi, &q);

		if (st == HAL_OK)
		{
			log_measurement(SEDS_DT_GYRO_DATA, &q);
		}
		else
		{
			log_err("Gyro: sync fetch failed: %d", st);
		}
	}
}


void test_accl_sync(SPI_HandleTypeDef *hspi, int lowpower)
{
	if (lowpower)
	{
		accl_conf.mode = Normal_400Hz;
	}

	assert(init_accl(&accl_conf) == HAL_OK);

	HAL_StatusTypeDef st;
	struct coords q = {0};

	for (int k = 0; k < SENSOR_SYNC_STEPS; ++k)
	{
		st = accl_read(hspi, &q);

		if (st == HAL_OK)
		{
			log_measurement(SEDS_DT_ACCEL_DATA, &q);
		}
		else
		{
			log_err("Accl: sync fetch failed: %d", st);
		}
	}
}


void noreturn test_sensors_sync(void)
{
	test_baro_sync(&hspi1, 0);
	test_baro_sync(&hspi1, 1);
	test_gyro_sync(&hspi1, 0);
	test_gyro_sync(&hspi1, 1);
	test_accl_sync(&hspi1, 0);
	test_accl_sync(&hspi1, 1);

	_Exit(0);
}