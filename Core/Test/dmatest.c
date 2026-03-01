/*
 * Flight Computer 26 DMA sensor tests.
 */

#include <assert.h>

#include "platform.h"
#include "testing.h"
#include "dma.h"


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


static inline void reset(void)
{
	__NVIC_DisableIRQ(Baro_EXTI);
  __NVIC_DisableIRQ(Gyro_EXTI_1);
  __NVIC_DisableIRQ(Gyro_EXTI_2);
  __NVIC_DisableIRQ(Accl_EXTI_1);
  __NVIC_DisableIRQ(Accl_EXTI_2);

	srand(now_ms());
}


void baro_dma_isr(int precise)
{
	reset();

	if (precise)
	{
		baro_conf.osr_p = BARO_OSR_X8;
  	baro_conf.iir_coef = BARO_IIR_COEF_15;
	}

	assert(init_baro(&baro_conf) == HAL_OK);

	struct baro k = {0};

	__NVIC_EnableIRQ(Baro_EXTI);

	for (int q = 0; q < MAX_ISR_CYCLES; ++q)
	{
		if (dma_fetch_baro(&k))
		{
			compensate_baro(&k);
			log_measurement(SEDS_DT_BAROMETER_DATA, &k);
		}

		random_wait
	}
}


void imu_dma_isr(int lowpower)
{
	reset();

	if (lowpower)
	{
		accl_conf.mode = Normal_400Hz;
		gyro_conf.bw = Gyro_47Hz_ODR_400Hz;
	}

	assert(init_gyro(&gyro_conf) == HAL_OK);
	assert(init_accl(&accl_conf) == HAL_OK);

	struct measurement k = {0};

	__NVIC_EnableIRQ(Gyro_EXTI_1);
  __NVIC_EnableIRQ(Accl_EXTI_1);

	for (int q = 0; q < MAX_ISR_CYCLES; ++q)
	{
		if (dma_fetch_imu(&k.gyro, &k.d.accl))
		{
			compensate_accl(&k.d.accl);
			log_measurement(SEDS_DT_GYRO_DATA,  &k.gyro);
			log_measurement(SEDS_DT_ACCEL_DATA, &k.d.accl);
		}

		random_wait
	}
}


void combination_dma_isr(int precise, int lowpower)
{
	reset();

	if (precise)
	{
		baro_conf.osr_p = BARO_OSR_X8;
  	baro_conf.iir_coef = BARO_IIR_COEF_15;
	}

	if (lowpower)
	{
		accl_conf.mode = Normal_400Hz;
		gyro_conf.bw = Gyro_47Hz_ODR_400Hz;
	}

	assert(init_baro(&baro_conf) == HAL_OK);
	assert(init_gyro(&gyro_conf) == HAL_OK);
	assert(init_accl(&accl_conf) == HAL_OK);

	struct measurement k = {0};

	__NVIC_EnableIRQ(Baro_EXTI);
  __NVIC_EnableIRQ(Gyro_EXTI_1);
  __NVIC_EnableIRQ(Accl_EXTI_1);

	for (int q = 0; q < MAX_ISR_CYCLES; ++q)
	{
		fu8 st = dma_fetch(&k, 0);

		if (st == RX_DONE)
		{
			compensate_all(&k);
			log_measurement(SEDS_DT_GYRO_DATA,      &k.baro);
			log_measurement(SEDS_DT_ACCEL_DATA,     &k.gyro);
			log_measurement(SEDS_DT_BAROMETER_DATA, &k.d.accl);
		}
		else
		{
			log_err("DMA: (testing) readiness: %u", st);
		}

		random_wait
	}
}


void baro_dma_sync(int precise)
{
	if (precise)
	{
		baro_conf.osr_p = BARO_OSR_X8;
  	baro_conf.iir_coef = BARO_IIR_COEF_15;
	}

	assert(init_baro(&baro_conf) == HAL_OK);

	fu32 dt;
	struct baro k = {0};

	for (int q = 0; q < MAX_SYNC_CYCLES; ++q)
	{
		if (dma_attempt_transfer(Sensor_Baro) != DMA_Ok)
		{
			random_wait
			continue;
		}

		dt = wait_on_sync_transfer(Sensor_Baro);
		log_err("Baro: wait delay: %u", dt);

		if (dma_fetch_baro_sync(&k))
		{
			compensate_baro(&k);
			log_measurement(SEDS_DT_BAROMETER_DATA, &k);
		}

		random_wait
	}
}


void imu_dma_sync(int lowpower)
{
	if (lowpower)
	{
		accl_conf.mode = Normal_400Hz;
		gyro_conf.bw = Gyro_47Hz_ODR_400Hz;
	}

	assert(init_gyro(&gyro_conf) == HAL_OK);
	assert(init_accl(&accl_conf) == HAL_OK);

	fu32 dt;
	struct measurement k = {0};

	for (int q = 0; q < MAX_SYNC_CYCLES; ++q)
	{
		if (dma_attempt_transfer(Sensor_Gyro) != DMA_Ok)
		{
			random_wait
			continue;
		}

		dt = wait_on_sync_transfer(Sensor_Gyro);
		log_err("Gyro: wait delay: %u", dt);

		if (dma_attempt_transfer(Sensor_Accl) != DMA_Ok)
		{
			random_wait
			continue;
		}

		dt = wait_on_sync_transfer(Sensor_Accl);
		log_err("Accl: wait delay: %u", dt);

		if (dma_fetch_imu_sync(&k.gyro, &k.d.accl))
		{
			compensate_accl(&k.d.accl);
			log_measurement(SEDS_DT_GYRO_DATA,  &k.baro);
			log_measurement(SEDS_DT_ACCEL_DATA, &k.gyro);
		}

		random_wait
	}
}


void noreturn dma_sensor_test(void)
{
	baro_dma_isr(0);
	baro_dma_isr(1);
	imu_dma_isr(0);
	imu_dma_isr(1);
	combination_dma_isr(0, 0);
	combination_dma_isr(1, 1);

	reset();

	baro_dma_sync(0);
	baro_dma_sync(1);
	imu_dma_sync(0);
	imu_dma_sync(1);

	_Exit(0);
}