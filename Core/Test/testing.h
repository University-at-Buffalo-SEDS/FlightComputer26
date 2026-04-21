/*
 * Flight Computer 26 testing definitions and API.
 */

#ifndef TESTING_H
#define TESTING_H

#include "platform.h"
#include "fctypes.h"


#define SENSOR_SYNC_STEPS 128
#define MAX_FETCH_INTERVAL_MS 50
#define MAX_CRIT_MSG_BYTES 256

#define MAX_ISR_CYCLES 16
#define MAX_SYNC_CYCLES 16

#define random_wait HAL_Delay(rand() % MAX_FETCH_INTERVAL_MS);
#define noreturn __attribute__ ((noreturn))


#ifdef TEST_SENSORS

void test_baro_sync(SPI_HandleTypeDef *hspi, int precise);
void test_gyro_sync(SPI_HandleTypeDef *hspi, int lowpower);
void test_accl_sync(SPI_HandleTypeDef *hspi, int lowpower);
void noreturn test_sensors_sync(void);

#endif /* TEST_SENSORS */


#ifdef MATH_FN_DEBUG

void drop_math_class(const char *msg, const char *file, int line, fu32 code);
void mxcheck(const matrix *m, uint16_t rows, uint16_t cols, const char *name);
void vecheck(const matrix *m, float *v, const char *name);
void offcheck(const matrix *c, const matrix *p, const char *name);

#define ok(pred, msg)																\
	do { 																							\
		if (!(pred))																		\
		{																								\
			drop_math_class(msg, __FILE__, __LINE__, 0);	\
		}																								\
	} while (0)

#define cmsis_math_debug(code)                      \
  do {                                              \
    if (code != ARM_MATH_SUCCESS)                   \
    {                                               \
			drop_math_class("matrix error",         			\
                      __FILE__, __LINE__, code);		\
    }                                               \
  } while (0)

#define math_call(fn, ...)                          \
  do {                                              \
    math_status st = (fn)(__VA_ARGS__);             \
    cmsis_math_debug(st);                           \
  } while (0)

#define mxok mxcheck
#define veok vecheck
#define offok offcheck

#else

#define math_call(fn, ...) (fn)(__VA_ARGS__)

#define mxok
#define veok
#define offok

#endif /* MATH_FN_DEBUG */


#endif /* TESTING_H */