#include <stdint.h>
#include <time.h>

#include "platform.h"

static struct timespec success_delay = {0, 100000000};
static struct timespec failure_delay = {0, 50000000};

static uint_fast8_t init_fail_baro = 0;
static uint_fast8_t init_fail_gyro = 0;
static uint_fast8_t init_fail_accel = 0;

/*
 * For the testing purposes, disabling interrputs
 * completely disables emulation drivers.
 */
uint_fast8_t irq = 1;

static uint_fast8_t baro_ready = 0;
static uint_fast8_t gyro_ready = 0 ;
static uint_fast8_t accel_ready = 0;

HAL_StatusTypeDef emu_baro_init()
{
  if (!init_fail_baro) {
    nanosleep(&success_delay, NULL);
    baro_ready = 1;
    return HAL_OK;
  } else {
    nanosleep(&failure_delay, NULL);
    return HAL_ERROR;
  }
}

HAL_StatusTypeDef emu_gyro_init()
{
  if (!init_fail_gyro) {
    nanosleep(&success_delay, NULL);
    gyro_ready = 1;
    return HAL_OK;
  } else {
    nanosleep(&failure_delay, NULL);
    return HAL_ERROR;
  }
}

HAL_StatusTypeDef emu_accel_init()
{
  if (!init_fail_accel) {
    nanosleep(&success_delay, NULL);
    accel_ready = 1;
    return HAL_OK;
  } else {
    nanosleep(&failure_delay, NULL);
    return HAL_ERROR;
  }
}

void break_baro() { init_fail_baro = 1; }
void break_gyro() { init_fail_gyro = 1; }
void break_accel() { init_fail_accel = 1; }

void unbreak_baro() { init_fail_baro = 0; }
void unbreak_gyro() { init_fail_gyro = 0; }
void unbreak_accel() { init_fail_accel = 0; }

void emu_disable_irq() { irq = 0; }
void emu_enable_irq() { irq = 1; }