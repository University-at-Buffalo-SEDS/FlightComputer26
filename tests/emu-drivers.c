#include "platform.h"
#include <stdint.h>

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

PL_HAL_Handle emu_baro_init()
{
  if (!init_fail_baro) {
    baro_ready = 1;
    return HAL_OK;
  } else {
    return HAL_ERROR;
  }
}

PL_HAL_Handle emu_gyro_init()
{
  if (!init_fail_gyro) {
    gyro_ready = 1;
    return HAL_OK;
  } else {
    return HAL_ERROR;
  }
}

PL_HAL_Handle emu_accel_init()
{
  if (!init_fail_accel) {
    accel_ready = 1;
    return HAL_OK;
  } else {
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