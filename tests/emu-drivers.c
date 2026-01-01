/*
 * Minimal runtime configuration interface
 * used to instruct emulated sensors of fault injections.
 */

#include <stdint.h>
#include <stdatomic.h>
#include <time.h>

#include "platform.h"

/// Default "init" delays.
static const struct timespec success_delay = {0, 100000000};
static const struct timespec failure_delay = {0, 50000000};

/// For each sensor k in EMU_SENSORS:
/// [k][0] - fail next init; [k][1] - sensor is ready.
static uint_fast8_t flags[EMU_SENSORS][2] = {0u};

/// Accessed inter-threads.
static atomic_uint_fast8_t irq = 1u;

/// Tries to initialize sensor according to its breakage flag.
/// Substitutes sensor initialization functions.
HAL_StatusTypeDef emu_init_sensor(int k)
{
  if (!flags[k][0]) {
    nanosleep(&success_delay, NULL);
    flags[k][1] = 1;
    return HAL_OK;
  } else {
    nanosleep(&failure_delay, NULL);
    return HAL_ERROR;
  }
}

/// Whether a sensor successfully initialized
uint_fast8_t sensor_is_ready(int k) { return flags[k][1]; }

/// Set a sensor to fail next init.
void break_sensor(int k)    { flags[k][0] = 1; }

/// Set a sensor to succeed next init.
void recover_sensor(int k)  { flags[k][0] = 0; }

/// Sets irq flag to 0
void emu_disable_irq()  { atomic_store_explicit(&irq, 0, memory_order_release); }

/// Sets irq flag to 1
void emu_enable_irq()   { atomic_store_explicit(&irq, 1, memory_order_release); }

/// Returns the value of irq flag
uint_fast8_t irq_enabled() { return atomic_load_explicit(&irq, memory_order_acquire); }