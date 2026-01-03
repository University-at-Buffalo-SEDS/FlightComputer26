/*
 * Minimal runtime configuration interface
 * used to instruct emulated sensors of fault injections.
 */

#include <stdlib.h>
#include <stdint.h>
#include <stdatomic.h>

#include "platform.h"

/// For each sensor k in EMU_SENSORS, the mask follows as:
/// Bit 1:  sensor will fail next init;
/// Bit 8:  sensor will report its state as normal;
/// Bit 16: sensor will produce incorrent measurements.
static unsigned int flags[EMU_SENSORS] = {0};

/// Accessed inter-threads.
static atomic_uint_t irq = 1;

/// Tries to initialize sensor according to its breakage flag.
/// Substitutes sensor initialization functions.
HAL_StatusTypeDef emu_init_sensor(unsigned int k)
{
  unsigned int i = k % EMU_SENSORS;
  if (flags[i] & EMU_INIT) {
    emu_sleep(SUCCESS_TICKS);
    flags[i] |= EMU_FUNC;
    return HAL_OK;
  } else {
    emu_sleep(FAILURE_TICKS);
    return HAL_ERROR;
  }
}

/// Sets a sensor to report functional but produce invalid measurements.
HAL_StatusTypedef emu_init_byzantine()
{
  int k = rand() % EMU_SENSORS;
  emu_sleep(SUCCESS_TICKS);
  flags[k] = EMU_FUNC | EMU_PRED;
  return HAL_OK;
}

/// Whether a sensor reports itself as functional.
unsigned int sensor_is_ready(unsigned int k)
{
  return flags[k % EMU_SENSORS] & EMU_FUNC;
}

/// Set a sensor to produce valid (> 0) or invalid (0) measurements.
void sensor_prediction(unsigned int sensor, unsigned int valid)
{
  if (valid)
    flags[sensor % EMU_SENSORS] &= ~EMU_PRED;
  else
    flags[sensor % EMU_SENSORS] |= EMU_PRED;
}

/// Set a sensor to succeed (> 0) or fail (0) next init.
void sensor_next_init(unsigned int sensor, unsigned int succeed)
{
  if (succeed)
    flags[sensor % EMU_SENSORS] &= ~EMU_INIT;
  else
    flags[sensor % EMU_SENSORS] |= EMU_INIT;
}

/// Sets the IRQ flag
void set_irq(unsigned int k)
{
  atomic_store_explicit(&irq, k, memory_order_release);
}

/// Returns the value of the IRQ flag
unsigned int irq_enabled()
{
  return atomic_load_explicit(&irq, memory_order_acquire);
}