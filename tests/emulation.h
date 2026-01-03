/*
 * Unified header and configuration file
 * for emulating different things on FC
 */

#ifndef EMULATION_H
#define EMULATION_H

#include <stdint.h>


/* ------ Emulation general definitions ------ */

#define EMU_TASKS   2u
#define EMU_SENSORS 3
#define YIELD_SLEEP 100

#define EMU_INIT 1u
#define EMU_FUNC (1u << 8)
#define EMU_PRED (1u << 16)

#define SUCCESS_TICKS 100
#define FAILURE_TICKS 50

#define SAMPLE_HZ 50
#define SAMPLE_TK (1e7 / SAMPLE_HZ)

#define MAX_DEVIATION 1.5f
#define CASUAL_SIGMA  0.375f


/* ------ ThreadX compatibility typedefs ------ */

typedef unsigned int UINT;
typedef unsigned long ULONG;
typedef UINT TX_THREAD;

typedef void (*task_t)(ULONG);
typedef struct { task_t fn; ULONG arg; } callee_t;


/* ------ HAL compatibility typedefs ------ */

typedef enum { SEDS_DT_MESSAGE_DATA } emu_data_e;
typedef enum { HAL_LOCKED, HAL_UNLOCKED } emu_spi_lock_e;

typedef enum { HAL_OK, HAL_ERROR } HAL_StatusTypeDef;

typedef struct { 
  emu_spi_lock_e Lock;
  void *pRxBufPtr;
} SPI_HandleTypeDef;


/* ------ Emulation core API ------ */

/// Initialize emulation.
void emu_init();

/// Millisecond conversion of host libc RTC wrapper.
uint32_t emu_time_ms();

/// Nanosleep wrapper.
/// Substituies tx_thread_sleep()
int emu_sleep(UINT ticks);

/// Substitutes tx_thread_resume()
void emu_yield(TX_THREAD *thread);

/// Substitutes log_telemetry_(a)sync(SEDS_DT_MESSAGE_DATA, ...)
void emu_print_buf(emu_data_e type, void *buf,
                   size_t size, size_t element_size);

/// Substitutes log_error_(a)sync(...)
void emu_print_err(const char *fmt, ...);

/// Substitutes several embedded API functions.
/// Visible indication of physical events
/// such as parachute deployment or expansion.
void emu_print_event(void *buf);

/// Substitutes tx_create_thread()
UINT emu_create_thread(TX_THREAD *thread, char *name, task_t entry, ULONG input,
                       ULONG *stack, UINT stack_size, UINT priority,
                       UINT preemption, UINT time_slice, UINT autostart);


/* ------ Emulation drivers API ------ */

/// Tries to initialize sensor according to its breakage flag.
/// Substitutes sensor initialization functions.
HAL_StatusTypeDef emu_init_sensor(unsigned int k);

/// Sets a sensor to report functional but produce invalid measurements.
HAL_StatusTypedef emu_init_byzantine();

/// Whether a sensor reports itself as functional.
unsigned int sensor_is_ready(unsigned int k);

/// Set a sensor to produce valid (> 0) or invalid (0) measurements.
void sensor_prediction(unsigned int sensor, unsigned int valid);

/// Set a sensor to succeed (> 0) or fail (0) next init.
void sensor_next_init(unsigned int sensor, unsigned int succeed);

/// Sets the IRQ flag
void set_irq(unsigned int k);

/// Returns the value of the IRQ flag
unsigned int irq_enabled();


/* ------ Emulation sensors API ------ */

/// Produce one normal sample (deprecated)
void produce_normal(float h1, float v1, float a, ULONG samp, float sigma);


#endif // EMULATION_H