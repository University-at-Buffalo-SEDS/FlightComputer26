/*
 * Unified header and configuration file
 * for emulating different things on FC
 */

#ifndef EMULATION_H
#define EMULATION_H

#include <stdint.h>
#include <time.h>


/* ------ Emulation general definitions ------ */

#define EMU_TASKS   2u
#define EMU_SENSORS 3
#define YIELD_SLEEP 100

#define TX_TO_SEC(ticks) (((float)ticks) / 100.0f)
#define NS_IN_SEC 1000000000L

#define SAMPLE_HZ 50
#define SAMPLE_NS (1e9 / SAMPLE_HZ)

#define MAX_DEVIATION 2.0f
#define CASUAL_SIGMA  0.5f

#define T_IDLE      2.0f
#define T_LAUNCH    2.0f
#define T_ASCENT    10.0f
#define T_BURNOUT   1.0f
#define T_APOGEE    6.0f
#define T_DESCENT   10.0f
#define T_REEF      2.0f
#define T_LANDED    2.0f


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

/// Millisecond conversion of host libc RTC wrapper.
uint32_t emu_time_ms();

/// Nanosleep wrapper.
int proper_sleep(time_t sec, long nsec);

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

/// Sets irq flag to 1
void emu_enable_irq();

/// Sets irq flag to 0
void emu_disable_irq();

/// Returns the value of irq flag
uint_fast8_t irq_enabled();

/// Set a sensor to fail next init.
void break_sensor(int k);

/// Set a sensor to succeed next init.
void recover_sensor(int k);

/// Tries to initialize sensor according to its breakage flag.
/// Substitutes sensor initialization functions.
HAL_StatusTypeDef emu_init_sensor(int k);


/* ------ Emulation sensors API ------ */

/// Produce one normal sample (deprecated)
void produce_normal(float h1, float v1, float a, ULONG samp, float sigma);


#endif // EMULATION_H