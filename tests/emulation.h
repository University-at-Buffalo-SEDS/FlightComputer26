/*
 * Unified header and configuration file
 * for emulating different things on FC
 */

#ifndef EMULATION_H
#define EMULATION_H

#include <time.h>

/* General ThreadX compatibility definitions */

#define UINT unsigned int
#define ULONG unsigned long
#define TX_THREAD UINT

typedef void (*task_t)(ULONG);
typedef struct {task_t fn; ULONG arg;} callee_t;

/* Emulation service constants and helpers */

#define EMU_TASKS           4u
#define MIN_TICKS_TO_YIELD  50u
#define FAKE_THREAD_INPUT   0u
#define FAKE_YIELD_CYCLES   4u

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

#define FILTER_RING_SIZE 32
#define FILTER_RING_MASK (FILTER_RING_SIZE - 1)

/* Poisoned minimal types for HAL and telemetry */

typedef enum { SEDS_DT_MESSAGE_DATA } emu_data_e;
typedef enum { HAL_LOCKED, HAL_UNLOCKED } emu_spi_lock_e;

typedef enum { HAL_OK, HAL_ERROR } HAL_StatusTypeDef;

typedef struct { 
  emu_spi_lock_e Lock;
  void *pRxBufPtr;
} SPI_HandleTypeDef;

/* Emulation Helpers */

/* Nanosleep wrapper */
void proper_sleep(time_t sec, long nsec);

void emu_enable_irq();
void emu_disable_irq();

/* Emulated drivers */

/*
 * Set whether initialization (!) functions
 * should fail next time. For recovery testing.
 */
void break_baro();
void break_gyro();
void break_accel();

void unbreak_baro();
void unbreak_gyro();
void unbreak_accel();

HAL_StatusTypeDef emu_baro_init();
HAL_StatusTypeDef emu_gyro_init();
HAL_StatusTypeDef emu_accel_init();

/* Emulation core */

void emu_yield(unsigned *thread);
void emu_sleep(unsigned ticks);
unsigned emu_create_thread(TX_THREAD *thread, char *name,
                          task_t entry, ULONG input,
                          unsigned long *stack, unsigned stack_size,
                          unsigned priority, unsigned preemption,
                          unsigned time_slice, unsigned autostart);
void emu_print_buf(emu_data_e type, void *buf,
                   size_t size, size_t element_size);
void emu_print_err(const char *fmt, ...);
void emu_print_event(void *buf);

/* Pseudo-filter */

void filter_init_testing();
void produce_normal(float h1, float v1, float a, ULONG samp, float sigma);


#endif // EMULATION_H