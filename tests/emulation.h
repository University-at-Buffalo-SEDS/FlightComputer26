/*
 * Unified header and configuration file
 * for emulating different things on FC
 */

typedef void *(*task_t)(void *);

#define EMU_TASKS           4u
#define MIN_TICKS_TO_YIELD  50u
#define FAKE_THREAD_INPUT   0u
#define FAKE_YIELD_CYCLES   4u

#define TX_TO_SEC(ticks) (((float)ticks) / 100.0f)

#define SAMPLE_HZ 50
#define SAMPLE_NS (1e9 / SAMPLE_HZ)

#define MAX_DEVIATION 2.0f

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

/* Emulation Helpers */

/* Nanosleep wrapper */
void proper_sleep(time_t sec, long nsec);

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