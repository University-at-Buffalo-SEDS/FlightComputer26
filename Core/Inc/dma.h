/*
 * Direct Memory Access configuration and API.
 */

#ifndef DMA_H
#define DMA_H

#include "platform.h"


/* ------ Containers ------ */

enum sensor {
  Sensor_Baro = 0,
  Sensor_Gyro = 1,
  Sensor_Accl = 2,

  Sensors
};

struct serial baro { float alt, p, t; };

struct serial measurement {
  struct coords gyro;
  union {
    struct coords accl;
    struct coords gps;
  } d;
  struct baro baro;
};

struct gpio_lookup {
  const GPIO_TypeDef *port[Sensors];
  const uint16_t pin[Sensors];
  const uint16_t drdy[Sensors];
};

struct selector {
  volatile fu8 next;
  volatile fu8 valid;
};

struct dma_flags {
  atomic_uint_fast16_t drdy;
  atomic_uint_fast16_t relv;
};


/* ------ DMA / sensor defs ------  */

#define SENSOR_BUF_SIZE 8

#define DMA_TIMEOUT_MS 50

#define BARO_MASK BARO_EXTI_Pin
#define GYRO_MASK (GYRO_EXTI_1_Pin | GYRO_EXTI_2_Pin)
#define ACCL_MASK (ACCL_EXTI_1_Pin | ACCL_EXTI_2_Pin)


/* ------ Producer / consumer benchmark timer ------ */

#ifdef DMA_BENCHMARK

#define dma_bench_log(dev)                                    \
  do {                                                        \
    fu32 dt = now_ms() - load(&rxts[dev], Acq);               \
    log_err("DMA: %u: consumed in %u ms", dev, dt);           \
  } while (0)

#define dma_bench_refresh(dev)                                \
  store(&rxts[dev], now_ms(), Rel)

#else

#define dma_bench_log(dev)     (void)0
#define dma_bench_refresh(dev) (void)0  

#endif // DMA_BENCHMARK


/* ------ Public API ------ */

/*
 * Tries to fetches barometer data from shared buffer.
 * Returns true on success and false if new data is unavailable.
 */
bool fetch_baro(struct baro *buf);

/*
 * Tries to fetches gyroscope data from shared buffer.
 * Returns true on success and false if new data is unavailable.
 */
bool fetch_gyro(struct coords *buf);

/*
 * Tries to fetches acceletometer data from shared buffer.
 * Returns true on success and false if new data is unavailable.
 */
bool fetch_accl(struct coords *buf);


#endif // DMA_H