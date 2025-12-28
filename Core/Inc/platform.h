// When resolving merge conflict, accept both versions
// of this file, except for parts marked as "duplicate",
// and remove this and other pre-merge comments. Thanks

// pre-merge comment: duplicate imports
#include <stdint.h>
#include "barometer.h"
#include "gyro.h"
#include "accel.h"
#include "stm32h5xx_hal.h"
#include "stm32h5xx_hal_spi.h"
#include "stm32h5xx_hal_def.h"
#include "stm32h5xx_hal_dcache.h"


#include "core_cm33.h"

// pre-merge comment: duplicate (Deployment)
extern SPI_HandleTypeDef hspi1;
extern DCACHE_HandleTypeDef hdcache1;

// pre-merge comment: the following is to be put under
// /* HAAL */ /* Data cache calls */ section
// other HAL-related defs (below, above) should go
// anywhere under the /* HAAL */ section

#define invalidate_dcache_addr_int(buf, size)         \
  (HAL_DCACHE_InvalidateByAddr_IT((&hdcache1),        \
                                  (uint32_t *)(buf),  \
                                  (size)))

#define clean_dcache_addr_int(buf, size)              \
  (HAL_DCACHE_CleanByAddr_IT((&hdcache1),             \
                             (uint32_t *)(buf),       \
                             (size)))

/* DMA transmit-receive */

#define dma_spi_txrx(txbuf, rxbuf, size)              \
  (HAL_SPI_TransmitReceive_DMA((&hspi1), (txbuf),     \
                               (rxbuf), (size)))

// pre-merge comment: the following is to be put under
// /* HAAL */ /* Sensor drivers and data collection */ section

#define BARO_TX_BYTE  ((uint8_t)(BARO_DATA_0 | BMP390_SPI_READ_BIT))
#define GYRO_TX_BYTE  ((uint8_t)(GYRO_CMD_READ(GYRO_RATE_X_LSB)))
#define ACCEL_TX_BYTE ((uint8_t)(ACCEL_CMD_READ(ACCEL_X_LSB)))

/* Peripheral sensor EXT interrupt pins */

#define ACCEL_INT_PIN_1 GPIO_PIN_4
#define ACCEL_INT_PIN_2 GPIO_PIN_5
#define GYRO_INT_PIN_1  GPIO_PIN_0
#define GYRO_INT_PIN_2  GPIO_PIN_1
#define BARO_INT_PIN    GPIO_PIN_7

/* Driver-specific data conversions */

#define U24(b0, b1, b2)                                             \
  (((uint32_t)(b2) << 16) | ((uint32_t)(b1) << 8) | (uint32_t)(b0))

#define I16(b0, b1)                                                 \
  ((int16_t)(((uint16_t)(b1) << 8) | (uint16_t)(b0)))

#define F16(b0, b1) ((float)I16(b0, b1))