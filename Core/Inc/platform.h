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

/* HAL and SPI handle aliases */

typedef HAL_StatusTypeDef PL_HAL_Handle;
typedef SPI_HandleTypeDef PL_SPI_Handle;

// pre-merge comment: the following is to be put under
// /* HAAL */ /* Data cache calls */ section
// other HAL-related defs (below, above) should go
// anywhere under the /* HAAL */ section

#define INVALIDATE_DCACHE_ADDR_INT(buf, size)         \
  (HAL_DCACHE_InvalidateByAddr_IT((&hdcache1),        \
                                  (uint32_t *)(buf),  \
                                  (size)))

#define CLEAN_DCACHE_ADDR_INT(buf, size)              \
  (HAL_DCACHE_CleanByAddr_IT((&hdcache1),             \
                             (uint32_t *)(buf),       \
                             (size)))

/* DMA transmit-receive */

#define DMA_SPI_TXRX(txbuf, rxbuf, size)              \
  (HAL_SPI_TransmitReceive_DMA((&hspi1), (txbuf),     \
                               (rxbuf), (size)))

// pre-merge comment: the following is to be put under
// /* HAAL */ /* Sensor drivers and data collection */ section

/* Buffer sizes and DMA transmit masks */

#define PL_BUF_SIZE_BARO  BMP390_BUF_SIZE
#define PL_BUF_SIZE_GYRO  GYRO_BUF_SIZE
#define PL_BUF_SIZE_ACCEL ACCEL_BUF_SIZE

#define BARO_TX_BYTE  ((uint8_t)(BARO_DATA_0 | BMP390_SPI_READ_BIT))
#define GYRO_TX_BYTE  ((uint8_t)(GYRO_CMD_READ(GYRO_RATE_X_LSB)))
#define ACCEL_TX_BYTE ((uint8_t)(ACCEL_CMD_READ(ACCEL_X_LSB)))

/* CS pin macros (only needed to move out driver logic) */

#define PL_CS_BARO_LOW()    BARO_CS_LOW()
#define PL_CS_BARO_HIGH()   BARO_CS_HIGH()
#define PL_CS_GYRO_LOW()    GYRO_CS_LOW()
#define PL_CS_GYRO_HIGH()   GYRO_CS_HIGH()
#define PL_CS_ACCEL_LOW()   ACCEL_CS_LOW()
#define PL_CS_ACCEL_HIGH()  ACCEL_CS_HIGH()

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

#define F16(b0, b1)                                                 \
  ((float)I16(b0, b1))

// pre-merge comment: duplicate (Deployment)
#define DISABLE_HAL_INTS()  __disable_irq()
#define ENABLE_HAL_INTS()   __enable_irq()  