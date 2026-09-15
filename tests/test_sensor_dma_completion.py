"""Execute production DMA completion logic with early and delayed IRQs."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class SensorDmaCompletion(unittest.TestCase):
    def test_completion_is_retained_before_wait_and_errors_are_bounded(self):
        source = (ROOT / "Core/Src/dma.c").read_text()
        callbacks = source[source.index("void HAL_SPI_TxRxCpltCallback"):
                           source.index("void HAL_GPIO_EXTI_Rising_Callback")]
        transfer = source[source.index("static inline void start_dma_transfer"):
                          source.index("/* Task */")]
        harness = r'''
#include <assert.h>
#include <stdint.h>
typedef unsigned UINT;
typedef int HAL_StatusTypeDef;
typedef int SPI_HandleTypeDef;
enum { TX_NO_WAIT=0, TX_SUCCESS=0, HAL_OK=0, HAL_BUSY=2,
       DMA_TIMEOUT_MS=20, SENSOR_BUF_SIZE=8, Rlx=0 };
static struct { int next, valid; } select;
static struct { unsigned drdy[3]; } gpio = {{1,2,4}};
static struct { unsigned drdy; } flags;
static int dma_completion, hspi1, mode, delivered, aborted, slept;
static uint8_t tx[3][8], dmarx[8];
#define SENSOR_PROBE(counter) ((void)0)
#define gpio_cs_high(sensor) ((void)(sensor))
#define gpio_cs_low(sensor) ((void)(sensor))
static void tx_semaphore_put(int *s) { ++*s; }
static void tx_thread_sleep(int ticks) { slept += ticks; }
static void HAL_SPI_Abort(int *spi) { (void)spi; aborted++; }
static void fetch_and(unsigned *p, unsigned mask, int order)
{ (void)order; *p &= mask; }
static void propagate_rx(void) { delivered++; }
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *);
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *);
static UINT tx_semaphore_get(int *s, UINT timeout) {
  if(timeout && mode == 1) HAL_SPI_TxRxCpltCallback(&hspi1);
  if(*s) { --*s; return TX_SUCCESS; }
  return 1;
}
static int dma_spi_txrx(const uint8_t *a, uint8_t *b, int n) {
  (void)a; (void)b; (void)n;
  if(mode == 0) HAL_SPI_TxRxCpltCallback(&hspi1);
  if(mode == 2) HAL_SPI_ErrorCallback(&hspi1);
  return mode == 4 ? HAL_BUSY : HAL_OK;
}
'''
        harness += callbacks + transfer + r'''
int main(void) {
  for(mode=0; mode<5; mode++) {
    dma_completion=1; /* stale completion must not satisfy this transfer */
    delivered=aborted=slept=0; select.valid=1;
    start_dma_transfer();
    assert(delivered == (mode < 2));
    assert(aborted == (mode == 3));
    assert(slept == (mode == 4));
  }
  return 0;
}
'''
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            (path / "test.c").write_text(harness)
            subprocess.run([os.environ.get("CC", "cc"), "-std=c11", "-Wall",
                            "-Wextra", "-Werror", "-Wno-unused-parameter",
                            str(path / "test.c"), "-o", str(path / "test")], check=True)
            subprocess.run([str(path / "test")], check=True)
