#include "board_temperature.h"
#include "board_temperature_math.h"
#include "main.h"
#include <stdbool.h>
#include "stm32h5xx_ll_adc.h"
#include "stm32h5xx_ll_bus.h"
#define ADC_COMMON ADC12_COMMON
#define TEMP_CHANNEL LL_ADC_CHANNEL_TEMPSENSOR
static uint32_t last_attempt;
static uint32_t sample_ms;
static int16_t sample_centi;
static bool valid;
static bool attempted;
static bool due(uint32_t now)
{
  if (attempted && (uint32_t)(now - last_attempt) < 5000U) return false;
  attempted = true; last_attempt = now; return true;
}
static void publish(uint32_t raw, uint32_t ref, uint32_t now)
{
  int16_t c;
  bool ok = board_temperature_convert(raw, ref, *VREFINT_CAL_ADDR,
      *TEMPSENSOR_CAL1_ADDR, *TEMPSENSOR_CAL2_ADDR,
      TEMPSENSOR_CAL1_TEMP, TEMPSENSOR_CAL2_TEMP, &c);
  uint32_t irq = __get_PRIMASK(); __disable_irq();
  if (ok) { sample_centi = c; sample_ms = now; }
  valid = ok;
  __set_PRIMASK(irq);
}
uint32_t board_temperature_health_flags(void)
{
  uint32_t now = HAL_GetTick();
  uint32_t irq = __get_PRIMASK(); __disable_irq();
  uint32_t flags = valid ? board_temperature_pack(sample_centi, now - sample_ms) : 0U;
  __set_PRIMASK(irq);
  return flags;
}

static void delay_us(uint32_t us)
{
  volatile uint32_t n = (SystemCoreClock / 1000000U + 1U) * us;
  while (n--) __NOP();
}
static bool wait_bits(volatile uint32_t *reg, uint32_t mask, bool set)
{
  uint32_t budget = SystemCoreClock / 10000U + 1U;
  while (((*reg & mask) != 0U) != set) if (--budget == 0U) return false;
  return true;
}
static uint8_t init_state;
static bool initialize(void)
{
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC);
  /* VB shares ADC12 with pressure ADC2: never change its common clock. */
  LL_ADC_SetCommonClock(ADC_COMMON, LL_ADC_CLOCK_SYNC_PCLK_DIV4);
  LL_ADC_SetCommonPathInternalChAdd(ADC_COMMON, LL_ADC_PATH_INTERNAL_TEMPSENSOR | LL_ADC_PATH_INTERNAL_VREFINT);
  LL_ADC_DisableDeepPowerDown(ADC1);
  LL_ADC_EnableInternalRegulator(ADC1);
  delay_us(LL_ADC_DELAY_INTERNAL_REGUL_STAB_US);
  LL_ADC_SetResolution(ADC1, LL_ADC_RESOLUTION_12B);
  LL_ADC_REG_SetSequencerLength(ADC1, LL_ADC_REG_SEQ_SCAN_DISABLE);
  LL_ADC_SetChannelSamplingTime(ADC1, TEMP_CHANNEL, LL_ADC_SAMPLINGTIME_640CYCLES_5);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_VREFINT, LL_ADC_SAMPLINGTIME_640CYCLES_5);
  LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED);
  if (!wait_bits(&ADC1->CR, ADC_CR_ADCAL, false)) return false;
  delay_us(2U);
  LL_ADC_ClearFlag_ADRDY(ADC1);
  LL_ADC_Enable(ADC1);
  if (!wait_bits(&ADC1->ISR, ADC_ISR_ADRDY, true)) return false;
  delay_us(LL_ADC_DELAY_TEMPSENSOR_STAB_US);
  return true;
}
static bool read_channel(uint32_t channel, uint32_t *out)
{
  if ((ADC1->CR & (ADC_CR_ADSTART | ADC_CR_JADSTART)) != 0U) return false;
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, channel);
  LL_ADC_ClearFlag_EOC(ADC1);
  LL_ADC_REG_StartConversion(ADC1);
  if (!wait_bits(&ADC1->ISR, ADC_ISR_EOC, true)) return false;
  *out = LL_ADC_REG_ReadConversionData12(ADC1);
  return wait_bits(&ADC1->CR, ADC_CR_ADSTART, false);
}
void board_temperature_poll(void)
{
  uint32_t now = HAL_GetTick();
  if (!due(now)) return;
  uint32_t raw = 0U, ref = 0U;
  if (!init_state) init_state = initialize() ? 1U : 2U;
  if (init_state == 1U && read_channel(TEMP_CHANNEL, &raw) && read_channel(LL_ADC_CHANNEL_VREFINT, &ref)) publish(raw, ref, now);
  else publish(0U, 0U, now);
}
