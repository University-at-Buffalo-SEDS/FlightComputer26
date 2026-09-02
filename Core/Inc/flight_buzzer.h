#ifndef FLIGHT_BUZZER_H
#define FLIGHT_BUZZER_H

#include "sedsnet.h"
#include "sedsnet_config.h"

extern volatile uint32_t g_flight_buzzer_enabled;
extern volatile uint32_t g_flight_buzzer_updates;
extern volatile uint32_t g_flight_buzzer_persist_restores;
extern volatile uint32_t g_flight_buzzer_persist_writes;
extern volatile uint32_t g_flight_buzzer_persist_errors;

SedsResult flight_buzzer_init(SedsRouter *router);
SedsResult flight_buzzer_poll(SedsRouter *router);

#endif
