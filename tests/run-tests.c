/*
 * Testing entry point.
 * This file should utilize the functionality
 * found in emulation.h to create testing scenarios.
 */

#include <stdio.h>

#include "platform.h"
#include "deployment.h"

static inline void test_normal_flight(enum state until_state)
{
  // TODO
}

static inline void test_total_failure(enum state failing_state)
{
  test_normal_flight(failing_state);
  // TODO
}

static inline void test_with_warnings(int deviations)
{
  // TODO
}

/// CTest single entry point.
int main()
{
  /* User configuation */
  const int testing_scenario = 0;
  const int num_deviations = 10;
  const enum state failing_state = LAUNCH;
  const unsigned int ticks_idling = 100;

  emu_init();
  emu_sleep(ticks_idling);

  switch (testing_scenario) {
    case 0: test_normal_flight(LANDED); break;
    case 1: test_total_failure(failing_state); break;
    case 2: test_with_warnings(num_deviations); break;
    /* ... */
  }
}