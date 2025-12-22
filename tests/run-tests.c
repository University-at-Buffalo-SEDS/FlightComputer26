#include "emulation.h"

static void test_normal_flight()
{
  ULONG samp;

  /* 
   * This should result in creation of the deployment thread.
   * Producer-consumer with ring buffer and a single atomic is a
   * simple lock-free that should work with 2 pthreads (subject to testing)
   */
  filter_init_testing();

  /* 1. Idle: noisy 0s */
  samp = (ULONG)(T_IDLE * SAMPLE_HZ);
  for (ULONG i = 0; i < samp; ++i) {
    produce_normal(0.0f, 0.0f, 0.0f, 1, CASUAL_SIGMA);
  }
}

int main()
{
  test_normal_flight();
}