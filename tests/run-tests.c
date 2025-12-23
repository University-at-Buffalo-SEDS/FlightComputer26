#include <stdio.h>
#include <stdlib.h>
#include <signal.h>

#include "deployment.h"
#include "platform.h"

void handler(int sig)
{
  emu_print_event("Host issued abortion signal via SIGINT!");
  force_abort_deployment();

  for (int i = 0; i < 10; ++i) {
    emu_print_event("Still spinning main thread!");
    proper_sleep(1, 0);
  }

  emu_print_event("This is just emulation, bye!");
  _Exit(0);
}

static void init_testing()
{
  srand(time(0));
  deployment_thread = 0;
  if (signal(SIGINT, handler) == SIG_ERR) {
    emu_print_err("Could not register UNIX signal.");
  } else {
    create_deployment_thread();
  }
}

static void test_normal_flight()
{
  ULONG samp;

  /* 
   * This should result in creation of the deployment thread.
   * Producer-consumer with ring buffer and a single atomic is a
   * simple lock-free that should work with 2 pthreads (subject to testing)
   */
  init_testing();

  /* 1. Idle: noisy 0s */
  samp = (ULONG)(T_IDLE * SAMPLE_HZ);
  for (ULONG i = 0; i < samp; ++i) {
    produce_normal(0.0f, 0.0f, 0.0f, 1, CASUAL_SIGMA);
  }

  while (1)
    emu_print_event("Virtually landed!");
}

int main()
{
  test_normal_flight();
}