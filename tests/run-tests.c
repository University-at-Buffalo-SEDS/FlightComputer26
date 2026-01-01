#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/signal.h>

#include "deployment.h"
#include "platform.h"

void handler(int sig)
{
  command_e cmd;
  /* SIGINT  = ABORT
   * SIGQUIT = FIRE_PYRO
   * SIGTSTP = FIRE_REEF */
  switch (sig) {
    case SIGINT: cmd = ABORT; break;
    case SIGQUIT: cmd = FIRE_PYRO; break;
    case SIGTSTP: cmd = FIRE_REEF; break;
    default: return;
  }

  deployment_send_command(cmd);

  if (cmd == FIRE_REEF) _Exit(0);
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