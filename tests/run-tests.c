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
  ukf_thread = 1;

  if (signal(SIGINT, handler) == SIG_ERR) {
    emu_print_err("Could not register UNIX signal.");
  } else {
    create_ukf_thread();
    create_deployment_thread();
  }
}

static void test_normal_flight()
{
  ULONG samp;

  init_testing();

  // TODO: after ukf is finished, supply it with raw data.
}

int main()
{
  test_normal_flight();
}