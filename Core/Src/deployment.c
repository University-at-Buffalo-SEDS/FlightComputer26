/*
 * Logic related to parachute deployment.
 */

#include <stdint.h>
#include <stdbool.h>

#include "FC-Threads.h"
#include "tx_api.h"
#include "tx_port.h"

#include <sedsprintf.h>
#include "telemetry.h"
#include "deployment.h"

/* Thread instance and variables */
TX_THREAD deployment_thread;
ULONG deployment_thread_stack[DEPLOYMENT_THREAD_STACK_SIZE / sizeof(ULONG)];

/* Log helper */
static inline void send_log(SedsDataType type, const char *msg) {
    char message[DEPLOYMENT_DEFAULT_LOG_SIZE];
    memcpy(message, msg, sizeof(message));
    log_telemetry_synchronous(type, message, sizeof(message), sizeof(char));
}

/* Performs calculations and necessary checks to infer rocket state.
 * Responsible for invoking parachute deployment and triggering abortion. */
static inline bool infer_rocket_state() {
    return false; // TODO
}

/* Entry point of the deployment thread; does not return under normal conditions */
void deployment_thread_entry(ULONG input) {
    (void)input;

    send_log(SEDS_DT_MESSAGE_DATA, "Deployment: starting thread");

    uint16_t retries = 0;
    for (;;) {
        if (!infer_rocket_state()) {
            send_log(SEDS_DT_GENERIC_ERROR, "Deployment: state inference failed");
            ++retries;
            
            if (retries >= DEPLOYMENT_THREAD_MAX_RETRIES) {
                send_log(SEDS_DT_GENERIC_ERROR, "FATAL: aborting deployment");
                return;
            }
        } else {
            retries = 0; // Reset counter (error mitigated)
        }
        tx_thread_sleep(DEPLOYMENT_THREAD_SLEEP);
    }
}

/* Creates a parachute deployment thread with defined parameters */
void create_deployment_thread(void) {
    UINT st = tx_thread_create(&deployment_thread,
                               "Deployment Thread",
                               deployment_thread_entry,
                               DEPLOYMENT_THREAD_INPUT,
                               deployment_thread_stack,
                               DEPLOYMENT_THREAD_STACK_SIZE,
                               DEPLOYMENT_THREAD_PRIORITY,
                               DEPLOYMENT_THREAD_PRIORITY, // No preemption
                               TX_NO_TIME_SLICE,
                               TX_AUTO_START);

    if (st != TX_SUCCESS) {
        die("Failed to create deployment thread: %u", (unsigned)st);
    }
}