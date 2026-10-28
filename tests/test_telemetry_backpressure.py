from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class TelemetryBackpressureTests(unittest.TestCase):
    def test_retained_busy_work_is_observed_without_masking_real_handler_errors(self):
        source = (ROOT / "Core/Src/telemetry.c").read_text()
        body = source.split("static void telemetry_record_queue_result", 1)[1].split(
            "static const SedsLocalEndpointDesc", 1)[0]
        declarations = r'''
#include <stdint.h>
#include <assert.h>
typedef int SedsResult;
#define SEDS_OK 0
#define SEDS_IO -14
#define SEDS_HANDLER_ERROR -16
#define SEDS_BAD_ARG -2
#define HAL_FDCAN_ERROR_FIFO_FULL 0x200U
volatile uint32_t g_fdcan_tx_fail_count, g_fdcan_tx_busy_count, g_fdcan_last_error;
uint32_t g_queue_observed_tx_fail_count, g_queue_observed_tx_busy_count;
volatile uint32_t g_telemetry_link_backpressure, g_telemetry_queue_errors;
volatile int32_t g_telemetry_last_queue_result;
'''
        checks = r'''
int main(void) {
    /* SDK queue service may retain a busy side and return SEDS_OK. */
    g_fdcan_tx_busy_count = 1;
    telemetry_record_queue_result(SEDS_OK);
    assert(g_telemetry_link_backpressure == 1 && g_telemetry_queue_errors == 0);
    telemetry_record_queue_result(SEDS_OK);
    assert(g_telemetry_link_backpressure == 1);
    g_fdcan_tx_busy_count = 2;
    telemetry_record_queue_result(SEDS_HANDLER_ERROR);
    assert(g_telemetry_link_backpressure == 2 && g_telemetry_queue_errors == 0);
    /* The same error without new physical backpressure is still fatal. */
    telemetry_record_queue_result(SEDS_HANDLER_ERROR);
    assert(g_telemetry_queue_errors == 1);
    g_fdcan_tx_fail_count = 1;
    g_fdcan_last_error = HAL_FDCAN_ERROR_FIFO_FULL;
    telemetry_record_queue_result(SEDS_HANDLER_ERROR);
    assert(g_telemetry_link_backpressure == 3 && g_telemetry_queue_errors == 1);
    telemetry_record_queue_result(SEDS_IO);
    assert(g_telemetry_link_backpressure == 4 && g_telemetry_queue_errors == 1);
    /* An unrelated API error must not be hidden by coincident congestion. */
    g_fdcan_tx_busy_count = 3;
    telemetry_record_queue_result(SEDS_BAD_ARG);
    assert(g_telemetry_link_backpressure == 5 && g_telemetry_queue_errors == 2);
    assert(g_telemetry_last_queue_result == SEDS_BAD_ARG);
    g_queue_observed_tx_busy_count = UINT32_MAX;
    g_fdcan_tx_busy_count = 0;
    telemetry_record_queue_result(SEDS_OK);
    assert(g_telemetry_link_backpressure == 6);
    return 0;
}
'''
        with tempfile.TemporaryDirectory() as directory:
            executable = Path(directory) / "backpressure"
            subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                            "-fsanitize=address,undefined", "-x", "c", "-", "-o", str(executable)],
                           input=declarations + "static void telemetry_record_queue_result" + body + checks,
                           text=True, check=True)
            subprocess.run([str(executable)], check=True, timeout=5)
