"""ThreadX stack budget for the FC tasks.

Every task that publishes through SEDSNet runs the router's immediate send
path on its own stack. Rust router frames are large (several are 0.7-1.3 KiB)
and the immediate path measured ~7 KiB deep on the distribution task before
Launch. At Launch the distribution task leaves streaming mode and sends the
ignition valve command and the flight-mode message; with an 8 KiB stack this
tripped the ThreadX stack check (startup_fault stage 7) and the watchdog reset
the FC. The evaluation task publishes flight-state changes the same way, and
in descent the distribution task runs the evaluator itself.

These checks keep the router callers at or above the measured budget and make
sure the shared ThreadX byte pool still holds every task stack.
"""
import json
import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

# Measured in the targeted Launch simulation (FirmwareSimulator flight-sim
# image, RF + PB + FC + GS, profile flight through Landed). Keep at least this
# much stack on every task that can call the SEDSNet router.
ROUTER_CALLER_MIN_BYTES = 12 * 1024
# The distribution task peaked at 9.3 KiB (8.8 KiB at Launch) and can
# initialise the router itself (~13 KiB static worst case), so it keeps a larger budget.
DIST_MIN_BYTES = 16 * 1024
# ThreadX byte-pool bookkeeping: one two-pointer header per allocated block
# plus the pool's own leading and trailing blocks.
POOL_BLOCK_OVERHEAD = 8
POOL_FIXED_OVERHEAD = 16


def _eval_size(expression: str) -> int:
    expression = re.sub(r"(\d+)U\b", r"\1", expression)
    if not re.fullmatch(r"[\d\s()*+]+", expression):
        raise AssertionError(f"unexpected size expression: {expression!r}")
    return int(eval(expression, {"__builtins__": {}}))  # noqa: S307 - digits/operators only


def _define(text: str, name: str) -> int:
    match = re.search(rf"(?m)^#define\s+{name}\s+(.+?)\s*$", text)
    if match is None:
        raise AssertionError(f"{name} is not defined")
    return _eval_size(match.group(1))


class ThreadStackBudget(unittest.TestCase):
    def setUp(self):
        self.tasks = (ROOT / "Core/Inc/fctasks.h").read_text()
        self.pool = _define((ROOT / "AZURE_RTOS/App/app_azure_rtos_config.h").read_text(),
                            "TX_APP_MEM_POOL_SIZE")
        self.stacks = {name: _define(self.tasks, name) for name in (
            "TLMT_STACK_BYTES", "RECV_STACK_BYTES", "EVAL_STACK_BYTES",
            "DIST_STACK_BYTES", "DMA_STACK_BYTES")}

    def test_router_callers_keep_the_measured_stack_budget(self):
        for name in ("TLMT_STACK_BYTES", "RECV_STACK_BYTES",
                     "EVAL_STACK_BYTES", "DIST_STACK_BYTES"):
            with self.subTest(task=name):
                self.assertGreaterEqual(self.stacks[name], ROUTER_CALLER_MIN_BYTES)
                self.assertEqual(self.stacks[name] % 8, 0)
        self.assertGreaterEqual(self.stacks["DIST_STACK_BYTES"], DIST_MIN_BYTES)

    def test_threadx_pool_holds_every_task_stack(self):
        needed = sum(self.stacks.values()) + POOL_FIXED_OVERHEAD + \
            POOL_BLOCK_OVERHEAD * len(self.stacks)
        self.assertGreaterEqual(self.pool, needed)
        # The tail after the stacks becomes the telemetry TLSF fallback region;
        # a stack increase must grow the pool, not consume that tail.
        self.assertGreaterEqual(self.pool - needed, 4000)
        self.assertLess(self.pool - needed, 4096)

    def test_cubemx_owns_the_same_pool_size(self):
        ioc = (ROOT / "FlightComputer26.ioc").read_text(encoding="utf-8")
        self.assertIn(f"THREADX.TX_APP_MEM_POOL_SIZE={self.pool}", ioc)

    def test_every_router_caller_reports_its_low_water_mark(self):
        telemetry = (ROOT / "Core/Src/telemetry.c").read_text()
        layout = json.loads((ROOT / "sim/board.json").read_text())
        probes = {p["symbol"]: p for p in layout["execution"]["memory_probes"]}
        for thread, symbol in (("telemetry_task", "g_telemetry_stack_free_min"),
                               ("evaluation_task", "g_evaluation_stack_free_min"),
                               ("distribution_task", "g_distribution_stack_free_min")):
            with self.subTest(symbol=symbol):
                self.assertIn(f"stack_sample_fill(&{thread}, &{symbol})", telemetry)
                self.assertIn(f"thread == &{thread}", telemetry)
                self.assertGreaterEqual(probes[symbol].get("minimum", 0), 2048)


if __name__ == "__main__":
    unittest.main()
