import json
import unittest
from pathlib import Path

import build


class QualificationContractTests(unittest.TestCase):
    def test_full_runner_profiles_memory_and_linked_network(self):
        root = Path(build.__file__).resolve().parent
        runner = (root / "sim" / "run_full.py").read_text(encoding="utf-8")
        script = (root / "build.py").read_text(encoding="utf-8")

        self.assertIn('"profile"', runner)
        self.assertIn('"--sample-count", "20"', runner)
        self.assertIn('"--traffic-iterations", "1000000"', runner)
        self.assertIn('"bay"', runner)
        self.assertIn('"activity_probe": "network_ready"', runner)
        self.assertIn('simulation_env["SEDS_FIRMWARE_SIM_TEST"] = "1"', runner)
        self.assertIn('run_live(command, "firmware simulation")', runner)
        self.assertIn('running ({int(now - started)}s elapsed)', runner)
        self.assertIn("Long-duration memory profile", script)
        self.assertIn("Network discovery and time sync", script)

    def test_layout_exposes_network_convergence(self):
        root = Path(build.__file__).resolve().parent
        layout = json.loads((root / "sim" / "board.json").read_text(encoding="utf-8"))
        self.assertLess(layout["execution"].get("memory_probe_warmup_samples", 0), layout["execution"]["sample_count"])
        probes = {
            probe["name"]: probe["symbol"]
            for probe in layout["execution"]["memory_probes"]
        }
        self.assertEqual(probes["network_ready"], "g_telemetry_network_ready")
        self.assertEqual(probes["discovery_seen"], "g_telemetry_discovery_seen")
        self.assertEqual(probes["timesync_valid"], "g_telemetry_timesync_valid")
        self.assertEqual(probes["fdcan_rx"], "g_fdcan_rx_count")

        telemetry = (root / "Core" / "Src" / "telemetry.c").read_text(encoding="utf-8")
        for symbol in (
            "g_telemetry_network_ready",
            "g_telemetry_discovery_seen",
            "g_telemetry_timesync_valid",
        ):
            self.assertIn(symbol, telemetry)

        can_bus = (root / "Core" / "Src" / "can_bus.c").read_text(encoding="utf-8")
        self.assertIn("g_fdcan_rx_count++", can_bus)

    def test_successful_can_transmit_has_nonblocking_led_indicator(self):
        root = Path(build.__file__).resolve().parent
        telemetry = (root / "Core" / "Src" / "telemetry.c").read_text(encoding="utf-8")
        tx_send = telemetry[telemetry.index("SedsResult tx_send"):telemetry.index("static void telemetry_can_rx")]
        self.assertIn("led_toggle(LED2_PORT, LED2_PIN)", tx_send)
        self.assertNotIn("blink(", tx_send)

    def test_shared_can_avoids_hop_retry_storms(self):
        root = Path(build.__file__).resolve().parent
        telemetry = (root / "Core" / "Src" / "telemetry.c").read_text(encoding="utf-8")
        cmake = (root / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn('seds_router_add_side_packed(r, "can", 3U, tx_send, NULL, false)', telemetry)
        self.assertIn('SEDSNET_MAX_QUEUE_BUDGET "12288"', cmake)

    def test_sd_failure_yields_instead_of_starving_telemetry(self):
        root = Path(build.__file__).resolve().parent
        filex = (root / "FileX" / "App" / "app_filex.c").read_text(encoding="utf-8")
        self.assertIn("tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND)", filex)
        self.assertNotIn("blink(Blue, true, 1);\n      blink(Blue, false, 1);", filex)

    def test_discovery_is_primed_after_can_and_router_startup(self):
        root = Path(build.__file__).resolve().parent
        telemetry = (root / "Core" / "Src" / "telemetry.c").read_text(encoding="utf-8")
        can_init = telemetry.index("can_bus_init(&hfdcan1)")
        router_init = telemetry.index("init_telemetry_router()", can_init)
        announce = telemetry.index("telemetry_announce_discovery()", router_init)
        self.assertLess(can_init, router_init)
        self.assertLess(router_init, announce)
        self.assertIn("process_all_queues_timeout(50)", telemetry[announce:])


    def test_periodic_health_check_does_not_serialize_topology(self):
        root = Path(build.__file__).resolve().parent
        telemetry = (root / "Core" / "Src" / "telemetry.c").read_text(encoding="utf-8")
        self.assertNotIn("seds_router_export_topology_len", telemetry)
        self.assertIn("g_telemetry_discovery_seen = 1U", telemetry)

if __name__ == "__main__":
    unittest.main()
