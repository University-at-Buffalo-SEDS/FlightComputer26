import json
import unittest
from pathlib import Path

import build


class QualificationContractTests(unittest.TestCase):
    def test_flight_buzzer_routes_to_flight_controller_endpoint(self):
        root = Path(build.__file__).resolve().parent
        schema = json.loads((root / "config" / "sedsnet.json").read_text(encoding="utf-8"))
        buzzer = next(item for item in schema["types"] if item["name"] == "FLIGHT_BUZZER")
        self.assertEqual(buzzer["endpoints"], ["ActuatorFlightController"])

    def test_router_clock_epoch_precedes_router_and_side_setup(self):
        root = Path(build.__file__).resolve().parent
        telemetry = (root / "Core" / "Src" / "telemetry.c").read_text(encoding="utf-8")
        create = telemetry.index("seds_router_new(")
        epoch = telemetry.index("g_router.start_time = init_now_ms;")
        side = telemetry.index('r, "can", 3U, tx_send')

        self.assertLess(epoch, create)
        self.assertLess(epoch, side)
        self.assertEqual(telemetry.count("g_router.start_time ="), 1)

    def test_telemetry_loop_services_received_and_transmit_work_together(self):
        root = Path(build.__file__).resolve().parent
        telemetry = (root / "Core" / "Src" / "telemetry.c").read_text(
            encoding="utf-8"
        )
        loop = telemetry.split("MrAnalog (WE_ARE_SO_BACK)", 1)[1]
        self.assertIn("process_all_queues_timeout(TELEMETRY_QUEUE_SERVICE_BUDGET_MS)", loop)
        self.assertNotIn("dispatch_tx_queue_timeout(TELEMETRY_QUEUE_SERVICE_BUDGET_MS)", loop)

    def test_full_runner_profiles_memory_and_linked_network(self):
        root = Path(build.__file__).resolve().parent
        runner = (root / "sim" / "run_full.py").read_text(encoding="utf-8")
        script = (root / "build.py").read_text(encoding="utf-8")

        self.assertIn('"profile"', runner)
        self.assertIn('"--sample-count", "20"', runner)
        self.assertIn('"--traffic-iterations", "1000000"', runner)
        self.assertIn('"bay"', runner)
        self.assertIn('"tx_probe": "fdcan_tx_ok"', runner)
        self.assertIn('"rx_probe": "fdcan_rx"', runner)
        self.assertIn('"host_nodes"', runner)
        self.assertIn('"groundstation"', runner)
        self.assertIn('"rocket_radio"', runner)
        self.assertIn('"fill_pico"', runner)
        self.assertIn('"GS_SIM_VALIDATE_VALVE_ROUNDTRIP": "1"', runner)
        self.assertIn('"probe": "valve_commands_received", "minimum": 1', runner)
        self.assertIn("forwarded status ACK to GroundStation", runner)
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

    def test_can_transmit_does_not_override_network_controlled_underglow(self):
        root = Path(build.__file__).resolve().parent
        telemetry = (root / "Core" / "Src" / "telemetry.c").read_text(encoding="utf-8")
        tx_send = telemetry[telemetry.index("SedsResult tx_send"):telemetry.index("static void telemetry_can_rx")]
        self.assertNotIn("LED2_PORT", tx_send)
        self.assertNotIn("blink(", tx_send)
        underglow = (root / "Core" / "Src" / "av_bay_underglow.c").read_text(
            encoding="utf-8"
        )
        self.assertIn("seds_router_enable_network_variable", underglow)
        self.assertIn("seds_router_on_network_variable_update", underglow)
        self.assertIn("seds_router_get_network_variable_packed_len", underglow)
        self.assertNotIn("seds_router_request_managed_variable", underglow)
        self.assertIn("HAL_GPIO_WritePin(LED2_PORT, LED2_PIN", underglow)

    def test_shared_can_avoids_hop_retry_storms(self):
        root = Path(build.__file__).resolve().parent
        telemetry = (root / "Core" / "Src" / "telemetry.c").read_text(encoding="utf-8")
        cmake = (root / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn('seds_router_add_side_packed(r, "can", 3U, tx_send, NULL, false)', telemetry)
        self.assertIn('SEDSNET_MAX_QUEUE_BUDGET "8192"', cmake)

    def test_sedsnet_can_payload_budget_matches_avionics_peers(self):
        root = Path(build.__file__).resolve().parent
        cmake = (root / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn('set(SEDSNET_MAX_STACK_PAYLOAD "128"', cmake)
        self.assertNotIn('set(SEDSNET_MAX_STACK_PAYLOAD "8"', cmake)

    def test_sd_failure_yields_instead_of_starving_telemetry(self):
        root = Path(build.__file__).resolve().parent
        filex = (root / "FileX" / "App" / "app_filex.c").read_text(encoding="utf-8")
        self.assertIn("tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND)", filex)
        self.assertNotIn("blink(Blue, true, 1);\n      blink(Blue, false, 1);", filex)

    def test_constrained_discovery_is_primed_after_can_and_router_startup(self):
        root = Path(build.__file__).resolve().parent
        telemetry = (root / "Core" / "Src" / "telemetry.c").read_text(encoding="utf-8")
        can_init = telemetry.index("can_bus_init(&hfdcan1)")
        router_init = telemetry.index("init_telemetry_router()", can_init)
        announce = telemetry.index("telemetry_poll_discovery()", router_init)
        self.assertLess(can_init, router_init)
        self.assertLess(router_init, announce)
        self.assertIn(
            "process_all_queues_timeout(TELEMETRY_QUEUE_SERVICE_BUDGET_MS)",
            telemetry[announce:],
        )

    def test_can_callback_dispatches_rx_and_tx_service_is_bounded(self):
        root = Path(build.__file__).resolve().parent
        telemetry = (root / "Core" / "Src" / "telemetry.c").read_text(
            encoding="utf-8"
        )
        callback = telemetry[telemetry.index("void rx_asynchronous") :]
        callback = callback[: callback.index("static UNUSED_FUNCTION")]
        self.assertIn("seds_router_receive_packed_from_side", callback)
        self.assertNotIn("seds_router_rx_packed_packet_to_queue_from_side", callback)
        self.assertIn("#define TELEMETRY_QUEUE_SERVICE_BUDGET_MS 1U", telemetry)
        self.assertIn(
            "process_all_queues_timeout(TELEMETRY_QUEUE_SERVICE_BUDGET_MS)",
            telemetry,
        )

        can_bus = (root / "Core" / "Src" / "can_bus.c").read_text(
            encoding="utf-8"
        )
        send_large = can_bus[can_bus.index("HAL_StatusTypeDef can_bus_send_large") :]
        send_large = send_large[: send_large.index("void can_bus_process_rx")]
        self.assertNotIn("tx_thread_sleep", send_large)
        self.assertIn("if (st != HAL_OK)\n      return st;", send_large)


    def test_periodic_health_check_does_not_serialize_topology(self):
        root = Path(build.__file__).resolve().parent
        telemetry = (root / "Core" / "Src" / "telemetry.c").read_text(encoding="utf-8")
        self.assertNotIn("seds_router_export_topology_len", telemetry)
        self.assertIn("g_telemetry_discovery_seen = 1U", telemetry)

if __name__ == "__main__":
    unittest.main()
