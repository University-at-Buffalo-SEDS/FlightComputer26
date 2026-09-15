import unittest
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

class CanHardwareContract(unittest.TestCase):
    def test_sim_requires_real_sensor_progress(self):
        layout = json.loads((ROOT / "sim/board.json").read_text())
        probes = {p["name"]: p for p in layout["execution"]["memory_probes"]}
        for name in ("sensor_dma_completed", "sensor_dma_delivered",
                     "imu_publish_ok", "barometer_publish_ok"):
            self.assertGreaterEqual(probes[name]["minimum"], 1)
        connections = {(p["from"], p["to"]) for p in layout["board"]["connections"]}
        self.assertIn(("gpio.35", "layoutFlightSensors@0"), connections)
        self.assertIn(("layoutFlightSensors.0", "exti@39"), connections)
        routes = {(r["request"], r["channel"]) for r in layout["board"]["dma_routes"]}
        self.assertIn(("spi1.DMARecieve", 0), routes)
        self.assertIn(("spi1.DMATransmit", 1), routes)

    def test_simulation_cannot_inject_sensor_packets_into_router(self):
        source = (ROOT / "Core/Src/telemetry.c").read_text()
        self.assertNotIn("next_sim_sensor_ms", source)
        self.assertNotIn("seds_router_log_typed(g_router.r, SEDS_DT_IMU_DATA", source)
        self.assertIn("Observe real producer calls", source)

    def test_release_vendor_lto_survives_cubemx_regeneration(self):
        cmake = (ROOT / "CMakeLists.txt").read_text()
        self.assertIn("foreach(FC_VENDOR_TARGET STM32_Drivers FileX ThreadX)", cmake)
        self.assertIn("$<$<AND:$<CONFIG:Release>,$<COMPILE_LANGUAGE:C>>:-flto>", cmake)

    def test_flight_sensor_downlink_uses_shared_rate(self):
        config = (ROOT / "Core/Inc/fcconfig.h").read_text()
        distribution = (ROOT / "Core/Src/distribution.c").read_text()
        self.assertRegex(config, r"#define LOG_RATE_GND\s+FC_TELEMETRY_PERIOD_MS")
        self.assertIn("timer_probe(mems[dev].tim_gnd, rates.gnd)", distribution)
        self.assertNotIn("timer_probe(mems[dev].tim_gnd, rates.gnd * 2)", distribution)

    def test_matches_the_avionics_fd_bus(self):
        source = (ROOT / "Core/Src/main.c").read_text()
        ioc = (ROOT / "FlightComputer26.ioc").read_text()
        self.assertIn("hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_NO_BRS", source)
        self.assertIn("hfdcan1.Init.AutoRetransmission = ENABLE", source)
        self.assertIn("hfdcan1.Init.NominalPrescaler = 10", source)
        self.assertIn("hfdcan1.Init.NominalTimeSeg1 = 13", source)
        self.assertIn("hfdcan1.Init.NominalTimeSeg2 = 3", source)
        self.assertIn("FDCAN1.CalculateBaudRateNominal=999999", ioc)
        self.assertIn("FDCAN1.AutoRetransmission=ENABLE", ioc)
        can = (ROOT / "Core/Src/can_bus.c").read_text()
        self.assertIn("CAN_BUS_TX_ENQUEUE_TIMEOUT_MS 5U", can)
        self.assertIn("HAL_FDCAN_AbortTxRequest", can)
        self.assertNotIn("< (uint32_t)frag_cnt", can)
