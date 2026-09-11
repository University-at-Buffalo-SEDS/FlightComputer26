import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

class CanHardwareContract(unittest.TestCase):
    def test_flight_sensor_downlink_is_one_hz(self):
        config = (ROOT / "Core/Inc/fcconfig.h").read_text()
        distribution = (ROOT / "Core/Src/distribution.c").read_text()
        self.assertRegex(config, r"#define LOG_RATE_GND\s+1000")
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
