import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

class CanHardwareContract(unittest.TestCase):
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
