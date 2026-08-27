import tempfile
import unittest
from pathlib import Path
from unittest import mock

import build


class OtaBuildScriptTests(unittest.TestCase):
    def test_ota_option_is_available(self):
        _preset, options = build.parse(["release", "ota"])
        self.assertEqual(options["image"], "ota")

    def test_ota_is_a_full_recovery_seds_image(self):
        with tempfile.TemporaryDirectory() as directory:
            build_dir = Path(directory)

            def fake_build(selected_dir, _target):
                (selected_dir / "FlightComputer26.launchcore.img").write_bytes(
                    b"packaged-launchcore-image"
                )

            with mock.patch.object(build, "build", side_effect=fake_build):
                artifact, address = build.select_artifact(build_dir, "ota")

            self.assertEqual(artifact.suffix, ".seds")
            self.assertEqual(artifact.read_bytes(), b"packaged-launchcore-image")
            self.assertEqual(address, build.APP_ADDR)


if __name__ == "__main__":
    unittest.main()
