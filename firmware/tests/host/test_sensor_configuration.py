"""Exercise the actual build gate, including stale resolved sdkconfig values."""
from pathlib import Path
import subprocess
import tempfile
import unittest


VALIDATOR = Path(__file__).resolve().parents[2] / "components/node_sensors/sensor_config.cmake"


class SensorConfiguration(unittest.TestCase):
    def check_configuration(self, a0=0, a1=0, s0=1, s1=0, rom0="", rom1="", *, valid=True):
        values = dict(CURA_SEN0193_0=a0, CURA_SEN0193_1=a1,
                      CURA_SEN0600_0=s0, CURA_SEN0600_1=s1,
                      CURA_DS18B20_0_ROM=rom0, CURA_DS18B20_1_ROM=rom1)
        with tempfile.TemporaryDirectory() as directory:
            script = Path(directory) / "check.cmake"
            script.write_text("".join(f'set(CONFIG_{key} "{value}")\n' for key, value in values.items())
                              + f'include("{VALIDATOR}")\n')
            result = subprocess.run(["cmake", "-P", str(script)], capture_output=True, text=True)
        self.assertEqual(result.returncode == 0, valid, result.stdout + result.stderr)

    def test_valid_selections(self):
        self.check_configuration()
        self.check_configuration(s0=254, s1=1)
        self.check_configuration(a0=1, s0=0, rom0="14000000BFA86A28")
        self.check_configuration(a0=1, a1=1, s0=0,
                                 rom0="14000000BFA86A28", rom1="7E000000540FA728")

    def test_invalid_selections(self):
        for values in [dict(s0=0), dict(s0=255), dict(s0=-1), dict(s1=1),
                       dict(a0=1), dict(a1=1), dict(a0=2),
                       dict(a0=1, s0=0), dict(rom0="14000000BFA86A28"),
                       dict(rom1="0000000000000000"),
                       dict(a0=1, s0=0, rom0="0000000000000000"),
                       dict(a0=1, s0=0, rom0="14000000BFA86A2X"),
                       dict(a0=1, a1=1, s0=0, rom0="14000000BFA86A28", rom1="14000000bfa86a28"),
                       dict(a0=1, s0=0, s1=2, rom0="14000000BFA86A28")]:
            with self.subTest(values=values):
                self.check_configuration(**values, valid=False)


if __name__ == "__main__":
    unittest.main()
