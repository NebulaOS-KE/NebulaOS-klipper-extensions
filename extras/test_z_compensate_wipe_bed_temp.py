# Run 13 (2026-10-18): the nozzle wipe's bed temperature. On ke-dev
# NEBULAOS_AUTO_CALIBRATE heated the bed to 65 C, then the wipe asked for
# 65 + bed_add_temp 60 = 125 C (> [heater_bed] max_temp 120) and Klipper
# refused ("Requested temperature (125.0) out of range (0.0:120.0)").
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import os
import unittest

from . import z_compensate


class WipeBedTempTest(unittest.TestCase):
    def test_auto_calibrate_case_stays_at_the_calibration_temperature(self):
        self.assertEqual(z_compensate.wipe_bed_temp(65., 60., 120.), 65.)

    def test_cold_bed_is_heated_to_bed_add_temp(self):
        self.assertEqual(z_compensate.wipe_bed_temp(0., 60., 120.), 60.)

    def test_never_above_the_bed_max_temp(self):
        self.assertEqual(z_compensate.wipe_bed_temp(110., 130., 120.), 120.)
        for target in range(0, 121, 5):
            self.assertLessEqual(z_compensate.wipe_bed_temp(target, 60., 120.), 120.)

    def test_no_bed_add_temp_keeps_the_target(self):
        self.assertEqual(z_compensate.wipe_bed_temp(55., 0., 120.), 55.)
        self.assertEqual(z_compensate.wipe_bed_temp(None, None, None), 0.)

    def test_the_wipe_command_uses_it(self):
        here = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(here, 'z_compensate.py')) as fh:
            src = fh.read()
        self.assertIn('wipe_bed_temp(bed_target, bed_add_temp, bed_max)', src)
        self.assertNotIn('bed_target + bed_add_temp,', src)


if __name__ == '__main__':
    unittest.main()
