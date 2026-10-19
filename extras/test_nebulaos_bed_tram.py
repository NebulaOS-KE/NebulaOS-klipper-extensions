# Tests for extras/nebulaos_bed_tram.py (G87): guided bed tramming with
# simulated SCREWS_TILT_CALCULATE results.
#
# Run from klippy/: python3 -m unittest extras.test_nebulaos_bed_tram -v
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import json
import os
import shutil
import tempfile
import unittest

from . import prtouch_test_support as fake
from . import nebulaos_bed_tram as tram


class FakeScrewsTilt:
    """Upstream screws_tilt_adjust's status, fed one measurement per call."""
    def __init__(self, measurements):
        self.measurements = list(measurements)
        self.results = {}
        self.calls = 0

    def measure(self):
        self.calls += 1
        zs = self.measurements.pop(0)
        self.results = {}
        for i, z in enumerate(zs):
            diff = zs[0] - z
            turns = abs(diff) / 0.5
            self.results['screw%d' % (i + 1,)] = {
                'z': z, 'is_base': i == 0,
                'sign': 'CW' if diff >= 0 else 'CCW',
                'adjust': '%02d:%02d' % (int(turns),
                                         round((turns - int(turns)) * 60))}

    def get_status(self, eventtime):
        return {'error': False, 'max_deviation': None,
                'results': self.results}


class TramGCode(fake.FakeGCode):
    def __init__(self, tilt):
        super().__init__()
        self.tilt = tilt
        self.raw = []

    def run_script_from_command(self, script):
        self.scripts_run.append(script)
        if script.startswith('SCREWS_TILT_CALCULATE'):
            self.tilt.measure()

    def respond_raw(self, msg):
        self.raw.append(msg)

    def last_prompt(self):
        start = max(i for i, m in enumerate(self.raw)
                    if m.startswith('// action:prompt_begin'))
        lines, buttons = [], []
        for m in self.raw[start:]:
            if m.startswith('// action:prompt_text '):
                lines.append(m[len('// action:prompt_text '):])
            elif m.startswith('// action:prompt_button '):
                buttons.append(tuple(m[len('// action:prompt_button '):]
                                     .split('|')))
        return lines, buttons


class FakePrintStats:
    def __init__(self, state='standby'):
        self.state = state

    def get_status(self, eventtime):
        return {'state': self.state}


class TramTestBase(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.tmp)
        self.state_path = os.path.join(self.tmp, 'cal', 'bed_tram.json')

    def build(self, measurements, mount=None, homed='xyz',
              print_state='standby', **cfg):
        if mount is not None:
            os.makedirs(os.path.dirname(self.state_path), exist_ok=True)
            with open(self.state_path, 'w') as f:
                json.dump({'schema': 1, 'mount': mount}, f)
        printer = fake.FakePrinter()
        tilt = FakeScrewsTilt(measurements)
        gcode = TramGCode(tilt)
        toolhead = fake.FakeToolhead()
        toolhead.homed_axes = homed
        printer.add_object('gcode', gcode)
        printer.add_object('toolhead', toolhead)
        printer.add_object('screws_tilt_adjust', tilt)
        printer.add_object('print_stats', FakePrintStats(print_state))
        values = {'state_path': self.state_path}
        values.update({k: str(v) for k, v in cfg.items()})
        config = fake.FakeConfig(values, section='nebulaos_bed_tram',
                                 printer=printer)
        obj = tram.NebulaOSBedTram(config)
        config.assert_all_consumed()
        return obj, gcode, tilt

    def tram_cmd(self, obj, **params):
        obj.cmd_tram(fake.FakeGCmd({k: str(v) for k, v in params.items()}))


class HelperTest(unittest.TestCase):
    def test_rigid_deviation_is_below_the_highest_corner(self):
        self.assertEqual([round(d, 3) for d in
                          tram.deviations([0.0, 0.15, 0.03, 0.12], 'rigid')],
                         [0.15, 0.0, 0.12, 0.03])

    def test_adjustable_deviation_is_from_front_left(self):
        self.assertEqual([round(d, 3) for d in
                          tram.deviations([0.0, 0.15, -0.2, 0.05],
                                          'adjustable')],
                         [0.0, 0.15, 0.2, 0.05])

    def test_shim_never_overshoots(self):
        self.assertAlmostEqual(tram.shim_mm(0.15, 0.10, 0.5), 0.10)
        self.assertAlmostEqual(tram.shim_mm(0.30, 0.10, 0.5), 0.30)
        self.assertAlmostEqual(tram.shim_mm(0.99, 0.10, 0.5), 0.50)

    def test_turn_words(self):
        self.assertEqual(tram.turn_words('00:15'), 'a quarter of a turn')
        self.assertEqual(tram.turn_words('01:30'),
                         '1 full turn and a half of a turn')
        self.assertEqual(tram.turn_words('00:59'), '1 full turn')


class RigidFlowTest(TramTestBase):
    def test_spec_example_one_corner_at_a_time(self):
        # Tim's example: FL +0.15, FR 0, RR +0.12, RL +0.03 (shim needed)
        obj, gcode, tilt = self.build(
            [[0.00, 0.15, 0.03, 0.12],      # FL lowest
             [0.09, 0.15, 0.03, 0.12],      # FL got 0.10 -> 0.06 left: OK
             [0.09, 0.15, 0.13, 0.12]],     # RR got 0.10 -> all OK
            mount='rigid')
        self.tram_cmd(obj)
        self.assertEqual(obj.state, tram.AWAITING_ADJUSTMENT)
        self.assertEqual(obj.target, 0)
        lines, buttons = gcode.last_prompt()
        self.assertTrue(any('Front left: add a 0.10 mm shim.' in l
                            for l in lines))
        self.assertTrue(any('Do not touch the other corners' in l
                            for l in lines))
        self.assertTrue(any('load cell' in l for l in lines))
        self.assertIn(('Done - measure again', 'NEBULAOS_BED_TRAM CONFIRM=1'),
                      buttons)
        self.tram_cmd(obj, CONFIRM=1)
        self.assertEqual(obj.target, 2)                    # rear right next
        self.assertTrue(obj.accepted[0])
        lines, _ = gcode.last_prompt()
        self.assertTrue(any('Front left: 0.06 mm - OK' in l for l in lines))
        self.assertTrue(any('Rear right: add a 0.10 mm shim.' in l
                            for l in lines))
        self.tram_cmd(obj, CONFIRM=1)
        self.assertEqual(obj.state, tram.COMPLETE)
        lines, buttons = gcode.last_prompt()
        self.assertIn('Bed tramming complete', lines[0])
        # front-left (load cell) was shimmed -> recalibrate the load cell
        self.assertIn(('Calibrate load cell', 'NEBULAOS_LOAD_CELL_CALIBRATE'),
                      buttons)

    def test_already_level_offers_z_offset_and_mesh(self):
        obj, gcode, tilt = self.build([[0.0, 0.04, 0.08, 0.02]],
                                      mount='rigid')
        self.tram_cmd(obj)
        self.assertEqual(obj.state, tram.COMPLETE)
        _, buttons = gcode.last_prompt()
        self.assertIn(('Z offset & bed mesh', 'NEBULAOS_AUTO_CALIBRATE'),
                      buttons)

    def test_no_improvement_stops_after_two_tries(self):
        obj, gcode, tilt = self.build(
            [[0.0, 0.3, 0.3, 0.3]] * 3, mount='rigid')
        self.tram_cmd(obj)
        self.tram_cmd(obj, CONFIRM=1)
        self.assertEqual(obj.state, tram.AWAITING_ADJUSTMENT)
        self.tram_cmd(obj, CONFIRM=1)
        self.assertEqual(obj.state, tram.STOPPED)
        self.assertIn('did not improve after 2 adjustments', obj.message)

    def test_ok_corner_is_never_selected_again(self):
        obj, gcode, tilt = self.build(
            [[0.0, 0.3, 0.25, 0.3],         # FL needs 0.30
             [0.3, 0.3, 0.25, 0.3],         # FL fixed; RR needs 0.05: OK
             [0.3, 0.3, 0.25, 0.3]],
            mount='rigid')
        self.tram_cmd(obj)
        self.assertEqual(obj.target, 0)
        self.tram_cmd(obj, CONFIRM=1)
        self.assertEqual(obj.state, tram.COMPLETE)

    def test_corner_that_moves_out_again_stops_with_explanation(self):
        obj, gcode, tilt = self.build(
            [[0.0, 0.3, 0.3, 0.25],         # FL 0.30, RL 0.05 (OK)
             [0.3, 0.3, 0.3, 0.05]],        # RL dropped after FL shim
            mount='rigid')
        self.tram_cmd(obj)
        self.tram_cmd(obj, CONFIRM=1)
        self.assertEqual(obj.state, tram.STOPPED)
        self.assertIn('rear left', obj.message)

    def test_noise_inside_tolerance_does_not_trigger_micro_adjustments(self):
        obj, gcode, tilt = self.build([[0.0, 0.098, 0.05, 0.1]],
                                      mount='rigid')
        self.tram_cmd(obj)
        self.assertEqual(obj.state, tram.COMPLETE)


class AdjustableFlowTest(TramTestBase):
    def test_turn_instruction_and_front_left_never_turned(self):
        obj, gcode, tilt = self.build(
            [[0.0, 0.0, -0.125, 0.02],      # RR 0.125 low vs FL base
             [0.0, 0.0, 0.0, 0.02]],
            mount='adjustable')
        self.tram_cmd(obj)
        self.assertEqual(obj.target, 2)
        lines, _ = gcode.last_prompt()
        # direction and amount are upstream's (fake mirrors its CW-M3 rule)
        self.assertTrue(any('Rear right: turn the screw a quarter of a turn '
                            'clockwise.' in l for l in lines))
        self.assertFalse(any('load cell' in l for l in lines))
        self.tram_cmd(obj, CONFIRM=1)
        self.assertEqual(obj.state, tram.COMPLETE)
        self.assertFalse(obj.load_cell_corner_changed)


class SessionTest(TramTestBase):
    def test_mount_is_asked_once_and_remembered(self):
        obj, gcode, tilt = self.build([[0.0, 0.0, 0.0, 0.0]])
        self.tram_cmd(obj)
        self.assertEqual(obj.state, tram.AWAITING_MOUNT)
        self.assertEqual(tilt.calls, 0)                  # nothing moved yet
        _, buttons = gcode.last_prompt()
        self.assertIn(('Rigid spacers', 'NEBULAOS_BED_TRAM MOUNT=rigid'),
                      buttons)
        self.tram_cmd(obj, MOUNT='rigid')
        self.assertEqual(obj.state, tram.COMPLETE)
        with open(self.state_path) as f:
            self.assertEqual(json.load(f)['mount'], 'rigid')
        obj2, gcode2, tilt2 = self.build([[0.0, 0.0, 0.0, 0.0]])
        self.assertEqual(obj2.mount, 'rigid')

    def test_homes_first_when_needed(self):
        obj, gcode, tilt = self.build([[0.0, 0.0, 0.0, 0.0]], mount='rigid',
                                      homed='')
        self.tram_cmd(obj)
        self.assertEqual(gcode.scripts_run[0], 'G28')
        self.assertIn('SAMPLES=3 SAMPLES_RESULT=median', gcode.scripts_run[1])

    def test_refused_while_printing(self):
        obj, gcode, tilt = self.build([], mount='rigid',
                                      print_state='printing')
        with self.assertRaises(fake.CommandError):
            self.tram_cmd(obj)
        self.assertEqual(tilt.calls, 0)

    def test_start_while_active_shows_the_current_step(self):
        obj, gcode, tilt = self.build([[0.0, 0.3, 0.3, 0.3]], mount='rigid')
        self.tram_cmd(obj)
        self.tram_cmd(obj)
        self.assertEqual(tilt.calls, 1)                  # no second measure

    def test_confirm_without_session_is_refused(self):
        obj, gcode, tilt = self.build([], mount='rigid')
        with self.assertRaises(fake.CommandError):
            self.tram_cmd(obj, CONFIRM=1)

    def test_cancel(self):
        obj, gcode, tilt = self.build([[0.0, 0.3, 0.3, 0.3]], mount='rigid')
        self.tram_cmd(obj)
        obj.cmd_cancel(fake.FakeGCmd({}))
        self.assertEqual(obj.state, tram.CANCELLED)


if __name__ == '__main__':
    unittest.main()
