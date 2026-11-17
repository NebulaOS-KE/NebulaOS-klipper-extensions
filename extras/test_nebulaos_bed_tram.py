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
    # The KE's screws (calibration.cfg), as upstream keeps them.
    screws = [((27.5, -0.5), 'front left'), ((196.5, -0.5), 'front right'),
              ((196.5, 169.5), 'rear right'), ((27.5, 169.5), 'rear left')]

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


class TramToolhead(fake.FakeToolhead):
    def __init__(self):
        super().__init__()
        self.manual_moves = []

    def manual_move(self, coord, speed):
        self.manual_moves.append((list(coord), speed))


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
        toolhead = TramToolhead()
        toolhead.homed_axes = homed
        self.toolhead = toolhead
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
        # An example: FL +0.15, FR 0, RR +0.12, RL +0.03 (shim needed)
        obj, gcode, tilt = self.build(
            [[0.00, 0.15, 0.03, 0.12],      # FL lowest
             [0.09, 0.15, 0.03, 0.12],      # FL got 0.10 -> 0.06 left: OK
             [0.09, 0.15, 0.13, 0.12]],     # RR got 0.10 -> all OK
            mount='rigid')
        self.tram_cmd(obj)
        self.assertEqual(obj.state, tram.AWAITING_ADJUSTMENT)
        self.assertEqual(obj.target, 0)
        lines, buttons = gcode.last_prompt()
        self.assertIn('Front left: -0.15 mm. Add a 0.10 mm shim to raise it.',
                      lines)
        # Signed, relative to the highest corner (2026-10-21).
        self.assertIn('Front left: -0.15 mm - ADJUST', lines)
        self.assertIn('Front right: 0.00 mm - highest', lines)
        self.assertIn('Rear right: -0.12 mm - later', lines)
        self.assertIn('Rear left: -0.03 mm - OK', lines)
        self.assertTrue(any('Do not touch the other corners' in l
                            for l in lines))
        self.assertTrue(any('load cell' in l for l in lines))
        self.assertIn(('Done - measure again', 'NEBULAOS_BED_TRAM CONFIRM=1'),
                      buttons)
        self.tram_cmd(obj, CONFIRM=1)
        self.assertEqual(obj.target, 2)                    # rear right next
        self.assertTrue(obj.accepted[0])
        lines, _ = gcode.last_prompt()
        self.assertIn('Front left: -0.06 mm - OK', lines)
        self.assertIn('Rear right: -0.12 mm - ADJUST', lines)
        self.assertIn('Rear right: -0.12 mm. Add a 0.10 mm shim to raise it.',
                      lines)
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
        self.assertTrue(any(l.startswith('Rear right: -0.1') and
                            l.endswith(' mm. Turn the screw a quarter of a '
                                       'turn clockwise.') for l in lines))
        self.assertIn('Front left: 0.00 mm - base', lines)
        self.assertIn('Rear left: +0.02 mm - OK', lines)
        self.assertFalse(any('load cell' in l for l in lines))
        self.tram_cmd(obj, CONFIRM=1)
        self.assertEqual(obj.state, tram.COMPLETE)
        self.assertFalse(obj.load_cell_corner_changed)

    def test_direction_is_upstreams_word_for_word(self):
        # Issue #18: the screen and Mainsail must never disagree. Mainsail
        # shows upstream's results[...]['sign']; the wizard must show the
        # same direction for both signs, whatever screw_thread says, and
        # never work one out itself.
        for upstream_sign, words in (('CW', 'clockwise'),
                                     ('CCW', 'counter-clockwise')):
            obj, gcode, tilt = self.build(
                [[0.0, 0.0, -0.125, 0.02], [0.0, 0.0, 0.0, 0.02]],
                mount='adjustable')
            measure = tilt.measure

            def forced(sign=upstream_sign, measure=measure):
                measure()
                for r in tilt.results.values():
                    r['sign'] = sign
            tilt.measure = forced
            self.tram_cmd(obj)
            self.assertEqual(obj.action['sign'], upstream_sign)
            lines, _ = gcode.last_prompt()
            self.assertTrue(any(l.endswith(' turn %s.' % (words,))
                                for l in lines), (upstream_sign, lines))


class SignedDeviationTest(TramTestBase):
    def test_rigid_signed_against_the_highest_corner(self):
        signed, ref = tram.signed_deviations([0.0, 0.15, 0.03, 0.12], 'rigid')
        self.assertEqual(ref, 1)
        self.assertEqual([round(v, 3) for v in signed],
                         [-0.15, 0.0, -0.12, -0.03])

    def test_adjustable_signed_against_the_base_corner(self):
        signed, ref = tram.signed_deviations([0.0, 0.15, -0.2, 0.05],
                                             'adjustable')
        self.assertEqual(ref, 0)
        self.assertEqual([round(v, 3) for v in signed],
                         [0.0, 0.15, -0.2, 0.05])

    def test_signed_format(self):
        self.assertEqual(tram.signed_mm(-0.23), '-0.23')
        self.assertEqual(tram.signed_mm(0.15), '+0.15')
        self.assertEqual(tram.signed_mm(-0.001), '0.00')
        self.assertEqual(tram.signed_mm(0.0), '0.00')

    def test_status_has_signed_values_and_keeps_deviations(self):
        obj, gcode, tilt = self.build([[0.0, 0.25, 0.03, 0.12]],
                                      mount='rigid')
        status = obj.get_status(0.)
        self.assertIsNone(status['signed_deviations'])
        self.assertIsNone(status['reference'])
        self.tram_cmd(obj)
        status = obj.get_status(0.)
        self.assertEqual(status['reference'], 1)
        self.assertEqual([round(v, 3) for v in status['signed_deviations']],
                         [-0.25, 0.0, -0.22, -0.13])
        # Unchanged for older screens: magnitudes.
        self.assertEqual([round(v, 3) for v in status['deviations']],
                         [0.25, 0.0, 0.22, 0.13])
        self.assertEqual(status['corners'][status['reference']],
                         'front right')

    def test_instruction_matches_tims_example(self):
        obj, gcode, tilt = self.build([[0.0, 0.23, 0.2, 0.22]],
                                      mount='rigid')
        self.tram_cmd(obj)
        self.assertTrue(obj.message.startswith(
            'Front left: -0.23 mm. Add a 0.20 mm shim to raise it.'))


class BusyTest(TramTestBase):
    """G91: 'busy' while a guided flow runs or waits for the user."""
    def test_busy_while_waiting_for_mount_and_adjustment(self):
        obj, gcode, tilt = self.build([[0.0, 0.3, 0.3, 0.3],
                                       [0.3, 0.3, 0.3, 0.3]])
        self.assertFalse(obj.get_status(0.)['busy'])
        self.tram_cmd(obj)
        self.assertEqual(obj.state, tram.AWAITING_MOUNT)
        self.assertTrue(obj.get_status(0.)['busy'])
        self.tram_cmd(obj, MOUNT='rigid')
        self.assertEqual(obj.state, tram.AWAITING_ADJUSTMENT)
        self.assertTrue(obj.get_status(0.)['busy'])
        self.tram_cmd(obj, CONFIRM=1)
        self.assertEqual(obj.state, tram.COMPLETE)
        self.assertFalse(obj.get_status(0.)['busy'])

    def test_busy_while_measuring(self):
        obj, gcode, tilt = self.build([[0.0, 0.0, 0.0, 0.0]], mount='rigid')
        seen = []
        measure = tilt.measure

        def spy():
            seen.append(obj.get_status(0.)['busy'])
            measure()
        tilt.measure = spy
        self.tram_cmd(obj)
        self.assertEqual(seen, [True])
        self.assertFalse(obj.get_status(0.)['busy'])

    def test_not_busy_after_cancel_or_stop(self):
        obj, gcode, tilt = self.build([[0.0, 0.3, 0.3, 0.3]] * 4,
                                      mount='rigid')
        self.tram_cmd(obj)
        obj.cmd_cancel(fake.FakeGCmd({}))
        self.assertFalse(obj.get_status(0.)['busy'])
        self.tram_cmd(obj)
        self.tram_cmd(obj, CONFIRM=1)
        self.tram_cmd(obj, CONFIRM=1)
        self.assertEqual(obj.state, tram.STOPPED)
        self.assertFalse(obj.get_status(0.)['busy'])


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


class ClearTheScrewsTest(TramTestBase):
    """Issue #24: after each measurement the nozzle leaves the rear-left
    screw and waits between the rear screws."""
    def test_moves_to_the_middle_of_the_rear_edge_after_measuring(self):
        obj, gcode, tilt = self.build([[0., -0.3, 0., 0.]], mount='rigid')
        self.tram_cmd(obj)
        self.assertEqual(obj.state, tram.AWAITING_ADJUSTMENT)
        self.assertEqual(self.toolhead.manual_moves[0], ([112.0, 169.5], 50.))

    def test_after_every_measurement_also_the_last(self):
        obj, gcode, tilt = self.build(
            [[0., -0.3, 0., 0.], [0., 0., 0., 0.]], mount='rigid')
        self.tram_cmd(obj)
        self.tram_cmd(obj, CONFIRM=1)
        self.assertEqual(obj.state, tram.COMPLETE)
        self.assertEqual([m[0] for m in self.toolhead.manual_moves],
                         [[112.0, 169.5], [None, None, 100.]] * 2)

    def test_no_move_when_the_measurement_failed(self):
        obj, gcode, tilt = self.build([[0., None, 0., 0.]], mount='rigid')
        def incomplete():
            tilt.calls += 1
            tilt.results = {'screw1': {'z': 0.}}
        tilt.measure = incomplete
        with self.assertRaises(Exception):
            self.tram_cmd(obj)
        self.assertEqual(self.toolhead.manual_moves, [])


class RecordingCalibration:
    """[nebulaos_calibration]'s history surface (G105)."""
    def __init__(self, fail=False):
        self.calls = []
        self.fail = fail

    def record_flow(self, key, summary):
        self.calls.append(('record', key, summary))
        if self.fail:
            raise RuntimeError("disk full")

    def mark_redo(self, keys, reason):
        self.calls.append(('redo', tuple(keys), reason))
        if self.fail:
            raise RuntimeError("disk full")


class CalibrationHistoryTest(TramTestBase):
    def _with_cal(self, measurements, fail=False):
        obj, gcode, tilt = self.build(measurements, mount='rigid')
        cal = RecordingCalibration(fail)
        obj.printer.add_object('nebulaos_calibration', cal)
        return obj, cal

    def test_shim_at_front_left_marks_load_cell_and_auto_calibrate(self):
        obj, cal = self._with_cal(
            [[0.00, 0.15, 0.13, 0.12], [0.09, 0.15, 0.13, 0.12]])
        self.tram_cmd(obj)
        self.assertEqual(cal.calls, [])          # nothing done yet
        self.tram_cmd(obj, CONFIRM=1)
        self.assertEqual(cal.calls[0], ('redo', ('load_cell', 'auto_calibrate'),
                                        'a shim was added at front left'))
        self.assertEqual(obj.state, tram.COMPLETE)
        self.assertEqual(cal.calls[1], ('record', 'bed_tram',
                                        'All corners within 0.06 mm'))

    def test_shim_elsewhere_marks_nothing(self):
        obj, cal = self._with_cal(
            [[0.15, 0.15, 0.00, 0.12], [0.15, 0.15, 0.10, 0.12]])
        self.tram_cmd(obj)
        self.assertEqual(obj.target, 2)
        self.tram_cmd(obj, CONFIRM=1)
        self.assertEqual([c[0] for c in cal.calls], ['record'])

    def test_stopped_flow_is_not_recorded_as_done(self):
        obj, cal = self._with_cal([[0.0, 0.3, 0.3, 0.3]] * 3)
        self.tram_cmd(obj)
        self.tram_cmd(obj, CONFIRM=1)
        self.tram_cmd(obj, CONFIRM=1)
        self.assertEqual(obj.state, tram.STOPPED)
        self.assertNotIn('record', [c[0] for c in cal.calls])

    def test_history_failure_never_breaks_tramming(self):
        obj, cal = self._with_cal(
            [[0.00, 0.15, 0.13, 0.12], [0.09, 0.15, 0.13, 0.12]], fail=True)
        self.tram_cmd(obj)
        with self.assertLogs(level='ERROR'):
            self.tram_cmd(obj, CONFIRM=1)
        self.assertEqual(obj.state, tram.COMPLETE)

    def test_without_calibration_object_nothing_happens(self):
        obj, gcode, tilt = self.build([[0.0, 0.04, 0.08, 0.02]],
                                      mount='rigid')
        self.tram_cmd(obj)
        self.assertEqual(obj.state, tram.COMPLETE)


class LiftAfterMeasuringTest(TramTestBase):
    """1.0.4: after measuring, the nozzle goes up so the bed plate comes off
    and the screws are easy to reach."""
    def test_goes_up_to_100_mm_after_the_xy_move(self):
        obj, gcode, tilt = self.build([[0., -0.3, 0., 0.]], mount='rigid')
        self.tram_cmd(obj)
        self.assertEqual(self.toolhead.manual_moves,
                         [([112.0, 169.5], 50.), ([None, None, 100.], 50.)])

    def test_height_is_configurable(self):
        obj, gcode, tilt = self.build([[0., -0.3, 0., 0.]], mount='rigid',
                                      clear_z=30)
        self.tram_cmd(obj)
        self.assertEqual(self.toolhead.manual_moves[-1][0], [None, None, 30.])

    def test_stays_below_the_z_limit(self):
        obj, gcode, tilt = self.build([[0., -0.3, 0., 0.]], mount='rigid')
        status = self.toolhead.get_status
        self.toolhead.get_status = lambda t: dict(status(t),
                                                  axis_maximum=[220., 220., 40., 0.])
        self.tram_cmd(obj)
        self.assertEqual(self.toolhead.manual_moves[-1][0], [None, None, 35.])

    def test_never_moves_down(self):
        obj, gcode, tilt = self.build([[0., -0.3, 0., 0.]], mount='rigid')
        self.toolhead.set_position_z(150.)
        self.tram_cmd(obj)
        self.assertEqual(self.toolhead.manual_moves, [([112.0, 169.5], 50.)])


class MeasuringStateTest(TramTestBase):
    """1.0.4: while homing and probing, the status says 'measuring', so the
    screen shows at once that the tap started something."""
    def test_measuring_while_the_corners_are_probed(self):
        obj, gcode, tilt = self.build([[0., -0.3, 0., 0.]])
        self.tram_cmd(obj)
        self.assertEqual(obj.state, tram.AWAITING_MOUNT)
        seen = []
        measure = tilt.measure
        def spy():
            seen.append(obj.get_status(0.)['state'])
            seen.append(obj.get_status(0.)['busy'])
            measure()
        tilt.measure = spy
        self.tram_cmd(obj, MOUNT='rigid')
        self.assertEqual(seen, [tram.MEASURING, True])
        self.assertEqual(obj.state, tram.AWAITING_ADJUSTMENT)

    def test_also_when_measuring_again(self):
        obj, gcode, tilt = self.build(
            [[0., -0.3, 0., 0.], [0., 0., 0., 0.]], mount='rigid')
        self.tram_cmd(obj)
        seen = []
        measure = tilt.measure
        tilt.measure = lambda: (seen.append(obj.state), measure())
        self.tram_cmd(obj, CONFIRM=1)
        self.assertEqual(seen, [tram.MEASURING])
        self.assertEqual(obj.state, tram.COMPLETE)

    def test_error_after_a_failed_measurement(self):
        obj, gcode, tilt = self.build([[0., None, 0., 0.]], mount='rigid')
        def incomplete():
            tilt.calls += 1
            tilt.results = {'screw1': {'z': 0.}}
        tilt.measure = incomplete
        with self.assertRaises(Exception):
            self.tram_cmd(obj)
        self.assertEqual(obj.state, tram.ERROR)


class ToleranceRetriesTest(TramTestBase):
    """1.0.4: a noisy corner is probed again (upstream
    SAMPLES_TOLERANCE_RETRIES) instead of failing the measurement."""
    def measure_script(self, obj, gcode):
        self.tram_cmd(obj)
        return [s for s in gcode.scripts_run
                if s.startswith('SCREWS_TILT_CALCULATE')][0]

    def test_three_retries_by_default(self):
        obj, gcode, tilt = self.build([[0., -0.3, 0., 0.]], mount='rigid')
        self.assertEqual(self.measure_script(obj, gcode),
                         "SCREWS_TILT_CALCULATE SAMPLES=3 SAMPLES_RESULT=median"
                         " SAMPLES_TOLERANCE_RETRIES=3")

    def test_configurable(self):
        obj, gcode, tilt = self.build([[0., -0.3, 0., 0.]], mount='rigid',
                                      tolerance_retries=0)
        self.assertTrue(self.measure_script(obj, gcode).endswith(
            " SAMPLES_TOLERANCE_RETRIES=0"))
