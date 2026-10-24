# Tests for extras/nebulaos_load_cell_calibration.py (G86): the guided water
# bottle calibration. A fake upstream LoadCell delivers sample windows that
# look like ke-dev's sensor (90.6 SPS measured vs 80 configured, ~110 counts
# noise, a weight LOWERS the counts).
#
# Run from klippy/: python3 -m unittest extras.test_nebulaos_load_cell_calibration -v
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import unittest

from . import prtouch_test_support as fake
from . import nebulaos_load_cell_calibration as lcc

SPS = 90.6
TARE = -240426          # ke-dev's empty bed, 2026-10-19
CPG = 94.9              # ke-dev at the front-left screw (1531 g -> 145292)


def _samples(median, drift=0., noise=110, n=181, stuck=False, start=100.):
    out, seed = [], 12345
    for i in range(n):
        seed = (seed * 1103515245 + 12345) & 0x7fffffff
        jitter = 0 if stuck else (seed % (2 * noise + 1)) - noise
        t = start + i / SPS
        counts = int(median + drift * (i / SPS - (n - 1) / SPS / 2.) + jitter)
        out.append((t, None, counts, TARE))
    return out


class FakeSensor:
    def get_samples_per_second(self):
        return 80.


class FakeCollector:
    def __init__(self, lc):
        self.lc = lc

    def start_collecting(self, min_time=None):
        pass

    def stop_collecting(self):
        return self.lc.windows.pop(0), 0


class FakeLoadCell:
    def __init__(self, windows=(), calibrated=False, counts_per_gram=None,
                 reference_tare=None, avg=None):
        self.windows = list(windows)
        self.calibrated = calibrated
        self.counts_per_gram = counts_per_gram
        self.reference_tare = reference_tare
        self.avg = avg
        self.set_calls = []
        self.tare_calls = []

    def get_collector(self):
        return FakeCollector(self)

    def get_sensor(self):
        return FakeSensor()

    def saturation_range(self):
        return -0x800000, 0x7FFFFF

    def counts_to_percent(self, counts):
        return counts / 0x7FFFFF * 100.

    def is_calibrated(self):
        return self.calibrated

    def get_counts_per_gram(self):
        return self.counts_per_gram

    def get_reference_tare_counts(self):
        return self.reference_tare

    def get_tare_counts(self):
        return self.reference_tare

    def avg_counts(self):
        return self.avg

    def set_calibration(self, cpg, tare):
        self.set_calls.append((cpg, tare))

    def tare(self, counts):
        self.tare_calls.append(counts)


class FakeProbe:
    def __init__(self, lc):
        self.lc = lc

    def get_load_cell(self):
        return self.lc


class FakePrintStats:
    def __init__(self, state='standby'):
        self.state = state

    def get_status(self, eventtime):
        return {'state': self.state}


class PromptGCode(fake.FakeGCode):
    def __init__(self):
        super().__init__()
        self.raw = []
        self.info = []

    def respond_raw(self, msg):
        self.raw.append(msg)

    def respond_info(self, msg):
        self.info.append(msg)

    def last_prompt(self):
        """(text lines, [(label, command)]) of the last prompt shown."""
        start = max(i for i, m in enumerate(self.raw)
                    if m.startswith('// action:prompt_begin'))
        lines, buttons = [], []
        for m in self.raw[start:]:
            if m.startswith('// action:prompt_text '):
                lines.append(m[len('// action:prompt_text '):])
            elif m.startswith('// action:prompt_button '):
                label, cmd = m[len('// action:prompt_button '):].split('|')
                buttons.append((label, cmd))
        return lines, buttons


def _build(lc, print_state='standby', **cfg):
    printer = fake.FakePrinter()
    gcode = PromptGCode()
    printer.add_object('gcode', gcode)
    printer.add_object('nebulaos_z_offset_probe', FakeProbe(lc))
    printer.add_object('print_stats', FakePrintStats(print_state))
    config = fake.FakeConfig({k: str(v) for k, v in cfg.items()},
                             section='nebulaos_load_cell_calibration',
                             printer=printer)
    obj = lcc.NebulaOSLoadCellCalibration(config)
    config.assert_all_consumed()
    return printer, gcode, obj


def _cmd(obj, **params):
    obj.cmd_calibrate(fake.FakeGCmd({k: str(v) for k, v in params.items()}))


def _weight(grams, cpg=CPG):
    return TARE - int(round(grams * cpg))      # a weight LOWERS the counts


def _up_to_weight(lc_windows, **kw):
    lc = FakeLoadCell([_samples(TARE)] + list(lc_windows), **kw)
    printer, gcode, obj = _build(lc)
    _cmd(obj)
    _cmd(obj, CHECK=1)
    return lc, gcode, obj


class HappyPathTest(unittest.TestCase):
    def test_bottle_flow_end_to_end(self):
        lc = FakeLoadCell(
            [_samples(TARE),                        # 1 empty bed
             _samples(_weight(1531)),               # 2 bottle on
             _samples(_weight(1520)),               # 3 put back (-0.7%)
             _samples(TARE - 500),                  # 4 removed (~5 g)
             _samples(TARE - 30000)],               # 5 finger (~316 g)
            calibrated=True, counts_per_gram=85.72, reference_tare=TARE + 3000)
        printer, gcode, obj = _build(lc)
        _cmd(obj)
        self.assertEqual(obj.state, lcc.AWAITING_EMPTY)
        lines, buttons = gcode.last_prompt()
        self.assertIn('Step 1 of 5', lines[0])
        self.assertIn(('Cancel', lcc.CANCEL), buttons)

        _cmd(obj, CHECK=1)
        self.assertEqual(obj.state, lcc.AWAITING_WEIGHT)
        self.assertEqual(obj.tare_counts // 100, TARE // 100)
        lines, buttons = gcode.last_prompt()
        self.assertTrue(any('front-left screw head' in l for l in lines))
        self.assertTrue(any('Weight: 1500 g' in l for l in lines))
        self.assertIn(('+10', lcc.CMD + ' ADJUST=10'), buttons)

        for delta in (10, 10, 10, 1):               # 1500 -> 1531, no typing
            _cmd(obj, ADJUST=delta)
        self.assertEqual(obj.pending_grams, 1531)
        _cmd(obj, WEIGH=1)
        self.assertEqual(obj.state, lcc.AWAITING_REPLACE)
        self.assertAlmostEqual(obj.counts_per_gram, CPG, delta=0.5)

        _cmd(obj, NEXT=1)
        self.assertEqual(obj.state, lcc.AWAITING_REMOVE)
        lines, _ = gcode.last_prompt()
        self.assertTrue(any('your scale said 1531 g' in l for l in lines))

        _cmd(obj, NEXT=1)
        self.assertEqual(obj.state, lcc.AWAITING_FINGER)
        _cmd(obj, MEASURE=1)
        self.assertAlmostEqual(obj.finger_grams, 316, delta=5)
        self.assertTrue(any('Printer measures 316 g' in l
                            for l in gcode.last_prompt()[0]))

        _cmd(obj, NEXT=1)
        self.assertEqual(obj.state, lcc.AWAITING_CONFIRMATION)
        self.assertAlmostEqual(obj.change_percent, 10.7, delta=1.)
        self.assertEqual(lc.set_calls, [])           # nothing saved yet
        _cmd(obj, ACCEPT=1)
        self.assertEqual(obj.state, lcc.COMPLETE)
        self.assertEqual(len(lc.set_calls), 1)
        self.assertAlmostEqual(lc.set_calls[0][0], CPG, delta=0.5)
        self.assertEqual(gcode.scripts_run[-1], 'SAVE_CONFIG')

    def test_finger_test_can_be_skipped(self):
        lc, gcode, obj = _up_to_weight([_samples(_weight(1500)),
                                        _samples(_weight(1500)),
                                        _samples(TARE)])
        _cmd(obj, WEIGH=1)
        _cmd(obj, NEXT=1)
        _cmd(obj, NEXT=1)
        _cmd(obj, NEXT=1)                            # skip the finger test
        self.assertEqual(obj.state, lcc.AWAITING_CONFIRMATION)

    def test_screen_number_entry(self):
        lc, gcode, obj = _up_to_weight([_samples(_weight(1531))])
        _cmd(obj, GRAMS=1531)
        self.assertEqual(obj.state, lcc.AWAITING_REPLACE)
        self.assertEqual(obj.grams, 1531)


class MistakesTest(unittest.TestCase):
    def test_typo_too_low_is_refused_without_reading(self):
        # 150 instead of 1500 would mean 10x the real touch force.
        lc, gcode, obj = _up_to_weight([])
        _cmd(obj, GRAMS=150)
        self.assertEqual(obj.state, lcc.AWAITING_WEIGHT)
        self.assertIn('between 1000 and 5000', obj.problem)
        self.assertTrue(gcode.last_prompt()[0][0].startswith('Problem:'))

    def test_typo_too_high_is_refused(self):
        lc, gcode, obj = _up_to_weight([])
        _cmd(obj, GRAMS=15310)
        self.assertEqual(obj.state, lcc.AWAITING_WEIGHT)

    def test_adjust_stays_in_range(self):
        lc, gcode, obj = _up_to_weight([])
        for _ in range(10):
            _cmd(obj, ADJUST=-100)
        self.assertEqual(obj.pending_grams, 1000)
        for _ in range(50):
            _cmd(obj, ADJUST=100)
        self.assertEqual(obj.pending_grams, 5000)

    def test_too_light_suggests_two_bottles_and_stays(self):
        # 800 g of reading = 0.9% of full scale
        lc, gcode, obj = _up_to_weight([_samples(_weight(800)),
                                        _samples(_weight(1531))])
        _cmd(obj, WEIGH=1)
        self.assertEqual(obj.state, lcc.AWAITING_WEIGHT)
        self.assertIn('two bottles', obj.problem)
        _cmd(obj, WEIGH=1)                           # retry in the same step
        self.assertEqual(obj.state, lcc.AWAITING_REPLACE)

    def test_bottle_off_the_screw_reads_low_which_is_the_safe_side(self):
        # Measured on ke-dev: rough placement read ~20% low. The flow still
        # passes; the lower counts/g makes the printer stop touches EARLIER.
        lc, gcode, obj = _up_to_weight([_samples(_weight(1531 * 0.8))])
        _cmd(obj, GRAMS=1531)
        self.assertEqual(obj.state, lcc.AWAITING_REPLACE)
        self.assertLess(obj.counts_per_gram, CPG)

    def test_bottle_not_on_the_bed_is_reported(self):
        lc, gcode, obj = _up_to_weight([_samples(TARE)])
        _cmd(obj, WEIGH=1)
        self.assertEqual(obj.state, lcc.AWAITING_WEIGHT)
        self.assertTrue('barely felt' in obj.problem
                        or 'did not feel' in obj.problem)

    def test_saturated_says_too_heavy(self):
        lc, gcode, obj = _up_to_weight([_samples(-0x800000, noise=0)])
        _cmd(obj, WEIGH=1)
        self.assertEqual(obj.state, lcc.AWAITING_WEIGHT)
        self.assertIn('too heavy', obj.problem)

    def test_replace_mismatch_stays_until_it_agrees(self):
        lc, gcode, obj = _up_to_weight([_samples(_weight(1531)),
                                        _samples(_weight(1300)),   # -15%
                                        _samples(_weight(1525))])
        _cmd(obj, GRAMS=1531)
        _cmd(obj, NEXT=1)
        self.assertEqual(obj.state, lcc.AWAITING_REPLACE)
        self.assertIn('your scale said 1531 g', obj.problem)
        _cmd(obj, NEXT=1)
        self.assertEqual(obj.state, lcc.AWAITING_REMOVE)

    def test_not_back_to_zero_stays(self):
        lc, gcode, obj = _up_to_weight([_samples(_weight(1531)),
                                        _samples(_weight(1531)),
                                        _samples(_weight(60)),     # 60 g left
                                        _samples(TARE)])
        _cmd(obj, WEIGH=1)
        _cmd(obj, NEXT=1)
        _cmd(obj, NEXT=1)
        self.assertEqual(obj.state, lcc.AWAITING_REMOVE)
        self.assertIn('does not return to zero', obj.problem)
        _cmd(obj, NEXT=1)
        self.assertEqual(obj.state, lcc.AWAITING_FINGER)

    def test_unstable_reading_stays(self):
        lc = FakeLoadCell([_samples(TARE, drift=2000.), _samples(TARE)])
        printer, gcode, obj = _build(lc)
        _cmd(obj)
        _cmd(obj, CHECK=1)
        self.assertEqual(obj.state, lcc.AWAITING_EMPTY)
        self.assertIn('not stable', obj.problem)
        _cmd(obj, CHECK=1)
        self.assertEqual(obj.state, lcc.AWAITING_WEIGHT)

    def test_unhealthy_sensor_is_explained(self):
        lc = FakeLoadCell([_samples(TARE, stuck=True)])
        printer, gcode, obj = _build(lc)
        _cmd(obj)
        _cmd(obj, CHECK=1)
        self.assertEqual(obj.state, lcc.AWAITING_EMPTY)
        self.assertIn('not working properly', obj.problem)
        self.assertIn('does not change', obj.problem)

    def test_empty_bed_far_from_saved_zero_is_mentioned(self):
        lc = FakeLoadCell([_samples(TARE)], calibrated=True,
                          counts_per_gram=85.72, reference_tare=TARE + 20000)
        printer, gcode, obj = _build(lc)
        _cmd(obj)
        _cmd(obj, CHECK=1)
        self.assertEqual(obj.state, lcc.AWAITING_WEIGHT)
        self.assertTrue(any('away from the last saved zero' in m
                            for m in gcode.info))

    def test_very_different_result_is_flagged_before_saving(self):
        lc = FakeLoadCell([_samples(TARE), _samples(_weight(1531)),
                           _samples(_weight(1531)), _samples(TARE)],
                          calibrated=True, counts_per_gram=60.,
                          reference_tare=TARE)
        printer, gcode, obj = _build(lc)
        _cmd(obj)
        _cmd(obj, CHECK=1)
        _cmd(obj, GRAMS=1531)
        _cmd(obj, NEXT=1)
        _cmd(obj, NEXT=1)
        _cmd(obj, NEXT=1)
        lines, buttons = gcode.last_prompt()
        self.assertTrue(any('very different' in l for l in lines))
        self.assertTrue(obj.get_status(0.)['change_warning'])
        self.assertIn(('Apply & Save', lcc.CMD + ' ACCEPT=1'), buttons)


class SessionTest(unittest.TestCase):
    def test_cancel_changes_nothing(self):
        lc, gcode, obj = _up_to_weight([_samples(_weight(1531))])
        _cmd(obj, WEIGH=1)
        obj.cmd_cancel(fake.FakeGCmd({}))
        self.assertEqual(obj.state, lcc.CANCELLED)
        self.assertEqual(lc.set_calls, [])
        self.assertNotIn('SAVE_CONFIG', gcode.scripts_run)

    def test_refused_while_printing(self):
        lc = FakeLoadCell([])
        printer, gcode, obj = _build(lc, print_state='printing')
        with self.assertRaises(fake.CommandError):
            _cmd(obj)
        self.assertEqual(obj.state, lcc.IDLE)

    def test_start_while_active_shows_the_current_step(self):
        # Second interface / reconnect: no second session, same step.
        lc, gcode, obj = _up_to_weight([])
        run = obj.run_id
        _cmd(obj)
        self.assertEqual(obj.state, lcc.AWAITING_WEIGHT)
        self.assertEqual(obj.run_id, run)
        self.assertTrue(any('Step 2 of 5' in l for l in gcode.last_prompt()[0]))

    def test_steps_out_of_order_are_refused(self):
        lc = FakeLoadCell([])
        printer, gcode, obj = _build(lc)
        for params in ({'CHECK': 1}, {'WEIGH': 1}, {'NEXT': 1},
                       {'MEASURE': 1}, {'ACCEPT': 1}, {'ADJUST': 10}):
            with self.assertRaises(fake.CommandError):
                _cmd(obj, **params)
        self.assertEqual(lc.set_calls, [])

    def test_status_for_the_screen(self):
        lc, gcode, obj = _up_to_weight([])
        st = obj.get_status(0.)
        self.assertEqual(st['state'], lcc.AWAITING_WEIGHT)
        self.assertEqual(st['pending_grams'], 1500)
        self.assertEqual((st['min_grams'], st['max_grams']), (1000, 5000))


class BusyTest(unittest.TestCase):
    """G91: 'busy' while a guided flow runs or waits for the user."""
    def test_busy_through_the_flow_until_saved(self):
        lc = FakeLoadCell([_samples(TARE), _samples(_weight(1500)),
                           _samples(_weight(1500)), _samples(TARE)])
        printer, gcode, obj = _build(lc)
        self.assertFalse(obj.get_status(0.)['busy'])
        _cmd(obj)
        self.assertTrue(obj.get_status(0.)['busy'])
        for params in ({'CHECK': 1}, {'WEIGH': 1}, {'NEXT': 1}, {'NEXT': 1},
                       {'NEXT': 1}):
            _cmd(obj, **params)
            self.assertTrue(obj.get_status(0.)['busy'], obj.state)
        self.assertEqual(obj.state, lcc.AWAITING_CONFIRMATION)
        _cmd(obj, ACCEPT=1)
        self.assertEqual(obj.state, lcc.COMPLETE)
        self.assertFalse(obj.get_status(0.)['busy'])

    def test_not_busy_after_cancel(self):
        lc, gcode, obj = _up_to_weight([_samples(_weight(1531))])
        self.assertTrue(obj.get_status(0.)['busy'])
        obj.cmd_cancel(fake.FakeGCmd({}))
        self.assertFalse(obj.get_status(0.)['busy'])


class ReadCommandTest(unittest.TestCase):
    def test_read_needs_a_calibration(self):
        printer, gcode, obj = _build(FakeLoadCell())
        with self.assertRaises(fake.CommandError):
            obj.cmd_read(fake.FakeGCmd({}))

    def test_read_reports_grams_as_a_magnitude(self):
        lc = FakeLoadCell(calibrated=True, counts_per_gram=CPG,
                          reference_tare=TARE, avg=_weight(1531))
        printer, gcode, obj = _build(lc)
        gcmd = fake.FakeGCmd({})
        obj.cmd_read(gcmd)
        self.assertIn('1531 g', gcmd.responses[0])


# ----------------------------------------------------------------------
# G99: NEBULAOS_LOAD_CELL_MODE / NEBULAOS_LOAD_CELL_AUTO_CHECK + status
# ----------------------------------------------------------------------
class FakeModeProbe(FakeProbe):
    """The probe's G99 surface (implemented and tested in
    test_nebulaos_raw_contact.RawArmingTest)."""
    def __init__(self, lc, mode='auto', check_result=(True, '')):
        super().__init__(lc)
        self.mode = mode
        self.check = 'never'
        self.reason = ''
        self.ref = None
        self.check_result = check_result
        self.checks = 0

    def set_load_cell_mode(self, mode):
        self.mode = mode
        return ''

    def run_auto_check(self):
        self.checks += 1
        ok, reason = self.check_result
        self.check = 'ok' if ok else 'failed'
        self.reason = reason
        self.ref = TARE if ok else None
        return ok, reason

    def auto_status(self):
        return {'mode': self.mode, 'mode_choice': self.mode,
                'auto_check': self.check, 'auto_check_reason': self.reason,
                'auto_check_time': None, 'auto_ready': self.ref is not None,
                'auto_profile': 'ke-stock-v1',
                'auto_reference_counts': self.ref, 'mode_warning': None}


def _build_mode(lc=None, **kw):
    printer, gcode, obj = _build(lc or FakeLoadCell(),
                                 print_state=kw.pop('print_state', 'standby'))
    probe = FakeModeProbe(lc or obj._load_cell(), **kw)
    printer.add_object('nebulaos_z_offset_probe', probe)
    return printer, gcode, obj, probe


class ModeCommandTest(unittest.TestCase):
    def test_status_has_the_g99_fields(self):
        printer, gcode, obj, probe = _build_mode()
        st = obj.get_status(0.)
        self.assertEqual(st['mode'], 'auto')
        self.assertEqual(st['auto_check'], 'never')
        self.assertEqual(st['auto_check_reason'], '')
        self.assertIs(st['auto_ready'], False)

    def test_status_without_a_probe_is_none_not_an_error(self):
        printer, gcode, obj = _build(FakeLoadCell())
        st = obj.get_status(0.)        # FakeProbe has no auto_status
        self.assertIsNone(st['mode'])
        self.assertIsNone(st['auto_ready'])

    def test_set_and_report(self):
        printer, gcode, obj, probe = _build_mode()
        gcmd = fake.FakeGCmd({'MODE': 'Manual'})
        obj.cmd_mode(gcmd)
        self.assertEqual(probe.mode, 'manual')
        self.assertEqual(obj.get_status(0.)['mode'], 'manual')
        self.assertIn('MISSING', gcmd.responses[-1])      # no bottle yet
        gcmd = fake.FakeGCmd({})
        obj.cmd_mode(gcmd)
        self.assertIn('manual', gcmd.responses[-1])
        obj.cmd_mode(fake.FakeGCmd({'MODE': 'auto'}))
        self.assertEqual(probe.mode, 'auto')

    def test_bad_mode_is_refused(self):
        printer, gcode, obj, probe = _build_mode()
        with self.assertRaises(fake.CommandError):
            obj.cmd_mode(fake.FakeGCmd({'MODE': 'raw'}))
        self.assertEqual(probe.mode, 'auto')

    def test_auto_check_ok(self):
        printer, gcode, obj, probe = _build_mode()
        gcmd = fake.FakeGCmd({})
        obj.cmd_auto_check(gcmd)
        st = obj.get_status(0.)
        self.assertEqual((st['auto_check'], st['auto_ready']), ('ok', True))
        self.assertIn('passed', gcmd.responses[-1])

    def test_auto_check_failure_says_why_and_offers_manual(self):
        printer, gcode, obj, probe = _build_mode(
            check_result=(False, 'the sensor reading does not change at all'))
        with self.assertRaises(fake.CommandError) as ctx:
            obj.cmd_auto_check(fake.FakeGCmd({}))
        self.assertIn('does not change', str(ctx.exception))
        self.assertIn('Manual', str(ctx.exception))
        st = obj.get_status(0.)
        self.assertEqual((st['auto_check'], st['auto_ready']),
                         ('failed', False))
        self.assertIn('does not change', st['auto_check_reason'])

    def test_auto_check_refused_while_printing_or_during_the_bottle_flow(self):
        printer, gcode, obj, probe = _build_mode(print_state='printing')
        with self.assertRaises(fake.CommandError):
            obj.cmd_auto_check(fake.FakeGCmd({}))
        lc = FakeLoadCell([_samples(TARE)])
        printer, gcode, obj, probe = _build_mode(lc)
        _cmd(obj)
        with self.assertRaises(fake.CommandError):
            obj.cmd_auto_check(fake.FakeGCmd({}))
        self.assertEqual(probe.checks, 0)

    def test_saving_a_bottle_calibration_selects_manual(self):
        lc = FakeLoadCell([_samples(TARE), _samples(_weight(1500)),
                           _samples(_weight(1500)), _samples(TARE)])
        printer, gcode, obj, probe = _build_mode(lc)
        _cmd(obj)
        for params in ({'CHECK': 1}, {'WEIGH': 1}, {'NEXT': 1}, {'NEXT': 1},
                       {'NEXT': 1}):
            _cmd(obj, **params)
        self.assertEqual(probe.mode, 'auto')        # not before Apply & Save
        _cmd(obj, ACCEPT=1)
        self.assertEqual(probe.mode, 'manual')
        self.assertEqual(gcode.scripts_run[-1], 'SAVE_CONFIG')

    def test_cancelled_bottle_flow_keeps_the_mode(self):
        lc = FakeLoadCell([_samples(TARE)])
        printer, gcode, obj, probe = _build_mode(lc)
        _cmd(obj)
        obj.cmd_cancel(fake.FakeGCmd({}))
        self.assertEqual(probe.mode, 'auto')


if __name__ == '__main__':
    unittest.main()


class RecordingCalibration:
    def __init__(self):
        self.calls = []

    def record_flow(self, key, summary):
        self.calls.append((key, summary))


class CalibrationHistoryTest(unittest.TestCase):
    """G105: a new load cell result records 'load_cell' (clears its Redo)."""

    def test_passed_auto_check_records(self):
        printer, gcode, obj, probe = _build_mode()
        cal = RecordingCalibration()
        printer.add_object('nebulaos_calibration', cal)
        obj.cmd_auto_check(fake.FakeGCmd({}))
        self.assertEqual(cal.calls, [('load_cell', 'Automatic check passed')])

    def test_failed_auto_check_does_not_record(self):
        printer, gcode, obj, probe = _build_mode(
            check_result=(False, 'the reading does not change'))
        cal = RecordingCalibration()
        printer.add_object('nebulaos_calibration', cal)
        with self.assertRaises(fake.CommandError):
            obj.cmd_auto_check(fake.FakeGCmd({}))
        self.assertEqual(cal.calls, [])

    def test_bottle_apply_and_save_records_before_the_restart(self):
        lc, gcode, obj = _up_to_weight([_samples(_weight(1500)),
                                        _samples(_weight(1500)),
                                        _samples(TARE)])
        cal = RecordingCalibration()
        obj.printer.add_object('nebulaos_calibration', cal)
        order = []
        cal.record_flow = lambda k, s: order.append(('record', k, s))
        orig = gcode.run_script_from_command
        gcode.run_script_from_command = lambda s: (order.append(s), orig(s))
        _cmd(obj, WEIGH=1)
        _cmd(obj, NEXT=1)
        _cmd(obj, NEXT=1)
        _cmd(obj, NEXT=1)
        _cmd(obj, ACCEPT=1)
        self.assertEqual(order[-1], 'SAVE_CONFIG')
        self.assertEqual(order[-2][:2], ('record', 'load_cell'))
        self.assertIn('counts per gram', order[-2][2])
