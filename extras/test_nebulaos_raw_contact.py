# Offline tests for nebulaos_raw_contact (G83b stages A/B).
#
# - idle statistics and the no-motion health check (healthy, stuck,
#   saturated, sensor errors, sample-rate mismatch, too few, non-finite)
# - polarity normalisation; calibrated mode keeps using grams
# - the klippy.log trace parser and the grams/raw-count replay pairing
# - upstream LCBestFit (the pinned source, class extracted and run) gives
#   the same contact Z from raw counts as from grams on the same samples,
#   and is invariant to the sensor gain
# - the qualified-profile registry is empty: raw contact refuses to move
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import ast
import types
import math
import os
import random
import sys
import unittest

from . import nebulaos_raw_contact as rc

HERE = os.path.dirname(os.path.abspath(__file__))


def _klipper_dir():
    for d in (os.environ.get('NEBULAOS_KLIPPER_DIR'),
              os.path.join(HERE, '..', '..', 'NebulaOS-firmware', 'vendor', 'klipper'),
              os.path.join(HERE, '..', '..', '..', 'klipper')):
        if d and os.path.isfile(os.path.join(d, 'klippy', 'extras', 'load_cell_probe.py')):
            return os.path.abspath(d)
    return None


def _upstream_lcbestfit():
    """The pinned upstream LCBestFit class, extracted from load_cell_probe.py
    and executed with upstream's own mathutil (importing the whole module
    would pull in the MCU/sensor stack)."""
    kdir = _klipper_dir()
    if kdir is None:
        return None
    src_path = os.path.join(kdir, 'klippy', 'extras', 'load_cell_probe.py')
    with open(src_path) as fh:
        src = fh.read()
    tree = ast.parse(src)
    node = next(n for n in tree.body
                if isinstance(n, ast.ClassDef) and n.name == 'LCBestFit')
    sys.path.insert(0, os.path.join(kdir, 'klippy'))
    try:
        import mathutil
    finally:
        sys.path.pop(0)
    ns = {'mathutil': mathutil, 'sys': sys, 'math': math}
    exec(compile(ast.Module(body=[node], type_ignores=[]), src_path, 'exec'), ns)
    return ns['LCBestFit']


def _contact_trace(gain_counts_per_gram=85.72, polarity=-1, noise_g=1.5,
                   z_contact=0.137, tare=-249399, seed=7):
    """A synthetic ascent like the probe records: Z rises from below the
    contact plane; below it the bed is compressed (force falls linearly to
    zero at contact), above it only noise. Returns samples
    (time, grams, counts, tare) and the matching (grams, z) window pairs."""
    rnd = random.Random(seed)
    samples, pairs = [], []
    t0, z0, speed, sps = 100.0, z_contact - 0.12, 10.0 / 60. * 6, 80.
    for i in range(24):
        t = t0 + i / sps
        z = z0 + speed * (i / sps)
        grams = max(0., (z_contact - z) * 4000.) + rnd.gauss(0, noise_g)
        counts = tare + polarity * int(round(grams * gain_counts_per_gram))
        samples.append((t, grams, counts, tare))
        pairs.append((grams, z))
    return {'ascent_start': t0, 'window': 0.3, 'samples': samples,
            'window_pairs': pairs}


class IdleCheckTest(unittest.TestCase):
    def _idle(self, n=160, sps=80., sigma=40., base=-249399, seed=3):
        rnd = random.Random(seed)
        return [(10. + i / sps, None, base + int(rnd.gauss(0, sigma)), base)
                for i in range(n)]

    def test_healthy_idle_window(self):
        stats, reasons = rc.validate_idle(self._idle(), None,
                                          (-0x800000, 0x7FFFFF), 80.)
        self.assertEqual(reasons, [])
        self.assertAlmostEqual(stats['sps'], 80., places=3)
        self.assertTrue(20. < stats['sigma'] < 80.)
        self.assertAlmostEqual(stats['median'], -249399, delta=20)

    def test_stuck_sensor_is_not_healthy(self):
        samples = [(10. + i / 80., None, -249399, -249399) for i in range(160)]
        stats, reasons = rc.validate_idle(samples, None, (-0x800000, 0x7FFFFF), 80.)
        self.assertIn(rc.STUCK, reasons)
        self.assertEqual(stats['sigma'], 0.)

    def test_saturated_errors_rate_and_count(self):
        samples = self._idle()
        samples[5] = (samples[5][0], None, 0x7FFFFF, samples[5][3])
        _, reasons = rc.validate_idle(samples, (2, 0), (-0x800000, 0x7FFFFF), 10.)
        self.assertIn(rc.SATURATED, reasons)
        self.assertIn(rc.SENSOR_ERRORS, reasons)
        self.assertIn(rc.SAMPLE_RATE_MISMATCH, reasons)
        _, reasons = rc.validate_idle(self._idle(n=5), None, None, None)
        self.assertIn(rc.TOO_FEW_SAMPLES, reasons)

    def test_no_samples_and_non_finite(self):
        _, reasons = rc.validate_idle([], None)
        self.assertIn(rc.NO_SAMPLES, reasons)
        bad = self._idle()
        bad[3] = (bad[3][0], None, float('nan'), bad[3][3])
        _, reasons = rc.validate_idle(bad, None)
        self.assertIn(rc.NON_FINITE, reasons)

    def test_drift_is_measured(self):
        samples = [(10. + i / 80., None, -249399 + 50 * i, -249399) for i in range(160)]
        stats, _ = rc.validate_idle(samples, None)
        self.assertAlmostEqual(stats['drift_per_s'], 50 * 80., delta=1.)


class PolarityTest(unittest.TestCase):
    def test_compression_positive_for_both_polarities(self):
        self.assertEqual(rc.compression_signal(-260000, -250000, -1), 10000)
        self.assertEqual(rc.compression_signal(-240000, -250000, 1), 10000)
        with self.assertRaises(ValueError):
            rc.compression_signal(1, 0, 0)

    def test_fit_magnitude_modes(self):
        s = (1.0, 116.6, -259399, -249399)
        self.assertEqual(rc.fit_magnitude(s, rc.MODE_CALIBRATED), 116.6)
        self.assertEqual(rc.fit_magnitude(s, rc.MODE_RAW, -1), 10000)
        # calibrated mode never touches the raw columns; raw mode never
        # needs grams (None on an uncalibrated cell)
        self.assertEqual(rc.fit_magnitude((1.0, None, -259399, -249399),
                                          rc.MODE_RAW, -1), 10000)


class TraceReplayTest(unittest.TestCase):
    def _log(self, trace):
        name = 'nebulaos_z_offset_probe'
        lines = ["2026-10-18 random line",
                 "%s: raw collector samples (%d, ascent_start=%.4f window=%.4f): %s"
                 % (name, len(trace['samples']), trace['ascent_start'],
                    trace['window'], [list(s) for s in trace['samples']]),
                 "%s: ascent fit window data (%d points, grams/z pairs): %s"
                 % (name, len(trace['window_pairs']), trace['window_pairs'])]
        return "\n".join(lines)

    def test_parser_reads_the_probe_log_format(self):
        trace = _contact_trace()
        parsed = rc.parse_contact_traces(self._log(trace) + "\n" + self._log(trace))
        self.assertEqual(len(parsed), 2)
        self.assertEqual(len(parsed[0]['samples']), 24)
        self.assertAlmostEqual(parsed[0]['ascent_start'], trace['ascent_start'], places=4)

    def test_replay_pairs_share_z_and_normalise_sign(self):
        trace = _contact_trace()
        grams_pairs, count_pairs = rc.replay_fit_pairs(trace, -1)
        self.assertEqual([p[1] for p in grams_pairs], [p[1] for p in count_pairs])
        # compression is positive in both domains
        self.assertGreater(count_pairs[0][0], 0)
        self.assertGreater(grams_pairs[0][0], 0)

    def test_mismatched_trace_is_rejected(self):
        trace = _contact_trace()
        trace['window_pairs'] = trace['window_pairs'][:-2]
        with self.assertRaises(ValueError):
            rc.replay_fit_pairs(trace, -1)


@unittest.skipIf(_klipper_dir() is None, "pinned Klipper source not found")
class UpstreamFitEquivalenceTest(unittest.TestCase):
    def setUp(self):
        self.fit = _upstream_lcbestfit()(printer=None)

    def test_raw_counts_give_the_same_contact_z_as_grams(self):
        trace = _contact_trace()
        grams_pairs, count_pairs = rc.replay_fit_pairs(trace, -1)
        z_g, below_g, above_g, _ = self.fit.find_best_fit(grams_pairs)
        z_c, below_c, above_c, _ = self.fit.find_best_fit(count_pairs)
        self.assertAlmostEqual(z_g, z_c, delta=0.001)
        self.assertEqual((below_g, above_g), (below_c, above_c))
        self.assertAlmostEqual(z_g, 0.137, delta=0.01)

    def test_fit_is_invariant_to_sensor_gain_and_polarity(self):
        zs = []
        for gain, pol in ((40., -1), (85.72, -1), (200., 1)):
            trace = _contact_trace(gain_counts_per_gram=gain, polarity=pol)
            zs.append(self.fit.find_best_fit(rc.replay_fit_pairs(trace, pol)[1])[0])
        self.assertLess(max(zs) - min(zs), 0.002)


REAL_TRACES = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'testdata',
                           'ke-dev-contact-traces-20261019.log')


@unittest.skipIf(_klipper_dir() is None, "pinned Klipper source not found")
class RealTraceReplayTest(unittest.TestCase):
    """Stage A on real data: the 9 contacts of a ke-dev Z-offset run
    (2026-10-19). The fit on sign-normalised raw counts must give the same
    contact Z as the calibrated grams fit on the same samples."""

    def setUp(self):
        self.fit = _upstream_lcbestfit()(printer=None)
        with open(REAL_TRACES) as handle:
            self.traces = rc.parse_contact_traces(handle.read())

    def test_all_contacts_parsed(self):
        self.assertEqual(len(self.traces), 9)

    def test_compression_lowers_the_counts_on_ke_dev(self):
        # Nozzle contact presses the bed down on the front-left load cell:
        # the raw counts FALL (polarity -1), by 15000-25000 counts (about
        # 180-290 g with this unit's calibration) at the peak of each touch.
        # A known weight on the front-left screw also reads negative
        # (Tim, 2026-10-19: 1531 g bottle -> -1695 g). The small positive
        # deltas in these traces are rebound/noise, not the contact.
        # Measured, not assumed.
        for trace in self.traces:
            deltas = [s[2] - s[3] for s in trace['samples']]
            self.assertLess(min(deltas), -15000)
            self.assertLess(max(deltas), 1500)
            self.assertGreater(-min(deltas), 10 * max(deltas))

    def test_raw_counts_give_the_same_contact_z_as_grams(self):
        for trace in self.traces:
            grams_pairs, count_pairs = rc.replay_fit_pairs(trace, -1)
            z_g, below_g, above_g, _ = self.fit.find_best_fit(grams_pairs)
            z_c, below_c, above_c, _ = self.fit.find_best_fit(count_pairs)
            self.assertAlmostEqual(z_g, z_c, delta=0.0005)
            self.assertEqual((below_g, above_g), (below_c, above_c))


class RealTraceTriggerReplayTest(unittest.TestCase):
    """G83b-auto step 3: the 9 ke-dev contacts through the raw trigger rule
    (abs_ge on tare-relative counts, 6400 = 75 g at this unit's 85.72 c/g).
    Evidence for stage C only - it qualifies nothing."""
    TRIGGER = 6400

    def setUp(self):
        with open(REAL_TRACES) as handle:
            self.traces = rc.parse_contact_traces(handle.read())

    def test_every_contact_crosses_the_trigger_before_its_peak(self):
        for trace in self.traces:
            deltas = [s[2] - s[3] for s in trace['samples']]
            cross = next(i for i, d in enumerate(deltas)
                         if abs(d) >= self.TRIGGER)
            peak = max(range(len(deltas)), key=lambda i: abs(deltas[i]))
            self.assertLess(cross, peak)

    def test_idle_before_contact_stays_far_below_the_trigger(self):
        # first 0.5 s of every trace (before the nozzle reaches the bed):
        # measured max 472 counts - more than 10x below the trigger
        for trace in self.traces:
            t0 = trace['samples'][0][0]
            idle = [abs(s[2] - s[3]) for s in trace['samples']
                    if s[0] - t0 < 0.5]
            self.assertLess(max(idle) * 10, self.TRIGGER)


class QualifiedProfileGateTest(unittest.TestCase):
    def test_no_profile_is_qualified_yet(self):
        self.assertEqual(rc.QUALIFIED_PROFILES, {})
        self.assertIsNone(rc.lookup_profile('ke-v1'))

    def test_raw_motion_is_refused_without_a_qualified_profile(self):
        self.assertEqual(rc.raw_motion_refusal(None)[0], rc.RAW_NO_PROFILE)
        self.assertEqual(rc.raw_motion_refusal('ke-v1')[0], rc.RAW_UNKNOWN_PROFILE)



def _probe_function(name):
    """Source of one method of nebulaos_z_offset_probe.ZOffsetProbe (the
    module needs the MCU stack to import, so its source is checked)."""
    with open(os.path.join(HERE, 'nebulaos_z_offset_probe.py')) as fh:
        src = fh.read()
    tree = ast.parse(src)
    cls = next(n for n in tree.body
               if isinstance(n, ast.ClassDef) and n.name == 'ZOffsetProbe')
    fn = next(n for n in cls.body
              if isinstance(n, ast.FunctionDef) and n.name == name)
    return ast.get_source_segment(src, fn), fn, src


class ProbeIntegrationSourceTest(unittest.TestCase):
    def test_diagnostic_never_moves_tares_or_saves(self):
        code, _, _ = _probe_function('cmd_raw_contact_diagnostic')
        for forbidden in ('probing_move', 'manual_move', 'SAVE_CONFIG',
                          '.tare(', 'set_offset_scale', 'set_trigger',
                          'set_raw_range', 'configfile', 'run_script'):
            self.assertNotIn(forbidden, code, forbidden)
        self.assertIn('validate_idle', code)

    def test_raw_mode_refuses_before_anything_is_armed(self):
        # Config raw_contact without an explicit plan refuses before the MCU
        # is armed; an explicit raw plan goes to _arm_raw (tested below).
        code, fn, _ = _probe_function('_tare_and_arm')
        self.assertIn('MODE_RAW', code)
        self.assertLess(code.index('raw_motion_refusal'),
                        code.index('set_raw_range'))
        self.assertLess(code.index('_arm_raw('), code.index('set_raw_range'))

    def test_default_mode_is_auto(self):
        # G83b-auto (Run 14): 'auto' - with QUALIFIED_PROFILES empty it is
        # the calibrated behaviour (see ResolveArmingTest).
        _, _, src = _probe_function('__init__')
        self.assertIn("'contact_detection_mode'", src)
        self.assertIn("raw_contact.MODE_AUTO)", src)

    def test_fit_uses_mode_aware_magnitude_and_no_fake_grams(self):
        code, _, _ = _probe_function('_fit_contact_z')
        self.assertIn('raw_contact.fit_magnitude(', code)
        # raw mode reports counts, never grams
        self.assertIn('_last_raw_peak_delta_counts', code)

    def test_status_keeps_is_calibrated_for_grams_only(self):
        code, _, _ = _probe_function('get_status')
        self.assertIn("'is_calibrated': self._load_cell.is_calibrated()", code)
        self.assertIn("'contact_qualified': kind is not None", code)
        self.assertIn("'effective_contact_mode'", code)


def _profile(**kw):
    p = {'id': 'test', 'polarity': -1, 'trigger_counts': 6400,
         'safety_counts': 170000, 'max_reference_drift_counts': 6400,
         'expected_sps': 80., 'sps_tolerance': 0.25, 'max_idle_sigma': 640.,
         'max_idle_drift_per_s': 800., 'evidence': 'test'}
    p.update(kw)
    return p


class ProfileTest(unittest.TestCase):
    def test_complete_profile_is_valid(self):
        self.assertEqual(rc.validate_profile(_profile()), [])

    def test_missing_polarity_is_refused(self):
        p = _profile()
        del p['polarity']
        self.assertIn('missing polarity', rc.validate_profile(p))
        self.assertTrue(rc.validate_profile(_profile(polarity=0)))

    def test_trigger_must_be_below_safety(self):
        self.assertTrue(rc.validate_profile(_profile(trigger_counts=200000)))


class CandidateTest(unittest.TestCase):
    def test_candidate_within_the_calibrated_envelope(self):
        p, problems = rc.candidate_profile(6400, -1, 85.72, 75., 2000., 80.)
        self.assertEqual(problems, [])
        self.assertEqual(p['safety_counts'], int(85.72 * 2000))
        self.assertEqual(p['polarity'], -1)

    def test_candidate_above_trigger_force_is_refused(self):
        # 9000 counts = 105 g on ke-dev > 75 g + 10%
        p, problems = rc.candidate_profile(9000, -1, 85.72, 75., 2000., 80.)
        self.assertIsNone(p)
        self.assertIn('above trigger_force', problems[0])

    def test_candidate_needs_a_calibration(self):
        p, problems = rc.candidate_profile(6400, -1, None, 75., 2000., 80.)
        self.assertIsNone(p)

    def test_candidate_is_never_reachable_by_lookup(self):
        p, _ = rc.candidate_profile(6400, -1, 85.72, 75., 2000., 80.)
        self.assertIsNone(rc.lookup_profile(p['id']))


class ResolveArmingTest(unittest.TestCase):
    def tearDown(self):
        rc.QUALIFIED_PROFILES.clear()

    def test_auto_today_is_the_calibrated_behaviour(self):
        for caller in (rc.CALLER_Z_OFFSET_PAIR, 'nozzle_clear', 'legacy'):
            self.assertEqual(rc.resolve_arming('auto', caller, True),
                             (rc.ARM_CALIBRATED, None))
            kind, refusal = rc.resolve_arming('auto', caller, False)
            self.assertIsNone(kind)
            self.assertEqual(refusal[0], rc.NOT_CALIBRATED)

    def test_other_callers_never_get_raw_even_with_a_qualified_profile(self):
        rc.QUALIFIED_PROFILES['ke-test'] = _profile(id='ke-test')
        for mode in rc.MODES:
            for caller in ('nozzle_clear', 'legacy_z_offset', 'auto_calibrate'):
                kind, _ = rc.resolve_arming(mode, caller, False, 'ke-test', 0)
                self.assertNotEqual(kind, rc.ARM_RAW)

    def test_calibration_beats_a_model_profile(self):
        rc.QUALIFIED_PROFILES['ke-test'] = _profile(id='ke-test')
        self.assertEqual(rc.resolve_arming('auto', rc.CALLER_Z_OFFSET_PAIR,
                                           True, 'ke-test', 0)[0],
                         rc.ARM_CALIBRATED)

    def test_every_qualified_profile_passes_the_schema(self):
        for pid, profile in rc.QUALIFIED_PROFILES.items():
            self.assertEqual(rc.validate_profile(profile), [], pid)

    def test_an_invalid_registry_profile_is_refused(self):
        bad = _profile(id='ke-bad')
        del bad['polarity']
        rc.QUALIFIED_PROFILES['ke-bad'] = bad
        kind, refusal = rc.resolve_arming('raw_contact', rc.CALLER_Z_OFFSET_PAIR,
                                          False, 'ke-bad', 0)
        self.assertIsNone(kind)
        self.assertEqual(refusal[0], rc.RAW_BAD_PROFILE)

    def test_raw_needs_a_reference(self):
        rc.QUALIFIED_PROFILES['ke-test'] = _profile(id='ke-test')
        kind, refusal = rc.resolve_arming('auto', rc.CALLER_Z_OFFSET_PAIR,
                                          False, 'ke-test', None)
        self.assertEqual(refusal[0], rc.RAW_NO_REFERENCE)
        kind, profile = rc.resolve_arming('auto', rc.CALLER_Z_OFFSET_PAIR,
                                          False, 'ke-test', -240000)
        self.assertEqual(kind, rc.ARM_RAW)

def _bound_probe_methods(names):
    """Methods of ZOffsetProbe executed on a fake (no MCU stack needed)."""
    ns = {'raw_contact': rc, 'math': __import__('math')}
    for name in names:
        code, fn, _ = _probe_function(name)
        exec(compile(ast.Module(body=[fn], type_ignores=[]), 'probe', 'exec'),
             ns)
    return {n: ns[n] for n in names}


class _Err(Exception):
    pass


class _Printer:
    def __init__(self):
        self.reactor = types.SimpleNamespace(monotonic=lambda: 0.,
                                             pause=lambda t: t)
        self.toolhead = types.SimpleNamespace(get_last_move_time=lambda: 42.)

    def get_reactor(self):
        return self.reactor

    def lookup_object(self, name):
        return {'toolhead': self.toolhead}[name]

    def command_error(self, msg):
        return _Err(msg)


class _Collector:
    def __init__(self, samples, log):
        self.samples, self.log = samples, log

    def start_collecting(self, min_time=None):
        self.log.append(('start', min_time))

    def collect_until(self, print_time=None):
        self.log.append(('until', print_time))
        return self.samples, 0


class _LoadCell:
    def __init__(self, samples, reference, calibrated=True):
        self.samples, self.reference = samples, reference
        self.calibrated = calibrated
        self.tared = None
        self.collect_log = []

    def is_calibrated(self):
        return self.calibrated

    def get_reference_tare_counts(self):
        return self.reference

    def get_collector(self):
        return _Collector(self.samples, self.collect_log)

    def saturation_range(self):
        return -0x800000, 0x7FFFFF

    def tare(self, counts):
        self.tared = counts


class _Trigger:
    def __init__(self):
        self.calls = []
        sos = types.SimpleNamespace(
            set_offset_scale=lambda o, sc: self.calls.append(('offset_scale', o, sc)))
        self.get_sos_filter = lambda: sos

    def set_raw_range(self, lo, hi):
        self.calls.append(('raw_range', lo, hi))

    def set_trigger(self, kind, value):
        self.calls.append(('trigger', kind, value))


def _idle(median, n=70, sps=90.6, noise=110):
    out, seed = [], 7
    for i in range(n):
        seed = (seed * 1103515245 + 12345) & 0x7fffffff
        out.append((i / sps, None, median + (seed % (2 * noise + 1)) - noise,
                    None))
    return out


class RawArmingTest(unittest.TestCase):
    """_arm_raw() per the Run 14 architecture review."""
    REF = -240426

    def _probe(self, samples, reference=REF, calibrated=True):
        m = _bound_probe_methods(['_arm_raw', '_raw_reference_counts',
                                  '_refuse'])
        probe = types.SimpleNamespace()
        probe._printer = _Printer()
        probe._name = 'nebulaos_z_offset_probe'
        probe._load_cell = _LoadCell(samples, reference, calibrated)
        probe._sensor = types.SimpleNamespace(
            get_range=lambda: (-0x800000, 0x7FFFFF))
        probe._mcu_trigger_analog = _Trigger()
        probe._raw_health_time = 0.75
        probe._raw_diag = None
        probe._last_contact_rejection_reason = None
        probe._active_profile = None
        probe._effective_mode = None
        for name, fn in m.items():
            setattr(probe, name, types.MethodType(fn, probe))
        return probe

    def test_arms_on_the_fresh_tare_with_the_window_on_the_reference(self):
        probe = self._probe(_idle(self.REF - 1000))
        probe._arm_raw(_profile())
        calls = probe._mcu_trigger_analog.calls
        self.assertEqual(calls[0], ('raw_range', self.REF - 170000,
                                    self.REF + 170000))
        self.assertEqual(calls[1][0], 'offset_scale')
        self.assertAlmostEqual(-calls[1][1], self.REF - 1000, delta=60)
        self.assertEqual(calls[1][2], 1)
        self.assertEqual(calls[2], ('trigger', 'abs_ge', 6400))
        self.assertEqual(probe._effective_mode, rc.MODE_RAW)
        self.assertEqual(probe._active_profile['polarity'], -1)

    def test_health_window_starts_after_queued_moves(self):
        probe = self._probe(_idle(self.REF))
        probe._arm_raw(_profile())
        self.assertEqual(probe._load_cell.collect_log,
                         [('start', 42.), ('until', 42.75)])

    def test_residual_preload_is_refused_not_re_tared(self):
        # Something presses on the bed: the empty reading is 20000 counts
        # off the reference. Re-taring would move the safety window.
        probe = self._probe(_idle(self.REF - 20000))
        with self.assertRaises(_Err):
            probe._arm_raw(_profile())
        self.assertEqual(probe._last_contact_rejection_reason,
                         rc.RAW_REFERENCE_DRIFT)
        self.assertEqual(probe._mcu_trigger_analog.calls, [])

    def test_short_window_is_refused(self):
        probe = self._probe(_idle(self.REF, n=6))      # the old touch tare
        with self.assertRaises(_Err):
            probe._arm_raw(_profile())
        self.assertEqual(probe._last_contact_rejection_reason, rc.RAW_UNHEALTHY)
        self.assertEqual(probe._mcu_trigger_analog.calls, [])

    def test_noisy_sensor_is_refused(self):
        probe = self._probe(_idle(self.REF, noise=3000))
        with self.assertRaises(_Err):
            probe._arm_raw(_profile())
        self.assertTrue(probe._raw_diag['reasons'][0].startswith(
            'noise_above_profile'))

    def test_no_reference_no_raw(self):
        probe = self._probe(_idle(self.REF), reference=None, calibrated=False)
        with self.assertRaises(_Err):
            probe._arm_raw(_profile())
        self.assertEqual(probe._last_contact_rejection_reason,
                         rc.RAW_NO_REFERENCE)


if __name__ == '__main__':
    unittest.main()
