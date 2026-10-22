# Offline tests for nebulaos_raw_contact (G83b stages A/B).
#
# - idle statistics and the no-motion health check (healthy, stuck,
#   saturated, sensor errors, sample-rate mismatch, too few, non-finite)
# - polarity normalisation; calibrated mode keeps using grams
# - the klippy.log trace parser and the grams/raw-count replay pairing
# - upstream LCBestFit (the pinned source, class extracted and run) gives
#   the same contact Z from raw counts as from grams on the same samples,
#   and is invariant to the sensor gain
# - the qualified-profile registry (G99: ke-stock-v1, Tim 2026-10-22) and
#   the per-mode arming rule, the same for every caller
# - raw arming on the fake probe: health window, settle loop, reference
#   drift, safety window; the automatic check (G99)
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


# realpath: on a printer this test is a symlink in klippy/extras (Run 14
# verifier note 3); the trace file stays next to the real source.
REAL_TRACES = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'testdata',
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
    def test_only_the_ke_stock_profile_is_qualified(self):
        # G99 (Tim 2026-10-22): ke-stock-v1 is the only registry entry
        self.assertEqual(sorted(rc.QUALIFIED_PROFILES), ['ke-stock-v1'])
        self.assertIsNone(rc.lookup_profile('ke-v1'))
        self.assertEqual(rc.DEFAULT_PROFILE_ID, 'ke-stock-v1')

    def test_ke_stock_v1_values(self):
        p = rc.lookup_profile('ke-stock-v1')
        self.assertEqual(rc.validate_profile(p), [])
        self.assertEqual((p['trigger_counts'], p['polarity']), (6400, -1))
        # the ke-dev compare candidate's windows (cpg 85.72084, 75 g, 2 kg)
        cand, _ = rc.candidate_profile(6400, -1, 85.72084, 75., 2000., 80.)
        for key in ('safety_counts', 'max_reference_drift_counts',
                    'max_idle_sigma', 'max_idle_drift_per_s', 'expected_sps',
                    'sps_tolerance'):
            self.assertEqual(p[key], cand[key], key)
        self.assertIn('Tim', p['evidence'])

    def test_ke_dev_idle_and_contacts_fit_the_profile(self):
        # measured: idle sigma 110 counts, 90.6 SPS; contact peaks
        # 15000-25000 counts - far below the 2 kg safety window
        p = rc.lookup_profile('ke-stock-v1')
        self.assertLess(110 * 10, p['trigger_counts'])
        self.assertLess(abs(90.6 - p['expected_sps']),
                        p['sps_tolerance'] * p['expected_sps'])
        self.assertLess(25000, p['safety_counts'])

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

    def test_every_touch_is_planned_before_anything_is_armed(self):
        # G99: a touch without a plan resolves one from the user's mode; a
        # refusal or a raw plan returns before the calibrated arming.
        code, fn, _ = _probe_function('_tare_and_arm')
        self.assertLess(code.index('self.plan_arming('),
                        code.index('set_raw_range'))
        self.assertLess(code.index('self._refuse('), code.index('set_raw_range'))
        self.assertLess(code.index('_arm_raw('), code.index('set_raw_range'))
        code, _, _ = _probe_function('touch_probe')
        self.assertLess(code.index('self.plan_arming('),
                        code.index('for i in range(pro_cnt)'))

    def test_default_mode_is_auto(self):
        # contact_detection_mode 'auto' = follow the user's choice, which
        # defaults to automatic (G99); the default profile is ke-stock-v1.
        _, _, src = _probe_function('__init__')
        self.assertIn("'contact_detection_mode'", src)
        self.assertIn("raw_contact.MODE_AUTO)", src)
        self.assertIn("raw_contact.DEFAULT_PROFILE_ID)", src)

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
    """G99: the user's mode decides, the same for every caller."""
    CALLERS = (rc.CALLER_Z_OFFSET_PAIR, rc.CALLER_AUTO_CALIBRATE,
               rc.CALLER_NOZZLE_CLEAN, rc.CALLER_TOUCH)
    KE = 'ke-stock-v1'

    def tearDown(self):
        for k in list(rc.QUALIFIED_PROFILES):
            if k != self.KE:
                del rc.QUALIFIED_PROFILES[k]

    def test_auto_is_raw_for_every_caller(self):
        for caller in self.CALLERS:
            for calibrated in (True, False):
                kind, profile = rc.resolve_arming('auto', caller, calibrated,
                                                  self.KE, -240000)
                self.assertEqual(kind, rc.ARM_RAW, caller)
                self.assertEqual(profile['id'], self.KE)

    def test_manual_is_the_calibration_for_every_caller(self):
        for caller in self.CALLERS:
            self.assertEqual(rc.resolve_arming('manual', caller, True,
                                               self.KE, -240000),
                             (rc.ARM_CALIBRATED, None))

    def test_manual_without_the_bottle_calibration_refuses(self):
        for caller in self.CALLERS:
            kind, refusal = rc.resolve_arming('manual', caller, False,
                                              self.KE, -240000)
            self.assertIsNone(kind)
            self.assertEqual(refusal[0], rc.NOT_CALIBRATED)
            self.assertIn('NEBULAOS_LOAD_CELL_CALIBRATE', refusal[1])

    def test_config_mode_names_are_accepted(self):
        self.assertEqual(rc.resolve_arming('calibrated_force', 'x', True,
                                           self.KE, 0)[0], rc.ARM_CALIBRATED)
        self.assertEqual(rc.resolve_arming('raw_contact', 'x', False,
                                           self.KE, 0)[0], rc.ARM_RAW)

    def test_effective_user_mode(self):
        self.assertEqual(rc.effective_user_mode('auto', 'manual'), 'manual')
        self.assertEqual(rc.effective_user_mode('auto', 'auto'), 'auto')
        self.assertEqual(rc.effective_user_mode('auto', 'garbage'), 'auto')
        self.assertEqual(rc.effective_user_mode('calibrated_force', 'auto'),
                         'manual')
        self.assertEqual(rc.effective_user_mode('raw_contact', 'manual'),
                         'auto')

    def test_auto_never_falls_back_to_the_calibration(self):
        # a calibrated unit in automatic mode without a reference refuses -
        # it does not silently use grams
        kind, refusal = rc.resolve_arming('auto', rc.CALLER_Z_OFFSET_PAIR,
                                          True, self.KE, None)
        self.assertIsNone(kind)
        self.assertEqual(refusal[0], rc.RAW_NO_REFERENCE)
        self.assertIn('NEBULAOS_LOAD_CELL_AUTO_CHECK', refusal[1])

    def test_a_failed_check_refuses_even_with_a_reference(self):
        kind, refusal = rc.resolve_arming('auto', rc.CALLER_TOUCH, False,
                                          self.KE, -240000,
                                          auto_check_failed=True)
        self.assertIsNone(kind)
        self.assertEqual(refusal[0], rc.RAW_AUTO_CHECK_FAILED)
        self.assertIn('Manual', refusal[1])

    def test_unknown_or_missing_profile_refuses(self):
        self.assertEqual(rc.resolve_arming('auto', 'x', True, None, 0)[1][0],
                         rc.RAW_NO_PROFILE)
        self.assertEqual(rc.resolve_arming('auto', 'x', True, 'ke-v9', 0)[1][0],
                         rc.RAW_UNKNOWN_PROFILE)

    def test_every_qualified_profile_passes_the_schema(self):
        for pid, profile in rc.QUALIFIED_PROFILES.items():
            self.assertEqual(rc.validate_profile(profile), [], pid)
            self.assertEqual(profile['id'], pid)

    def test_an_invalid_registry_profile_is_refused(self):
        bad = _profile(id='ke-bad')
        del bad['polarity']
        rc.QUALIFIED_PROFILES['ke-bad'] = bad
        kind, refusal = rc.resolve_arming('raw_contact', rc.CALLER_Z_OFFSET_PAIR,
                                          False, 'ke-bad', 0)
        self.assertIsNone(kind)
        self.assertEqual(refusal[0], rc.RAW_BAD_PROFILE)

    def test_describe_idle_reasons(self):
        text = rc.describe_idle_reasons(
            [rc.STUCK, 'noise_above_profile (sigma 900 counts)',
             rc.DRIFT_ABOVE_PROFILE])
        self.assertIn('does not change', text)
        self.assertIn('too noisy', text)
        self.assertIn('does not settle', text)

def _probe_ns():
    from . import nebulaos_load_cell_mode
    import logging
    import time
    return {'raw_contact': rc, 'math': math, 'lc_mode': nebulaos_load_cell_mode,
            'logging': logging, 'time': time}


def _bound_probe_methods(names):
    """Methods of ZOffsetProbe executed on a fake (no MCU stack needed)."""
    ns = _probe_ns()
    for name in names:
        code, fn, _ = _probe_function(name)
        exec(compile(ast.Module(body=[fn], type_ignores=[]), 'probe', 'exec'),
             ns)
    return {n: ns[n] for n in names}


# G99 functions appended to ZOffsetProbe at module level: (name in the
# module, attribute on the class)
_APPENDED = (('_save_lc_state', '_save_lc_state'),
             ('_set_load_cell_mode', 'set_load_cell_mode'),
             ('_run_auto_check', 'run_auto_check'),
             ('_record_auto_check', '_record_auto_check'),
             ('_auto_status', 'auto_status'))


def _appended_probe_functions():
    with open(os.path.join(HERE, 'nebulaos_z_offset_probe.py')) as fh:
        src = fh.read()
    wanted = dict(_APPENDED)
    fns = [n for n in ast.parse(src).body
           if isinstance(n, ast.FunctionDef) and n.name in wanted]
    ns = _probe_ns()
    exec(compile(ast.Module(body=fns, type_ignores=[]), 'p', 'exec'), ns)
    return {attr: ns[name] for name, attr in _APPENDED}


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
    def __init__(self, lc):
        self.lc = lc

    def start_collecting(self, min_time=None):
        self.lc.collect_log.append(('start', min_time))

    def collect_until(self, print_time=None):
        self.lc.collect_log.append(('until', print_time))
        # one fixed window, or successive windows (the last one repeats)
        w = self.lc.samples
        if w and isinstance(w[0], list):
            return (w.pop(0) if len(w) > 1 else w[0]), 0
        return w, 0


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
        return _Collector(self)

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

    def _probe(self, samples, reference=REF, calibrated=True,
               persisted=None, mode='auto', profile_id='ke-stock-v1',
               user_mode='auto', state=None, path=None):
        from . import nebulaos_load_cell_mode as lcm
        m = _bound_probe_methods(['_arm_raw', '_raw_reference_counts',
                                  '_refuse', 'plan_arming', 'contact_ready',
                                  '_raw_health_window', '_settled_idle',
                                  'load_cell_mode', '_auto_reference_counts',
                                  '_tare_and_arm'])
        m.update(_appended_probe_functions())
        probe = types.SimpleNamespace()
        probe._printer = _Printer()
        probe._name = 'nebulaos_z_offset_probe'
        probe._load_cell = _LoadCell(samples, reference, calibrated)
        probe._sensor = types.SimpleNamespace(
            get_range=lambda: (-0x800000, 0x7FFFFF))
        probe._mcu_trigger_analog = _Trigger()
        probe._raw_health_time = 0.75
        probe._raw_settle_time = 5.
        probe._auto_check_time = 2.
        probe._raw_reference = persisted
        probe._contact_mode = mode
        probe._raw_profile_id = profile_id
        probe._raw_diag = None
        probe._last_contact_rejection_reason = None
        probe._active_profile = None
        probe._effective_mode = None
        probe._lc_state = state or lcm.default_state()
        probe._lc_state['mode'] = user_mode
        probe._mode_path = path or os.path.join(
            self._tmpdir(), 'load_cell_mode.json')
        probe._mode_warning = ''
        for name, fn in m.items():
            setattr(probe, name, types.MethodType(fn, probe))
        return probe

    def _tmpdir(self):
        import shutil
        import tempfile
        d = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, d, True)
        return d

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

    def _drifting(self, n=70):
        # the ke-dev compare window: the bed still settling (-865 counts/s)
        return [(i / 90.6, None, self.REF - int(865 * i / 90.6), None)
                for i in range(n)]

    def test_settling_bed_is_measured_again_until_calm(self):
        probe = self._probe([self._drifting(), self._drifting(), _idle(self.REF)])
        probe._arm_raw(_profile())
        self.assertEqual(probe._raw_diag['settle_windows'], 3)
        self.assertAlmostEqual(probe._raw_diag['settle_seconds'], 2.25)
        self.assertEqual(probe._mcu_trigger_analog.calls[2], ('trigger', 'abs_ge', 6400))
        starts = [t for k, t in probe._load_cell.collect_log if k == 'start']
        self.assertEqual(starts, [42., 42.75, 43.5])      # consecutive windows

    def test_never_calm_is_refused_after_the_settle_time(self):
        probe = self._probe([self._drifting()])
        with self.assertRaises(_Err) as ctx:
            probe._arm_raw(_profile())
        self.assertEqual(probe._raw_diag['settle_windows'], 7)    # 5 s / 0.75 s
        self.assertIn('drift_above_profile', str(ctx.exception))
        self.assertEqual(probe._mcu_trigger_analog.calls, [])

    def test_a_sensor_fault_is_not_waited_out(self):
        stuck = [(i / 90.6, None, self.REF, None) for i in range(70)]
        probe = self._probe([stuck, _idle(self.REF)])
        with self.assertRaises(_Err):
            probe._arm_raw(_profile())
        self.assertEqual(probe._raw_diag['settle_windows'], 1)
        self.assertEqual(probe._mcu_trigger_analog.calls, [])

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

    def test_uncalibrated_unit_arms_around_its_persisted_reference(self):
        probe = self._probe(_idle(self.REF + 500), reference=None,
                            calibrated=False, persisted=self.REF)
        probe._arm_raw(_profile())
        self.assertEqual(probe._mcu_trigger_analog.calls[0],
                         ('raw_range', self.REF - 170000, self.REF + 170000))

    def test_contact_ready_stays_false_without_a_qualified_profile(self):
        probe = self._probe([], reference=None, calibrated=False,
                            persisted=self.REF, profile_id=None)
        for caller in (rc.CALLER_Z_OFFSET_PAIR, rc.CALLER_AUTO_CALIBRATE,
                       rc.CALLER_NOZZLE_CLEAN):
            self.assertFalse(probe.contact_ready(caller))

    def test_auto_mode_is_ready_for_every_caller_with_a_reference(self):
        # G99: the Run 14 "auto-calibrate never raw" rule is lifted
        probe = self._probe([], reference=None, calibrated=False,
                            persisted=self.REF)
        for caller in (rc.CALLER_Z_OFFSET_PAIR, rc.CALLER_AUTO_CALIBRATE,
                       rc.CALLER_NOZZLE_CLEAN, rc.CALLER_TOUCH):
            self.assertTrue(probe.contact_ready(caller))
            self.assertEqual(probe.plan_arming(caller)[0], rc.ARM_RAW)
        no_ref = self._probe([], reference=None, calibrated=False,
                             persisted=None)
        self.assertFalse(no_ref.contact_ready(rc.CALLER_Z_OFFSET_PAIR))
        self.assertEqual(no_ref.plan_arming(rc.CALLER_TOUCH)[1][0],
                         rc.RAW_NO_REFERENCE)

    def test_auto_mode_on_a_calibrated_unit_still_needs_the_check(self):
        # the bottle zero is not the automatic reference
        probe = self._probe([], calibrated=True, persisted=None)
        self.assertFalse(probe.contact_ready(rc.CALLER_Z_OFFSET_PAIR))

    def test_calibrated_unit_is_ready_in_manual_mode(self):
        probe = self._probe([], user_mode='manual')
        for caller in (rc.CALLER_Z_OFFSET_PAIR, rc.CALLER_AUTO_CALIBRATE,
                       rc.CALLER_NOZZLE_CLEAN):
            self.assertTrue(probe.contact_ready(caller))
            self.assertEqual(probe.plan_arming(caller),
                             (rc.ARM_CALIBRATED, None))

    def test_manual_without_the_bottle_calibration_refuses(self):
        probe = self._probe([], calibrated=False, persisted=self.REF,
                            user_mode='manual')
        self.assertFalse(probe.contact_ready(rc.CALLER_NOZZLE_CLEAN))
        with self.assertRaises(_Err) as ctx:
            probe._tare_and_arm(None)
        self.assertEqual(probe._last_contact_rejection_reason,
                         rc.NOT_CALIBRATED)
        self.assertIn('bottle', str(ctx.exception))
        self.assertEqual(probe._mcu_trigger_analog.calls, [])

    def test_config_override_beats_the_user_choice(self):
        probe = self._probe([], calibrated=True, persisted=None,
                            mode='calibrated_force', user_mode='auto')
        self.assertEqual(probe.load_cell_mode(), 'manual')
        self.assertTrue(probe.contact_ready(rc.CALLER_TOUCH))

    def test_touch_without_a_plan_follows_auto_mode(self):
        # the nozzle clean calls touch_probe() without a plan: raw arming
        probe = self._probe(_idle(self.REF), reference=None, calibrated=False,
                            persisted=self.REF)
        probe._tare_and_arm(None)
        self.assertEqual(probe._effective_mode, rc.MODE_RAW)
        self.assertEqual(probe._mcu_trigger_analog.calls[-1],
                         ('trigger', 'abs_ge', 6400))

    def test_safety_refusals_still_win_in_auto_mode(self):
        cases = (
            (_idle(self.REF - 20000), rc.RAW_REFERENCE_DRIFT),   # preload
            ([(i / 90.6, None, self.REF, None) for i in range(70)],
             rc.RAW_UNHEALTHY),                                 # stuck
            (_idle(self.REF, noise=3000), rc.RAW_UNHEALTHY),     # noisy
            (_idle(self.REF, n=6), rc.RAW_UNHEALTHY),            # too short
        )
        for samples, code in cases:
            probe = self._probe(samples, reference=None, calibrated=False,
                                persisted=self.REF)
            with self.assertRaises(_Err):
                probe._tare_and_arm(None)
            self.assertEqual(probe._last_contact_rejection_reason, code)
            self.assertEqual(probe._mcu_trigger_analog.calls, [])

    # --- G99 automatic check ------------------------------------------
    def test_auto_check_ok_stores_the_reference_in_the_state_file(self):
        from . import nebulaos_load_cell_mode as lcm
        probe = self._probe(_idle(self.REF, n=180), reference=None,
                            calibrated=False)
        ok, reason = probe.run_auto_check()
        self.assertTrue(ok, reason)
        st = probe.auto_status()
        self.assertEqual(st['auto_check'], 'ok')
        self.assertTrue(st['auto_ready'])
        self.assertAlmostEqual(st['auto_reference_counts'], self.REF, delta=60)
        saved, warning = lcm.read_state(probe._mode_path)
        self.assertEqual(warning, '')
        self.assertEqual(saved['auto_check'], 'ok')
        self.assertEqual(saved['reference_counts'], st['auto_reference_counts'])
        self.assertEqual(saved['profile_id'], 'ke-stock-v1')
        # no motion, no tare, no MCU arming, no Klipper config
        self.assertIsNone(probe._load_cell.tared)
        self.assertEqual(probe._mcu_trigger_analog.calls, [])
        # and the touch is now armed around it
        self.assertTrue(probe.contact_ready(rc.CALLER_AUTO_CALIBRATE))

    def test_auto_check_waits_for_a_settling_bed(self):
        probe = self._probe([self._drifting(n=180), _idle(self.REF, n=180)],
                            reference=None, calibrated=False)
        ok, _ = probe.run_auto_check()
        self.assertTrue(ok)
        self.assertEqual(probe._raw_diag['settle_windows'], 2)

    def test_auto_check_failure_clears_the_reference_and_says_why(self):
        from . import nebulaos_load_cell_mode as lcm
        stuck = [(i / 90.6, None, self.REF, None) for i in range(180)]
        probe = self._probe(stuck, reference=None, calibrated=False,
                            persisted=self.REF)
        self.assertTrue(probe.contact_ready(rc.CALLER_TOUCH))  # legacy ref
        ok, reason = probe.run_auto_check()
        self.assertFalse(ok)
        self.assertIn('does not change', reason)
        st = probe.auto_status()
        self.assertEqual((st['auto_check'], st['auto_ready']), ('failed', False))
        self.assertEqual(st['auto_check_reason'], reason)
        self.assertIsNone(st['auto_reference_counts'])
        self.assertFalse(probe.contact_ready(rc.CALLER_TOUCH))
        self.assertEqual(probe.plan_arming(rc.CALLER_TOUCH)[1][0],
                         rc.RAW_AUTO_CHECK_FAILED)
        self.assertEqual(lcm.read_state(probe._mode_path)[0]['auto_check'],
                         'failed')

    def test_auto_check_never_calm_fails(self):
        probe = self._probe([self._drifting(n=180)], reference=None,
                            calibrated=False)
        ok, reason = probe.run_auto_check()
        self.assertFalse(ok)
        self.assertIn('does not settle', reason)
        self.assertEqual(probe._raw_diag['settle_windows'], 3)   # 5 s / 2 s

    def test_auto_check_without_a_profile_fails(self):
        probe = self._probe(_idle(self.REF, n=180), profile_id='nope')
        ok, reason = probe.run_auto_check()
        self.assertFalse(ok)
        self.assertIn('not available', reason)

    def test_mode_switch_is_persisted_without_restart(self):
        from . import nebulaos_load_cell_mode as lcm
        probe = self._probe([], calibrated=True, persisted=None)
        self.assertEqual(probe.set_load_cell_mode('manual'), '')
        self.assertEqual(probe.load_cell_mode(), 'manual')
        self.assertTrue(probe.contact_ready(rc.CALLER_TOUCH))
        self.assertEqual(lcm.read_state(probe._mode_path)[0]['mode'], 'manual')
        probe.set_load_cell_mode('auto')
        self.assertEqual(lcm.read_state(probe._mode_path)[0]['mode'], 'auto')
        with self.assertRaises(_Err):
            probe.set_load_cell_mode('bogus')

    def test_unwritable_state_file_keeps_the_live_choice_with_a_warning(self):
        probe = self._probe([], path='/proc/nebulaos-no-such/dir/x.json')
        with self.assertLogs(level='ERROR'):
            warning = probe.set_load_cell_mode('manual')
        self.assertIn('could not save', warning)
        self.assertEqual(probe.load_cell_mode(), 'manual')
        self.assertEqual(probe.auto_status()['mode_warning'], warning)

    def _capture_cmd(self):
        with open(os.path.join(HERE, 'nebulaos_z_offset_probe.py')) as fh:
            src = fh.read()
        fn = next(n for n in ast.parse(src).body
                  if isinstance(n, ast.FunctionDef)
                  and n.name == '_cmd_raw_contact_reference')
        ns = {'raw_contact': rc}
        exec(compile(ast.Module(body=[fn], type_ignores=[]), 'p', 'exec'), ns)
        return ns['_cmd_raw_contact_reference']

    def _capture_probe(self, samples):
        probe = self._probe(samples, reference=None, calibrated=False)
        staged, scripts = [], []
        objs = {'toolhead': probe._printer.toolhead,
                'configfile': types.SimpleNamespace(
                    set=lambda sec, opt, val: staged.append((sec, opt, val))),
                'gcode': types.SimpleNamespace(
                    run_script_from_command=scripts.append)}
        probe._printer.lookup_object = lambda name, default=None: \
            objs.get(name, default)
        probe._sensor.get_samples_per_second = lambda: 80.
        return probe, staged, scripts

    def test_reference_capture_saves_the_median(self):
        probe, staged, scripts = self._capture_probe(_idle(self.REF, n=180))
        gcmd = types.SimpleNamespace(get_float=lambda n, d, **k: d,
                                     respond_info=lambda m: None,
                                     error=_Err)
        self._capture_cmd()(probe, gcmd)
        self.assertEqual(staged[0][:2], ('nebulaos_z_offset_probe',
                                         'raw_reference_counts'))
        self.assertAlmostEqual(staged[0][2], self.REF, delta=60)
        self.assertEqual(scripts, ['SAVE_CONFIG'])

    def test_reference_capture_refuses_a_stuck_sensor(self):
        stuck = [(i / 90., None, self.REF, None) for i in range(180)]
        probe, staged, scripts = self._capture_probe(stuck)
        gcmd = types.SimpleNamespace(get_float=lambda n, d, **k: d,
                                     respond_info=lambda m: None,
                                     error=_Err)
        with self.assertRaises(_Err):
            self._capture_cmd()(probe, gcmd)
        self.assertEqual((staged, scripts), ([], []))

    def test_no_reference_no_raw(self):
        probe = self._probe(_idle(self.REF), reference=None, calibrated=False)
        with self.assertRaises(_Err):
            probe._arm_raw(_profile())
        self.assertEqual(probe._last_contact_rejection_reason,
                         rc.RAW_NO_REFERENCE)


if __name__ == '__main__':
    unittest.main()
