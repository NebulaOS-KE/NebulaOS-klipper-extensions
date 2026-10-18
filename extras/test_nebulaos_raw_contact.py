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
        code, fn, _ = _probe_function('_tare_and_arm')
        first = fn.body[0]
        self.assertIsInstance(first, ast.If)
        self.assertIn('MODE_RAW', ast.unparse(first.test))
        # the refusal raises; nothing below it runs in raw mode
        self.assertTrue(any(isinstance(n, ast.Raise) for n in ast.walk(first)))
        self.assertLess(code.index('raw_motion_refusal'),
                        code.index('set_raw_range'))

    def test_default_mode_is_calibrated_force(self):
        _, _, src = _probe_function('__init__')
        self.assertIn("'contact_detection_mode'", src)
        self.assertIn("raw_contact.MODE_CALIBRATED)", src)

    def test_fit_uses_mode_aware_magnitude_and_no_fake_grams(self):
        code, _, _ = _probe_function('_fit_contact_z')
        self.assertIn('raw_contact.fit_magnitude(', code)
        # raw mode reports counts, never grams
        self.assertIn('_last_raw_peak_delta_counts', code)

    def test_status_keeps_is_calibrated_for_grams_only(self):
        code, _, _ = _probe_function('get_status')
        self.assertIn("'is_calibrated': self._load_cell.is_calibrated()", code)
        self.assertIn("'contact_qualified': (not raw_mode) and calibrated", code)

if __name__ == '__main__':
    unittest.main()
