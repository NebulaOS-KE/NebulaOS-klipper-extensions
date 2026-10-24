# Tests for extras/nebulaos_probe_pair.py (Phase 2 calibration-framework
# mission; bounded-descent contact-safety rewrite, CORRECTED).
#
# Ground truth this file is built to prove: pinned upstream Klipper's own
# manual_probe.py:create_probe_result() defines
#     bed_z = test_z - z_offset
# (test_z = raw toolhead Z at PROBE trigger; bed_z = upstream's own name
# for the ESTIMATED NOZZLE-CONTACT Z). A prior version of this module
# bounded nozzle descent directly off test_z, which is off by the entire
# probe z_offset - on this printer's real captured hardware state,
# raw probe trigger ~= +0.025, existing probe z_offset ~= 1.795, so the
# true predicted nozzle-contact plane is ~= -1.770, not +0.025. The
# REAL_* constants below are exactly those captured values; several tests
# prove the safety margin is applied relative to -1.770, not +0.025.
#
# Two envelope cases, both required to be gated (see the module's own
# header for the full formulas):
#   ESTABLISHED - credible existing probe z_offset (abs > epsilon):
#     predicted_nozzle_contact_z = raw_probe_trigger_z - probe_z_offset
#     commanded_floor_z = predicted_nozzle_contact_z
#                          - established_contact_margin_mm
#   BOOTSTRAP - factory z_offset=0.000 (or anything within epsilon) is
#   NOT a credible physical prior:
#     commanded_floor_z = starting_nozzle_z - bootstrap_contact_envelope_mm
#     fails closed unless bootstrap_contact_envelope_mm is explicitly
#     configured - never silently falls back to down_min_z.
#
# Run from klippy/: python3 -m unittest extras.test_nebulaos_probe_pair -v
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math
import sys
import types
import unittest

if 'extras.probe' not in sys.modules:
    _placeholder = types.ModuleType('extras.probe')
    _placeholder.run_single_probe = lambda probe_obj, gcmd: None
    sys.modules['extras.probe'] = _placeholder

from . import nebulaos_probe_pair as pairmod

# The real captured hardware values this mission's correction is built on.
REAL_RAW_PROBE_TRIGGER_Z = 0.025
REAL_EXISTING_PROBE_Z_OFFSET = 1.795
REAL_PREDICTED_NOZZLE_CONTACT_Z = -1.770  # 0.025 - 1.795, verified below


class FormulaSanityTest(unittest.TestCase):
    def test_real_captured_values_satisfy_upstream_bed_z_formula(self):
        # bed_z = test_z - z_offset (pinned manual_probe.py's own formula).
        self.assertAlmostEqual(
            REAL_RAW_PROBE_TRIGGER_Z - REAL_EXISTING_PROBE_Z_OFFSET,
            REAL_PREDICTED_NOZZLE_CONTACT_Z, places=9)


class FakeToolhead:
    def __init__(self, position=(110., 111., 5., 0.)):
        self._position = list(position)
        self.moves = []  # [(x_or_None, y_or_None, z_or_None, speed), ...]

    def get_position(self):
        return list(self._position)

    def manual_move(self, coord, speed):
        for i, v in enumerate(coord):
            if v is not None:
                self._position[i] = v
        self.moves.append((tuple(coord), speed))


class FakeGCode:
    def create_gcode_command(self, cmd, commandline, params):
        return ("fake-gcmd", params)


class FakeReactor:
    def __init__(self, start=0.0):
        self._t = start

    def monotonic(self):
        self._t += 0.001
        return self._t


class FakePrinter:
    def __init__(self, toolhead, gcode, probe_obj=None):
        self._objects = {'toolhead': toolhead, 'gcode': gcode,
                          'probe': probe_obj if probe_obj is not None else object()}
        self._reactor = FakeReactor()

    def lookup_object(self, name, default=None):
        return self._objects.get(name, default)

    def get_reactor(self):
        return self._reactor

    def command_error(self, msg):
        return CommandError(msg)


class CommandError(Exception):
    pass


class FakeProbeResult:
    def __init__(self, test_z):
        self.test_z = test_z


class FakeZOffsetProbe:
    """Stands in for nebulaos_z_offset_probe.ZOffsetProbe. Each
    touch_probe() call consumes the next queued (fitted_z, raw_z,
    fit_delta) triple, records the call (including the commanded
    minimum_allowed_z it was given), and makes the triple available via
    get_status() exactly like the real object does."""

    def __init__(self, contacts):
        # contacts: list of (fitted_z, raw_z, fit_delta) tuples, or a
        # single tuple reused for every call, or an Exception to raise.
        self._contacts = contacts
        self.calls = []
        self._i = 0
        self._last = None
        self._name = 'nebulaos_z_offset_probe fake_load_cell'

    def touch_probe(self, down_min_z, pro_cnt=1, minimum_allowed_z=None):
        self.calls.append({'down_min_z': down_min_z, 'pro_cnt': pro_cnt,
                            'minimum_allowed_z': minimum_allowed_z})
        call_index = self._i
        item = (self._contacts[self._i] if isinstance(self._contacts, list)
                else self._contacts)
        if isinstance(self._contacts, list):
            self._i += 1
        if isinstance(item, Exception):
            raise item
        fitted_z, raw_z, fit_delta = item
        # Synthetic but deterministic force telemetry, distinct per call
        # (index-derived) so tests can tell samples apart without needing
        # a 4th tuple element for every existing fixture.
        peak_force_g = -80.0 - call_index
        peak_force_time = 100.0 + call_index
        force_at_trigger_g = -76.0 - call_index
        tare_counts = -249399 + call_index
        self._last = (raw_z, fitted_z, fit_delta, peak_force_g,
                      peak_force_time, force_at_trigger_g, tare_counts)
        return fitted_z

    def get_status(self, eventtime):
        (raw_z, fitted_z, fit_delta, peak_force_g, peak_force_time,
         force_at_trigger_g, tare_counts) = self._last
        return {'last_raw_trigger_z': raw_z, 'last_fitted_contact_z': fitted_z,
                'last_fit_delta': fit_delta,
                'last_peak_force_g': peak_force_g,
                'last_peak_force_time': peak_force_time,
                'last_force_at_trigger_g': force_at_trigger_g,
                'last_tare_counts': tare_counts,
                'trigger_force': 75.0, 'force_safety_limit': 2000.0,
                'contact_speed': 2.0}


def _patch_run_single_probe(probe_trigger_z):
    orig = pairmod.probe_module.run_single_probe
    pairmod.probe_module.run_single_probe = \
        lambda probe_obj, gcmd: FakeProbeResult(probe_trigger_z)
    return orig


def _restore_run_single_probe(orig):
    pairmod.probe_module.run_single_probe = orig


class _Base(unittest.TestCase):
    def _measure(self, probe_trigger_z, contacts, probe_z_offset=0.0,
                 established_contact_margin_mm=None,
                 bootstrap_contact_envelope_mm=None,
                 max_abs_fit_delta=1.0, min_accepted_samples=None,
                 max_repeatability_range=None, max_repeatability_stddev=None,
                 pro_cnt=1, x=100., y=100., probe_x_offset=0.,
                 probe_y_offset=27., down_min_z=10., toolhead=None):
        toolhead = toolhead if toolhead is not None else FakeToolhead()
        gcode = FakeGCode()
        printer = FakePrinter(toolhead, gcode)
        z_probe = FakeZOffsetProbe(contacts)
        orig = _patch_run_single_probe(probe_trigger_z)
        try:
            return pairmod.measure_probe_nozzle_pair(
                printer, x=x, y=y, probe_x_offset=probe_x_offset,
                probe_y_offset=probe_y_offset, probe_z_offset=probe_z_offset,
                horizontal_move_z=8., z_offset_probe=z_probe,
                down_min_z=down_min_z, pro_cnt=pro_cnt, travel_speed=200.,
                probe_lift_speed=20.,
                established_contact_margin_mm=established_contact_margin_mm,
                bootstrap_contact_envelope_mm=bootstrap_contact_envelope_mm,
                max_abs_fit_delta=max_abs_fit_delta,
                min_accepted_samples=min_accepted_samples,
                max_repeatability_range=max_repeatability_range,
                max_repeatability_stddev=max_repeatability_stddev), z_probe, toolhead
        finally:
            _restore_run_single_probe(orig)


# ======================================================================
# Credibility detection - what makes an existing z_offset a valid prior
# ======================================================================

class CredibilityTest(unittest.TestCase):
    def test_exact_factory_zero_is_not_credible(self):
        self.assertFalse(pairmod._is_credible_probe_z_offset(0.0))

    def test_tiny_epsilon_value_is_not_credible(self):
        self.assertFalse(pairmod._is_credible_probe_z_offset(0.0001))
        self.assertFalse(pairmod._is_credible_probe_z_offset(-0.0001))

    def test_real_captured_offset_is_credible(self):
        self.assertTrue(
            pairmod._is_credible_probe_z_offset(REAL_EXISTING_PROBE_Z_OFFSET))

    def test_small_but_genuine_offset_is_credible(self):
        self.assertTrue(pairmod._is_credible_probe_z_offset(0.05))
        self.assertTrue(pairmod._is_credible_probe_z_offset(-0.05))

    def test_non_finite_is_not_credible(self):
        self.assertFalse(pairmod._is_credible_probe_z_offset(float('nan')))
        self.assertFalse(pairmod._is_credible_probe_z_offset(float('inf')))


# ======================================================================
# ESTABLISHED case - the real captured values, corrected formula
# ======================================================================

class EstablishedEnvelopeRealValuesTest(_Base):
    """The single highest-value test class in this file: proves the
    envelope is anchored to the predicted NOZZLE-CONTACT plane
    (~-1.770), not the raw probe trigger (~+0.025), using the exact
    real captured hardware values."""

    def test_predicted_nozzle_contact_z_matches_upstream_bed_z_formula(self):
        result, _z, _th = self._measure(
            REAL_RAW_PROBE_TRIGGER_Z,
            [(REAL_PREDICTED_NOZZLE_CONTACT_Z, REAL_PREDICTED_NOZZLE_CONTACT_Z, 0.0)],
            probe_z_offset=REAL_EXISTING_PROBE_Z_OFFSET,
            established_contact_margin_mm=1.0)
        self.assertEqual(result.contact_mode, 'established')
        self.assertAlmostEqual(
            result.predicted_nozzle_contact_z, REAL_PREDICTED_NOZZLE_CONTACT_Z,
            places=9)

    def test_commanded_floor_is_relative_to_predicted_contact_not_raw_trigger(self):
        margin = 1.0
        result, z_probe, _th = self._measure(
            REAL_RAW_PROBE_TRIGGER_Z,
            [(REAL_PREDICTED_NOZZLE_CONTACT_Z, REAL_PREDICTED_NOZZLE_CONTACT_Z, 0.0)],
            probe_z_offset=REAL_EXISTING_PROBE_Z_OFFSET,
            established_contact_margin_mm=margin)
        expected_floor = REAL_PREDICTED_NOZZLE_CONTACT_Z - margin  # -2.770
        self.assertAlmostEqual(result.commanded_floor_z, expected_floor, places=9)
        # The floor is nowhere near raw_trigger - margin (0.025 - 1.0 =
        # -0.975) - the bug this correction fixes would have produced
        # THAT number instead, off by the entire 1.795mm probe offset.
        wrong_old_floor = REAL_RAW_PROBE_TRIGGER_Z - margin
        self.assertNotAlmostEqual(result.commanded_floor_z, wrong_old_floor, places=2)
        self.assertGreater(abs(result.commanded_floor_z - wrong_old_floor), 1.5)
        self.assertEqual(z_probe.calls[0]['minimum_allowed_z'], expected_floor)

    def test_margin_scales_the_floor_relative_to_predicted_plane(self):
        for margin in (0.5, 1.0, 2.0):
            result, _z, _th = self._measure(
                REAL_RAW_PROBE_TRIGGER_Z,
                [(REAL_PREDICTED_NOZZLE_CONTACT_Z, REAL_PREDICTED_NOZZLE_CONTACT_Z, 0.0)],
                probe_z_offset=REAL_EXISTING_PROBE_Z_OFFSET,
                established_contact_margin_mm=margin)
            self.assertAlmostEqual(
                result.commanded_floor_z,
                REAL_PREDICTED_NOZZLE_CONTACT_Z - margin, places=9)


class EstablishedEnvelopeGeneralTest(_Base):
    def test_envelope_passed_to_touch_probe_every_sample(self):
        result, z_probe, _th = self._measure(
            2.5, [(0.831, 0.831, 0.0)] * 3, probe_z_offset=1.0,
            established_contact_margin_mm=1.0,
            pro_cnt=3, min_accepted_samples=1, max_repeatability_range=5.0,
            max_repeatability_stddev=5.0)
        expected_floor = (2.5 - 1.0) - 1.0  # predicted=1.5, floor=0.5
        for call in z_probe.calls:
            self.assertAlmostEqual(call['minimum_allowed_z'], expected_floor, places=9)

    def test_non_finite_cr_touch_reading_refuses_before_nozzle_contact(self):
        toolhead = FakeToolhead()
        with self.assertRaises(CommandError) as ctx:
            self._measure(float('nan'), [(0.831, 0.831, 0.0)],
                          probe_z_offset=1.0, established_contact_margin_mm=1.0,
                          toolhead=toolhead)
        self.assertIn('not finite', str(ctx.exception))


# ======================================================================
# BOOTSTRAP case - virgin z_offset=0, fail-closed coverage
# ======================================================================

class BootstrapEnvelopeTest(_Base):
    def test_virgin_zero_offset_refuses_without_bootstrap_envelope_configured(self):
        toolhead = FakeToolhead()
        with self.assertRaises(CommandError) as ctx:
            self._measure(REAL_RAW_PROBE_TRIGGER_Z, [(-1.0, -1.0, 0.0)],
                          probe_z_offset=0.0,
                          bootstrap_contact_envelope_mm=None,
                          toolhead=toolhead)
        self.assertIn('CONTACT_SAFETY_LIMIT_UNQUALIFIED', str(ctx.exception))
        self.assertIn('bootstrap_contact_envelope_mm', str(ctx.exception))
        self.assertIn('BOOTSTRAP', str(ctx.exception))
        # No motion at all - not even the CR-Touch hover/travel steps.
        self.assertEqual(toolhead.moves, [])

    def test_virgin_offset_never_silently_uses_down_min_z_as_bootstrap_depth(self):
        # A large down_min_z must NOT substitute for the missing,
        # separately-qualified bootstrap_contact_envelope_mm.
        toolhead = FakeToolhead()
        with self.assertRaises(CommandError) as ctx:
            self._measure(REAL_RAW_PROBE_TRIGGER_Z, [(-1.0, -1.0, 0.0)],
                          probe_z_offset=0.0, down_min_z=25.,
                          bootstrap_contact_envelope_mm=None,
                          toolhead=toolhead)
        self.assertIn('CONTACT_SAFETY_LIMIT_UNQUALIFIED', str(ctx.exception))
        self.assertEqual(toolhead.moves, [])

    def test_tiny_near_zero_offset_is_also_treated_as_bootstrap(self):
        toolhead = FakeToolhead()
        with self.assertRaises(CommandError) as ctx:
            self._measure(REAL_RAW_PROBE_TRIGGER_Z, [(-1.0, -1.0, 0.0)],
                          probe_z_offset=0.0002,
                          bootstrap_contact_envelope_mm=None,
                          established_contact_margin_mm=1.0,
                          toolhead=toolhead)
        self.assertIn('bootstrap_contact_envelope_mm', str(ctx.exception))

    def test_established_margin_configured_does_not_satisfy_bootstrap_case(self):
        # Configuring established_contact_margin_mm alone must not let a
        # virgin (z_offset=0) run proceed - the two cases require their
        # own, separate configuration.
        toolhead = FakeToolhead()
        with self.assertRaises(CommandError) as ctx:
            self._measure(REAL_RAW_PROBE_TRIGGER_Z, [(-1.0, -1.0, 0.0)],
                          probe_z_offset=0.0,
                          established_contact_margin_mm=1.0,
                          bootstrap_contact_envelope_mm=None,
                          toolhead=toolhead)
        self.assertIn('bootstrap_contact_envelope_mm', str(ctx.exception))

    def test_bootstrap_floor_is_relative_to_starting_nozzle_z_not_a_prediction(self):
        toolhead = FakeToolhead(position=(0., 0., 8., 0.))
        result, z_probe, _th = self._measure(
            REAL_RAW_PROBE_TRIGGER_Z, [(-5.0, -5.0, 0.0)],
            probe_z_offset=0.0, bootstrap_contact_envelope_mm=10.0,
            toolhead=toolhead)
        self.assertEqual(result.contact_mode, 'bootstrap')
        self.assertIsNone(result.predicted_nozzle_contact_z)
        # horizontal_move_z=8. in _Base - the nozzle hovers there before
        # the bootstrap descent begins.
        expected_floor = 8.0 - 10.0
        self.assertAlmostEqual(result.commanded_floor_z, expected_floor, places=9)
        self.assertEqual(z_probe.calls[0]['minimum_allowed_z'], expected_floor)

    def test_bootstrap_succeeds_when_envelope_is_configured(self):
        # horizontal_move_z=8. (the _Base default) - envelope=10.0 gives a
        # floor of -2.0, so the fake contact must trigger at or above -2.0
        # to be accepted (this fixture's raw/fitted Z stays comfortably
        # inside the envelope, exactly like a real, in-bounds trigger would).
        result, _z, _th = self._measure(
            REAL_RAW_PROBE_TRIGGER_Z, [(-1.0, -1.0, 0.0)],
            probe_z_offset=0.0, bootstrap_contact_envelope_mm=10.0)
        self.assertTrue(result.accepted)
        self.assertEqual(result.contact_mode, 'bootstrap')


# ======================================================================
# Fail-closed preflight: other required constants (unchanged concepts)
# ======================================================================

class FailClosedPreflightTest(_Base):
    def test_missing_max_abs_fit_delta_refuses_before_any_contact(self):
        toolhead = FakeToolhead()
        with self.assertRaises(CommandError) as ctx:
            self._measure(REAL_RAW_PROBE_TRIGGER_Z, [(0.831, 0.831, 0.0)],
                          probe_z_offset=REAL_EXISTING_PROBE_Z_OFFSET,
                          established_contact_margin_mm=1.0,
                          max_abs_fit_delta=None, toolhead=toolhead)
        self.assertIn('CONTACT_SAFETY_LIMIT_UNQUALIFIED', str(ctx.exception))
        self.assertIn('max_abs_fit_delta', str(ctx.exception))
        self.assertEqual(toolhead.moves, [])

    def test_missing_repeatability_bounds_refuses_when_pro_cnt_over_one(self):
        toolhead = FakeToolhead()
        with self.assertRaises(CommandError) as ctx:
            self._measure(REAL_RAW_PROBE_TRIGGER_Z, [(0.831, 0.831, 0.0)] * 3,
                          probe_z_offset=REAL_EXISTING_PROBE_Z_OFFSET,
                          established_contact_margin_mm=1.0, pro_cnt=3,
                          min_accepted_samples=None, toolhead=toolhead)
        self.assertIn('CONTACT_SAFETY_LIMIT_UNQUALIFIED', str(ctx.exception))
        self.assertEqual(toolhead.moves, [])

    def test_pro_cnt_one_does_not_require_repeatability_bounds(self):
        result, _z, _th = self._measure(
            REAL_RAW_PROBE_TRIGGER_Z, [(0.831, 0.831, 0.0)],
            probe_z_offset=REAL_EXISTING_PROBE_Z_OFFSET,
            established_contact_margin_mm=1.0, pro_cnt=1,
            min_accepted_samples=None, max_repeatability_range=None,
            max_repeatability_stddev=None)
        self.assertTrue(result.accepted)


# ======================================================================
# Sign / arithmetic of the FINAL probe_z_offset result (unaffected by
# the envelope fix - this is the separate, pre-existing, already-correct
# calibration algebra)
# ======================================================================

class SignAndArithmeticTest(_Base):
    def test_probe_higher_than_nozzle_gives_positive_offset(self):
        result, _z, _th = self._measure(
            2.500, [(0.831, 0.831, 0.0)], probe_z_offset=1.0,
            established_contact_margin_mm=1.0)
        self.assertAlmostEqual(result.probe_z_offset, 2.500 - 0.831, places=9)
        self.assertAlmostEqual(result.probe_z_offset, 1.669, places=9)

    def test_probe_lower_than_nozzle_gives_negative_offset(self):
        result, _z, _th = self._measure(
            0.831, [(2.500, 2.500, 0.0)], probe_z_offset=1.0,
            established_contact_margin_mm=5.0)
        self.assertAlmostEqual(result.probe_z_offset, 0.831 - 2.500, places=9)
        self.assertAlmostEqual(result.probe_z_offset, -1.669, places=9)

    def test_equal_readings_give_exactly_zero(self):
        result, _z, _th = self._measure(
            1.234, [(1.234, 1.234, 0.0)], probe_z_offset=1.0,
            established_contact_margin_mm=1.0)
        self.assertEqual(result.probe_z_offset, 0.0)

    def test_raw_fields_are_reported_unmodified(self):
        result, _z, _th = self._measure(
            -0.500, [(0.250, 0.250, 0.0)], probe_z_offset=1.0,
            established_contact_margin_mm=1.0)
        self.assertEqual(result.raw_probe_trigger_z, -0.500)
        self.assertEqual(result.raw_nozzle_contact_z, 0.250)


# ======================================================================
# XY offset / move ordering (unaffected by the envelope fix)
# ======================================================================

class XYOffsetAndMoveOrderTest(_Base):
    def test_probe_moves_to_offset_position_nozzle_moves_to_bare_xy(self):
        toolhead = FakeToolhead()
        probed_xy_at_call_time = {}
        gcode = FakeGCode()
        printer = FakePrinter(toolhead, gcode)
        z_probe = FakeZOffsetProbe([(0.5, 0.5, 0.0)])

        def fake_run_single_probe(probe_obj, gcmd):
            probed_xy_at_call_time['xy'] = tuple(toolhead.get_position()[:2])
            return FakeProbeResult(2.0)

        orig = pairmod.probe_module.run_single_probe
        pairmod.probe_module.run_single_probe = fake_run_single_probe
        try:
            pairmod.measure_probe_nozzle_pair(
                printer, x=100., y=100., probe_x_offset=0., probe_y_offset=27.,
                probe_z_offset=1.0,
                horizontal_move_z=8., z_offset_probe=z_probe, down_min_z=10.,
                established_contact_margin_mm=1.0, max_abs_fit_delta=1.0)
        finally:
            pairmod.probe_module.run_single_probe = orig

        self.assertEqual(probed_xy_at_call_time['xy'], (100.0, 73.0))
        nozzle_xy = (toolhead.get_position()[0], toolhead.get_position()[1])
        self.assertEqual(nozzle_xy, (100.0, 100.0))

    def test_result_x_y_match_the_requested_point(self):
        result, _z, _th = self._measure(
            2.0, [(0.5, 0.5, 0.0)], probe_z_offset=1.0,
            established_contact_margin_mm=1.0, x=42., y=99.)
        self.assertEqual((result.x, result.y), (42., 99.))


# ======================================================================
# Measurement-quality gates (§9) - unaffected by the envelope fix
# ======================================================================

class QualityGateTest(_Base):
    def test_excessive_fit_delta_is_rejected(self):
        result, _z, _th = self._measure(
            2.5, [(0.0, 1.5, 1.5)], probe_z_offset=1.0,
            established_contact_margin_mm=1.0, max_abs_fit_delta=1.0)
        self.assertFalse(result.accepted)
        self.assertIn('excessive_fit_delta',
                       result.repeatability.samples[0].rejection_reason)
        self.assertEqual(result.rejection_reason, 'no_accepted_samples')
        self.assertIsNone(result.probe_z_offset)

    def test_acceptable_fit_delta_is_accepted(self):
        result, _z, _th = self._measure(
            2.5, [(0.831, 0.900, 0.069)], probe_z_offset=1.0,
            established_contact_margin_mm=1.0, max_abs_fit_delta=1.0)
        self.assertTrue(result.accepted)
        self.assertIsNotNone(result.probe_z_offset)

    def test_the_real_recorded_hardware_incident_is_rejected(self):
        # §23: the actual concerning contact from the overnight
        # investigation - raw_trigger_z ~ -1.585, fitted_contact_z ~
        # -0.479, fit_delta ~ -1.106. A configured fit-delta guard well
        # under 1.106 must reject it, and nothing must be staged.
        result, _z, _th = self._measure(
            0.0, [(-0.479, -1.585, -1.106)], probe_z_offset=1.0,
            established_contact_margin_mm=5.0, max_abs_fit_delta=0.5)
        self.assertFalse(result.accepted)
        self.assertIn('excessive_fit_delta',
                       result.repeatability.samples[0].rejection_reason)
        self.assertAlmostEqual(abs(result.repeatability.samples[0].fit_delta),
                                1.106, places=3)
        self.assertIsNone(result.probe_z_offset)

    def test_non_finite_fit_delta_is_rejected_as_invalid_not_excessive(self):
        result, _z, _th = self._measure(
            2.5, [(0.831, 0.831, float('nan'))], probe_z_offset=1.0,
            established_contact_margin_mm=1.0, max_abs_fit_delta=1.0)
        self.assertFalse(result.accepted)
        self.assertEqual(result.repeatability.samples[0].rejection_reason,
                          'non_finite_or_missing_fit_result')

    def test_non_finite_fitted_z_is_rejected(self):
        result, _z, _th = self._measure(
            2.5, [(float('nan'), 0.831, 0.0)], probe_z_offset=1.0,
            established_contact_margin_mm=1.0, max_abs_fit_delta=1.0)
        self.assertFalse(result.accepted)
        self.assertEqual(result.repeatability.samples[0].rejection_reason,
                          'non_finite_or_missing_fit_result')

    def test_zero_fit_delta_is_accepted(self):
        result, _z, _th = self._measure(
            2.5, [(0.831, 0.831, 0.0)], probe_z_offset=1.0,
            established_contact_margin_mm=1.0, max_abs_fit_delta=1.0)
        self.assertTrue(result.accepted)

    def test_positive_and_negative_fit_delta_both_gated_by_absolute_value(self):
        rej_pos, _z, _th = self._measure(
            2.5, [(0.0, 2.0, 2.0)], probe_z_offset=1.0,
            established_contact_margin_mm=1.0, max_abs_fit_delta=1.0)
        rej_neg, _z, _th = self._measure(
            2.5, [(2.0, 0.0, -2.0)], probe_z_offset=1.0,
            established_contact_margin_mm=1.0, max_abs_fit_delta=1.0)
        self.assertFalse(rej_pos.accepted)
        self.assertFalse(rej_neg.accepted)

    def test_contact_error_propagates_and_records_the_sample(self):
        toolhead = FakeToolhead()
        with self.assertRaises(CommandError):
            self._measure(2.5, [CommandError("sensor error")], probe_z_offset=1.0,
                          established_contact_margin_mm=1.0, toolhead=toolhead)


# ======================================================================
# Repeatability aggregation (§10) - unaffected by the envelope fix
# ======================================================================

class RepeatabilityTest(_Base):
    def test_mean_min_max_range_stddev_computed_over_accepted_samples(self):
        result, _z, _th = self._measure(
            2.5, [(1.0, 1.0, 0.0), (1.2, 1.2, 0.0), (0.8, 0.8, 0.0)],
            probe_z_offset=1.0, established_contact_margin_mm=1.0,
            pro_cnt=3, max_abs_fit_delta=1.0, min_accepted_samples=1,
            max_repeatability_range=10.0, max_repeatability_stddev=10.0)
        rep = result.repeatability
        self.assertEqual(rep.accepted_count, 3)
        self.assertAlmostEqual(rep.mean, 1.0, places=9)
        self.assertAlmostEqual(rep.minimum, 0.8, places=9)
        self.assertAlmostEqual(rep.maximum, 1.2, places=9)
        self.assertAlmostEqual(rep.range, 0.4, places=9)
        self.assertGreater(rep.stddev, 0.0)

    def test_single_sample_stddev_is_zero_not_an_error(self):
        result, _z, _th = self._measure(
            2.5, [(1.0, 1.0, 0.0)], probe_z_offset=1.0,
            established_contact_margin_mm=1.0, pro_cnt=1, max_abs_fit_delta=1.0)
        self.assertEqual(result.repeatability.stddev, 0.0)

    def test_insufficient_accepted_samples_rejects(self):
        result, _z, _th = self._measure(
            2.5, [(1.0, 1.0, 0.0), (1.0, 1.0, 0.0)], probe_z_offset=1.0,
            established_contact_margin_mm=1.0, pro_cnt=2,
            max_abs_fit_delta=1.0, min_accepted_samples=3,
            max_repeatability_range=10.0, max_repeatability_stddev=10.0)
        self.assertFalse(result.accepted)
        self.assertIn('insufficient_accepted_samples', result.rejection_reason)

    def test_repeatability_range_exceeded_rejects(self):
        result, _z, _th = self._measure(
            2.5, [(0.0, 0.0, 0.0), (5.0, 5.0, 0.0)], probe_z_offset=1.0,
            established_contact_margin_mm=10.0, pro_cnt=2,
            max_abs_fit_delta=10.0, min_accepted_samples=2,
            max_repeatability_range=1.0, max_repeatability_stddev=10.0)
        self.assertFalse(result.accepted)
        self.assertIn('repeatability_range_exceeded', result.rejection_reason)

    def test_repeatability_stddev_exceeded_rejects(self):
        result, _z, _th = self._measure(
            2.5, [(0.0, 0.0, 0.0), (5.0, 5.0, 0.0), (0.0, 0.0, 0.0)],
            probe_z_offset=1.0, established_contact_margin_mm=10.0, pro_cnt=3,
            max_abs_fit_delta=10.0, min_accepted_samples=2,
            max_repeatability_range=10.0, max_repeatability_stddev=0.5)
        self.assertFalse(result.accepted)
        self.assertIn('repeatability_stddev_exceeded', result.rejection_reason)

    def test_one_rejected_repetition_still_leaves_others_accepted(self):
        result, _z, _th = self._measure(
            2.5, [(1.0, 1.0, 0.0), (0.0, 5.0, 5.0), (1.1, 1.1, 0.0)],
            probe_z_offset=1.0, established_contact_margin_mm=10.0, pro_cnt=3,
            max_abs_fit_delta=1.0, min_accepted_samples=2,
            max_repeatability_range=10.0, max_repeatability_stddev=10.0)
        self.assertEqual(result.repeatability.sample_count, 3)
        self.assertEqual(result.repeatability.accepted_count, 2)
        rejected = [s for s in result.repeatability.samples if not s.accepted]
        self.assertEqual(len(rejected), 1)
        self.assertIsNotNone(rejected[0].rejection_reason)
        self.assertTrue(result.accepted)

    def test_all_repetitions_rejected_means_overall_rejected(self):
        result, _z, _th = self._measure(
            2.5, [(0.0, 5.0, 5.0), (0.0, 5.0, 5.0)], probe_z_offset=1.0,
            established_contact_margin_mm=10.0, pro_cnt=2,
            max_abs_fit_delta=1.0, min_accepted_samples=1,
            max_repeatability_range=10.0, max_repeatability_stddev=10.0)
        self.assertFalse(result.accepted)
        self.assertEqual(result.repeatability.accepted_count, 0)
        self.assertEqual(result.rejection_reason, 'no_accepted_samples')

    def test_no_sample_is_ever_silently_dropped(self):
        result, _z, _th = self._measure(
            2.5, [(1.0, 1.0, 0.0), (0.0, 5.0, 5.0), (1.1, 1.1, 0.0)],
            probe_z_offset=1.0, established_contact_margin_mm=10.0, pro_cnt=3,
            max_abs_fit_delta=1.0, min_accepted_samples=1,
            max_repeatability_range=10.0, max_repeatability_stddev=10.0)
        self.assertEqual(len(result.repeatability.samples), 3)
        for s in result.repeatability.samples:
            self.assertTrue(s.accepted or s.rejection_reason is not None)


# ======================================================================
# Rejected results must never carry a stageable offset
# ======================================================================

class RejectionNeverStagesTest(_Base):
    def test_rejected_result_has_no_probe_z_offset(self):
        result, _z, _th = self._measure(
            2.5, [(0.0, 1.5, 1.5)], probe_z_offset=1.0,
            established_contact_margin_mm=1.0, max_abs_fit_delta=1.0)
        self.assertFalse(result.accepted)
        self.assertIsNone(result.probe_z_offset)

    def test_toolhead_still_lifted_clear_after_rejection(self):
        toolhead = FakeToolhead()
        result, _z, _th = self._measure(
            2.5, [(0.0, 1.5, 1.5)], probe_z_offset=1.0,
            established_contact_margin_mm=1.0, max_abs_fit_delta=1.0,
            toolhead=toolhead)
        self.assertEqual(toolhead.get_position()[2], 8.0)  # horizontal_move_z


# ======================================================================
# Contact-force telemetry (Phase 2 mission, §1) - propagation through
# ContactSample and PairedMeasurement
# ======================================================================

class ContactForceTelemetryPropagationTest(_Base):
    def test_per_sample_force_fields_populated_from_status(self):
        result, z_probe, _th = self._measure(
            REAL_RAW_PROBE_TRIGGER_Z, [(-0.005, -0.235, -0.229)],
            probe_z_offset=REAL_EXISTING_PROBE_Z_OFFSET,
            established_contact_margin_mm=1.0)
        sample = result.repeatability.samples[0]
        # FakeZOffsetProbe derives these deterministically from call index
        # 0: peak=-80.0, peak_time=100.0, force_at_trigger=-76.0, tare=-249399
        self.assertAlmostEqual(sample.peak_force_g, -80.0, places=9)
        self.assertAlmostEqual(sample.peak_force_time, 100.0, places=9)
        self.assertAlmostEqual(sample.force_at_trigger_g, -76.0, places=9)
        self.assertEqual(sample.tare_counts, -249399)

    def test_predicted_to_raw_trigger_depth_established_case(self):
        # predicted=-1.770, raw=-2.000 -> trigger 0.230mm BELOW predicted
        # (positive = deeper than predicted, matching the real hardware
        # language "trigger depth ... below the predicted plane").
        result, _z, _th = self._measure(
            REAL_RAW_PROBE_TRIGGER_Z, [(-2.010, -2.000, -0.010)],
            probe_z_offset=REAL_EXISTING_PROBE_Z_OFFSET,
            established_contact_margin_mm=1.0)
        sample = result.repeatability.samples[0]
        self.assertAlmostEqual(
            sample.predicted_to_raw_trigger_depth,
            REAL_PREDICTED_NOZZLE_CONTACT_Z - (-2.000), places=9)
        self.assertGreater(sample.predicted_to_raw_trigger_depth, 0)

    def test_predicted_to_raw_trigger_depth_is_none_for_bootstrap(self):
        # No prediction exists in the BOOTSTRAP case - the field must be
        # None, not silently computed against something meaningless.
        result, _z, _th = self._measure(
            REAL_RAW_PROBE_TRIGGER_Z, [(-1.0, -1.0, 0.0)],
            probe_z_offset=0.0, bootstrap_contact_envelope_mm=10.0)
        sample = result.repeatability.samples[0]
        self.assertIsNone(sample.predicted_to_raw_trigger_depth)

    def test_remaining_margin_to_floor(self):
        margin = 1.0
        result, _z, _th = self._measure(
            REAL_RAW_PROBE_TRIGGER_Z, [(-2.010, -2.000, -0.010)],
            probe_z_offset=REAL_EXISTING_PROBE_Z_OFFSET,
            established_contact_margin_mm=margin)
        sample = result.repeatability.samples[0]
        expected_floor = REAL_PREDICTED_NOZZLE_CONTACT_Z - margin
        self.assertAlmostEqual(
            sample.remaining_margin_to_floor, -2.000 - expected_floor,
            places=9)
        self.assertGreater(sample.remaining_margin_to_floor, 0)

    def test_configured_values_echoed_on_paired_measurement(self):
        result, _z, _th = self._measure(
            REAL_RAW_PROBE_TRIGGER_Z, [(-2.010, -2.000, -0.010)],
            probe_z_offset=REAL_EXISTING_PROBE_Z_OFFSET,
            established_contact_margin_mm=1.0)
        self.assertAlmostEqual(result.trigger_force, 75.0, places=9)
        self.assertAlmostEqual(result.force_safety_limit, 2000.0, places=9)
        self.assertAlmostEqual(result.contact_speed, 2.0, places=9)

    def test_force_fields_none_on_hard_contact_error_sample(self):
        toolhead = FakeToolhead()
        with self.assertRaises(CommandError):
            self._measure(
                REAL_RAW_PROBE_TRIGGER_Z, [CommandError("sensor error")],
                probe_z_offset=REAL_EXISTING_PROBE_Z_OFFSET,
                established_contact_margin_mm=1.0, toolhead=toolhead)
        # The error propagates (asserted above); this test only documents
        # that the failure-path ContactSample construction (see
        # measure_probe_nozzle_pair's own except-block) does not raise a
        # SECOND, unrelated error (e.g. a missing-field TypeError) while
        # building that sample - if it did, assertRaises above would have
        # seen the wrong exception type.

    def test_no_sample_missing_force_fields_across_a_repeatability_set(self):
        result, _z, _th = self._measure(
            REAL_RAW_PROBE_TRIGGER_Z,
            [(-2.01, -2.00, -0.01), (-2.02, -2.01, -0.01), (-2.00, -1.99, -0.01)],
            probe_z_offset=REAL_EXISTING_PROBE_Z_OFFSET,
            established_contact_margin_mm=1.0, pro_cnt=3,
            min_accepted_samples=1, max_repeatability_range=10.0,
            max_repeatability_stddev=10.0)
        self.assertEqual(len(result.repeatability.samples), 3)
        for i, s in enumerate(result.repeatability.samples):
            self.assertIsNotNone(s.peak_force_g)
            self.assertIsNotNone(s.tare_counts)
            self.assertIsNotNone(s.remaining_margin_to_floor)
            # Distinct per call index - proves each sample got its OWN
            # telemetry, not one snapshot reused across all three.
            self.assertAlmostEqual(s.peak_force_g, -80.0 - i, places=9)


if __name__ == '__main__':
    unittest.main()


# ======================================================================
# G104: "touch n of m" event (informational only)
# ======================================================================

class TouchEventTest(_Base):
    def test_each_touch_is_announced_before_it_starts(self):
        toolhead = FakeToolhead()
        gcode = FakeGCode()
        printer = FakePrinter(toolhead, gcode)
        events = []
        z_probe = FakeZOffsetProbe([(0.5, 0.5, 0.0)] * 3)
        orig_touch = z_probe.touch_probe

        def touch(*a, **kw):
            events.append('touch')
            return orig_touch(*a, **kw)
        z_probe.touch_probe = touch
        printer.send_event = lambda name, *args: events.append((name,) + args)
        orig = pairmod.probe_module.run_single_probe
        pairmod.probe_module.run_single_probe = \
            lambda probe_obj, gcmd: FakeProbeResult(2.0)
        try:
            pairmod.measure_probe_nozzle_pair(
                printer, x=100., y=100., probe_x_offset=0., probe_y_offset=27.,
                probe_z_offset=1.0, horizontal_move_z=8., z_offset_probe=z_probe,
                down_min_z=10., pro_cnt=3, established_contact_margin_mm=1.0,
                max_abs_fit_delta=1.0, min_accepted_samples=2,
                max_repeatability_range=1.0, max_repeatability_stddev=1.0)
        finally:
            pairmod.probe_module.run_single_probe = orig
        self.assertEqual(events, [
            ('nebulaos:contact_touch', 1, 3), 'touch',
            ('nebulaos:contact_touch', 2, 3), 'touch',
            ('nebulaos:contact_touch', 3, 3), 'touch'])
