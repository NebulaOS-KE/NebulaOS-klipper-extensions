# Tests for extras/nebulaos_calibration.py (Phase 2 calibration-framework
# mission; contact-safety stabilization, corrected) - the NebulaOS
# calibration coordinator: NEBULAOS_Z_OFFSET_CALIBRATE (LOAD_CELL only -
# manual users call pristine upstream PROBE_CALIBRATE directly) and
# NEBULAOS_CALIBRATION_STATUS. The PID-default-target and bed-mesh-named-
# profile Python wrappers have been removed (upstream-first cleanup -
# see nebulaos_calibration.py's own header for why the bed-mesh one was a
# real duplication: pinned BED_MESH_CALIBRATE PROFILE=<name> already does
# exactly what the wrapper did).
#
# Deliberately stubs nebulaos_probe_pair.measure_probe_nozzle_pair() itself
# (already independently tested for sign/coordinate/envelope correctness
# in test_nebulaos_probe_pair.py) - what THIS file proves is the
# coordinator's OWN logic: preflight gating, magnitude/finite validation,
# the live probe-offset adapter (including forwarding the CURRENT
# probe_z_offset and the two new envelope constants unchanged), configfile
# staging, and command delegation.
#
# Run from klippy/: python3 -m unittest extras.test_nebulaos_calibration -v
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import sys
import types
import unittest

from . import prtouch_test_support as fake

if 'extras.probe' not in sys.modules:
    _placeholder = types.ModuleType('extras.probe')
    _placeholder.run_single_probe = lambda probe_obj, gcmd: None
    sys.modules['extras.probe'] = _placeholder

from . import nebulaos_calibration
from . import nebulaos_probe_pair


class FakeProbeOffsets:
    def __init__(self, x_offset, y_offset, z_offset):
        self.x_offset = x_offset
        self.y_offset = y_offset
        self.z_offset = z_offset


class FakeCmdHelper:
    def __init__(self, name):
        self.name = name


class FakeProbeObj:
    """Stands in for the real registered 'probe' object (PrinterBLTouch),
    modeling exactly the three attributes nebulaos_calibration.py's live
    adapter depends on - probe_offsets.z_offset (mutable, the CURRENT
    prior this whole workflow is refining), probe_offsets.x_offset/
    y_offset, and cmd_helper.name (for configfile.set()'s section
    argument) - all confirmed directly against the pinned bltouch.py
    source (see that module's own comment). Defaults to a CREDIBLE
    (nonzero) z_offset, matching the real captured hardware state most
    tests in this file assume - tests that specifically need the
    BOOTSTRAP/virgin case pass z_offset=0. explicitly."""

    def __init__(self, x_offset=0., y_offset=27., z_offset=1.795):
        self.probe_offsets = FakeProbeOffsets(x_offset, y_offset, z_offset)
        self.cmd_helper = FakeCmdHelper('bltouch')

    def get_offsets(self, gcmd=None):
        o = self.probe_offsets
        return o.x_offset, o.y_offset, o.z_offset


class FakeZOffsetProbe:
    def __init__(self, is_calibrated=True):
        self._is_calibrated = is_calibrated

    def get_status(self, eventtime):
        return {'is_calibrated': self._is_calibrated}


class FakeZCompensate:
    def __init__(self, home_x, home_y):
        self.home_x = home_x
        self.home_y = home_y


class FakeConfigFile:
    def __init__(self):
        self.set_calls = []

    def set(self, section, option, value):
        self.set_calls.append((section, option, value))


def _build(z_offset_probe=None, probe_obj=None, z_compensate=None,
           config_overrides=None):
    printer = fake.FakePrinter()
    gcode = fake.FakeGCode()
    printer.add_object('gcode', gcode)
    printer.add_object('probe', probe_obj if probe_obj is not None else FakeProbeObj())
    printer.add_object('configfile', FakeConfigFile())
    if z_offset_probe is not None:
        printer.add_object('nebulaos_z_offset_probe', z_offset_probe)
    if z_compensate is not None:
        printer.add_object('z_compensate', z_compensate)

    values = dict(config_overrides or {})
    config = fake.FakeConfig(values, section='nebulaos_calibration', printer=printer)
    coordinator = nebulaos_calibration.NebulaOSCalibration(config)
    config.assert_all_consumed()
    return printer, gcode, coordinator


def _accepted_measurement(x, y, probe_trigger_z, nozzle_contact_z,
                           contact_mode='established'):
    repeatability = nebulaos_probe_pair.RepeatabilityResult(
        sample_count=1, accepted_count=1, samples=[], mean=nozzle_contact_z,
        minimum=nozzle_contact_z, maximum=nozzle_contact_z, range=0.0,
        stddev=0.0, accepted=True, rejection_reason=None)
    return nebulaos_probe_pair.PairedMeasurement(
        x=x, y=y, contact_id=1.0, contact_mode=contact_mode,
        predicted_nozzle_contact_z=probe_trigger_z - 1.795,
        commanded_floor_z=probe_trigger_z - 1.795 - 1.0,
        raw_probe_trigger_z=probe_trigger_z,
        raw_nozzle_contact_z=nozzle_contact_z,
        repeatability=repeatability,
        probe_z_offset=probe_trigger_z - nozzle_contact_z,
        accepted=True, rejection_reason=None,
        trigger_force=75.0, force_safety_limit=2000.0, contact_speed=2.0)


def _rejected_measurement(x, y, reason='excessive_fit_delta(9.0>1.0)'):
    repeatability = nebulaos_probe_pair.RepeatabilityResult(
        sample_count=1, accepted_count=0, samples=[], mean=None,
        minimum=None, maximum=None, range=None, stddev=None,
        accepted=False, rejection_reason=reason)
    return nebulaos_probe_pair.PairedMeasurement(
        x=x, y=y, contact_id=1.0, contact_mode='established',
        predicted_nozzle_contact_z=0.2, commanded_floor_z=-0.8,
        raw_probe_trigger_z=2.0,
        raw_nozzle_contact_z=None, repeatability=repeatability,
        probe_z_offset=None, accepted=False, rejection_reason=reason,
        trigger_force=75.0, force_safety_limit=2000.0, contact_speed=2.0)


def _stub_pair(probe_trigger_z, nozzle_contact_z):
    def fake_measure(printer, x, y, probe_x_offset, probe_y_offset,
                      probe_z_offset,
                      horizontal_move_z, z_offset_probe, down_min_z,
                      pro_cnt=1, travel_speed=None, probe_lift_speed=None,
                      established_contact_margin_mm=None,
                      bootstrap_contact_envelope_mm=None,
                      max_abs_fit_delta=None, min_accepted_samples=None,
                      max_repeatability_range=None,
                      max_repeatability_stddev=None):
        return _accepted_measurement(x, y, probe_trigger_z, nozzle_contact_z)
    return fake_measure


class LoadCellHappyPathTest(unittest.TestCase):
    def test_measurement_applied_live_and_staged(self):
        z_probe = FakeZOffsetProbe(is_calibrated=True)
        probe_obj = FakeProbeObj(x_offset=0., y_offset=27., z_offset=1.795)
        zc = FakeZCompensate(home_x=110., home_y=111.)
        printer, gcode, coord = _build(z_probe, probe_obj, zc)

        orig = nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair
        nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = \
            _stub_pair(probe_trigger_z=2.500, nozzle_contact_z=0.831)
        try:
            gcmd = fake.FakeGCmd({})
            coord.cmd_z_offset_calibrate(gcmd)
        finally:
            nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = orig

        self.assertAlmostEqual(probe_obj.probe_offsets.z_offset, 1.669, places=9)
        configfile = printer.lookup_object('configfile')
        self.assertEqual(len(configfile.set_calls), 1)
        section, option, value = configfile.set_calls[0]
        self.assertEqual(section, 'bltouch')
        self.assertEqual(option, 'z_offset')
        self.assertAlmostEqual(float(value), 1.669, places=2)
        self.assertEqual(coord.z_offset_state, 'complete')
        self.assertAlmostEqual(coord.z_offset_result, 1.669, places=9)
        self.assertIsNone(coord.z_offset_error)

    def test_explicit_xy_override_bypasses_z_compensate(self):
        z_probe = FakeZOffsetProbe(is_calibrated=True)
        probe_obj = FakeProbeObj()
        printer, gcode, coord = _build(z_probe, probe_obj, z_compensate=None)
        seen = {}

        def fake_measure(printer_, x, y, *a, **kw):
            seen['xy'] = (x, y)
            return _accepted_measurement(x, y, 1.0, 0.5)
        orig = nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair
        nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = fake_measure
        try:
            coord.cmd_z_offset_calibrate(fake.FakeGCmd({'X': '42', 'Y': '99'}))
        finally:
            nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = orig
        self.assertEqual(seen['xy'], (42.0, 99.0))

    def test_current_probe_z_offset_forwarded_unchanged(self):
        # The coordinator must forward the CURRENT live probe z_offset
        # (the prior this workflow is refining) exactly as read from
        # probe_obj.get_offsets() - not re-derive or clamp it.
        z_probe = FakeZOffsetProbe(is_calibrated=True)
        probe_obj = FakeProbeObj(z_offset=1.795)
        zc = FakeZCompensate(110., 111.)
        printer, gcode, coord = _build(z_probe, probe_obj, zc)
        seen = {}

        def fake_measure(printer_, x, y, probe_x_offset, probe_y_offset,
                          probe_z_offset, *a, **kw):
            seen['probe_z_offset'] = probe_z_offset
            return _accepted_measurement(x, y, 1.0, 0.5)
        orig = nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair
        nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = fake_measure
        try:
            coord.cmd_z_offset_calibrate(fake.FakeGCmd({}))
        finally:
            nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = orig
        self.assertAlmostEqual(seen['probe_z_offset'], 1.795, places=9)


class LoadCellPreflightTest(unittest.TestCase):
    def test_no_load_cell_configured_raises_and_mentions_probe_calibrate(self):
        printer, gcode, coord = _build(z_offset_probe=None,
                                        z_compensate=FakeZCompensate(110., 111.))
        with self.assertRaises(fake.CommandError) as ctx:
            coord.cmd_z_offset_calibrate(fake.FakeGCmd({}))
        self.assertIn('PROBE_CALIBRATE', str(ctx.exception))

    def test_uncalibrated_load_cell_raises_and_mentions_probe_calibrate(self):
        z_probe = FakeZOffsetProbe(is_calibrated=False)
        printer, gcode, coord = _build(z_probe, z_compensate=FakeZCompensate(110., 111.))
        with self.assertRaises(fake.CommandError) as ctx:
            coord.cmd_z_offset_calibrate(fake.FakeGCmd({}))
        msg = str(ctx.exception)
        self.assertIn('LOAD_CELL_CALIBRATE', msg)
        self.assertIn('PROBE_CALIBRATE', msg)

    def test_no_reference_point_available_raises(self):
        z_probe = FakeZOffsetProbe(is_calibrated=True)
        printer, gcode, coord = _build(z_probe, z_compensate=None)
        with self.assertRaises(fake.CommandError):
            coord.cmd_z_offset_calibrate(fake.FakeGCmd({}))

    def test_no_method_parameter_accepted_or_needed(self):
        # There is no METHOD= dispatch any more - the command works with
        # a bare gcmd, and passing METHOD= (if a caller still does, out of
        # habit) is simply ignored, not rejected, since gcmd.get() calls
        # for it no longer exist in this handler at all.
        z_probe = FakeZOffsetProbe(is_calibrated=True)
        zc = FakeZCompensate(110., 111.)
        printer, gcode, coord = _build(z_probe, FakeProbeObj(), zc)
        orig = nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair
        nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = \
            _stub_pair(1.0, 0.5)
        try:
            coord.cmd_z_offset_calibrate(fake.FakeGCmd({'METHOD': 'LOAD_CELL'}))
        finally:
            nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = orig
        self.assertEqual(coord.z_offset_state, 'complete')


class LoadCellValidationTest(unittest.TestCase):
    def test_implausibly_large_correction_is_rejected_and_not_applied(self):
        z_probe = FakeZOffsetProbe(is_calibrated=True)
        probe_obj = FakeProbeObj(z_offset=1.795)
        zc = FakeZCompensate(110., 111.)
        printer, gcode, coord = _build(z_probe, probe_obj, zc)
        orig = nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair
        # 5mm apart - exceeds the default 2mm max_offset_correction_mm.
        nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = \
            _stub_pair(probe_trigger_z=5.0, nozzle_contact_z=0.0)
        try:
            with self.assertRaises(fake.CommandError) as ctx:
                coord.cmd_z_offset_calibrate(fake.FakeGCmd({}))
        finally:
            nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = orig
        self.assertIn('max_offset_correction_mm', str(ctx.exception))
        # Must NOT have mutated the live probe offset or staged anything.
        self.assertAlmostEqual(probe_obj.probe_offsets.z_offset, 1.795, places=9)
        configfile = printer.lookup_object('configfile')
        self.assertEqual(configfile.set_calls, [])
        self.assertEqual(coord.z_offset_state, 'error')
        self.assertIn('max_offset_correction_mm', coord.z_offset_error)

    def test_non_finite_measurement_is_rejected(self):
        z_probe = FakeZOffsetProbe(is_calibrated=True)
        probe_obj = FakeProbeObj(z_offset=1.795)
        zc = FakeZCompensate(110., 111.)
        printer, gcode, coord = _build(z_probe, probe_obj, zc)
        orig = nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair
        nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = \
            _stub_pair(probe_trigger_z=float('nan'), nozzle_contact_z=0.0)
        try:
            with self.assertRaises(fake.CommandError):
                coord.cmd_z_offset_calibrate(fake.FakeGCmd({}))
        finally:
            nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = orig
        self.assertAlmostEqual(probe_obj.probe_offsets.z_offset, 1.795, places=9)

    def test_custom_max_offset_correction_mm_is_honored(self):
        z_probe = FakeZOffsetProbe(is_calibrated=True)
        probe_obj = FakeProbeObj(z_offset=1.795)
        zc = FakeZCompensate(110., 111.)
        printer, gcode, coord = _build(
            z_probe, probe_obj, zc,
            config_overrides={'max_offset_correction_mm': '10'})
        orig = nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair
        nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = \
            _stub_pair(probe_trigger_z=5.0, nozzle_contact_z=0.0)
        try:
            coord.cmd_z_offset_calibrate(fake.FakeGCmd({}))
        finally:
            nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = orig
        self.assertEqual(coord.z_offset_state, 'complete')
        self.assertAlmostEqual(probe_obj.probe_offsets.z_offset, 5.0, places=9)

    def test_thermal_drift_within_correction_bound_is_accepted(self):
        # Regression test for the max_offset_correction_mm semantics bug
        # (Phase 2 mission §6): a hot absolute z_offset of 2.300mm is past
        # the default 2.0mm bound in isolation, but the prior established
        # calibration is 1.795mm, so the actual CORRECTION this run implies
        # is only 0.505mm - well within max_offset_correction_mm=2.0. This
        # is real thermal-expansion calibration data, not an implausible
        # result, and must be accepted and applied.
        z_probe = FakeZOffsetProbe(is_calibrated=True)
        probe_obj = FakeProbeObj(z_offset=1.795)
        zc = FakeZCompensate(110., 111.)
        printer, gcode, coord = _build(z_probe, probe_obj, zc)
        orig = nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair
        nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = \
            _stub_pair(probe_trigger_z=2.300, nozzle_contact_z=0.0)
        try:
            coord.cmd_z_offset_calibrate(fake.FakeGCmd({}))
        finally:
            nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = orig
        self.assertEqual(coord.z_offset_state, 'complete')
        self.assertAlmostEqual(probe_obj.probe_offsets.z_offset, 2.300, places=9)

    def test_correction_bound_is_checked_against_prior_not_absolute_value(self):
        # A small absolute value can still be an implausible CORRECTION if
        # the prior is itself large - the bound must track the prior, not
        # a fixed absolute magnitude. Prior 1.795 -> new 0.5 is a -1.295mm
        # correction (within bound, accepted); this pins that the gate is
        # symmetric and prior-relative in both directions.
        z_probe = FakeZOffsetProbe(is_calibrated=True)
        probe_obj = FakeProbeObj(z_offset=1.795)
        zc = FakeZCompensate(110., 111.)
        printer, gcode, coord = _build(z_probe, probe_obj, zc)
        orig = nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair
        nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = \
            _stub_pair(probe_trigger_z=0.5, nozzle_contact_z=0.0)
        try:
            coord.cmd_z_offset_calibrate(fake.FakeGCmd({}))
        finally:
            nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = orig
        self.assertEqual(coord.z_offset_state, 'complete')
        self.assertAlmostEqual(probe_obj.probe_offsets.z_offset, 0.5, places=9)

    def test_correction_exceeding_bound_relative_to_prior_is_still_rejected(self):
        # The fix must not simply stop rejecting anything: a genuinely
        # implausible correction relative to the prior (here +2.205mm from
        # 1.795 to 4.0) must still be refused, and must not stage or apply.
        z_probe = FakeZOffsetProbe(is_calibrated=True)
        probe_obj = FakeProbeObj(z_offset=1.795)
        zc = FakeZCompensate(110., 111.)
        printer, gcode, coord = _build(z_probe, probe_obj, zc)
        orig = nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair
        nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = \
            _stub_pair(probe_trigger_z=4.0, nozzle_contact_z=0.0)
        try:
            with self.assertRaises(fake.CommandError) as ctx:
                coord.cmd_z_offset_calibrate(fake.FakeGCmd({}))
        finally:
            nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = orig
        self.assertIn('max_offset_correction_mm', str(ctx.exception))
        self.assertAlmostEqual(probe_obj.probe_offsets.z_offset, 1.795, places=9)
        configfile = printer.lookup_object('configfile')
        self.assertEqual(configfile.set_calls, [])
        self.assertEqual(coord.z_offset_state, 'error')

    def test_bootstrap_mode_is_not_subject_to_the_correction_bound(self):
        # BOOTSTRAP has no credible prior to correct from (see
        # nebulaos_probe_pair.py's _is_credible_probe_z_offset) -
        # max_offset_correction_mm is a fine-tune bound for refining an
        # ESTABLISHED calibration, and does not apply here. Plausibility
        # for a bootstrap result is measure_probe_nozzle_pair's own
        # bootstrap_contact_envelope_mm, exercised elsewhere.
        z_probe = FakeZOffsetProbe(is_calibrated=True)
        probe_obj = FakeProbeObj(z_offset=1.795)
        zc = FakeZCompensate(110., 111.)
        printer, gcode, coord = _build(z_probe, probe_obj, zc)

        def fake_measure(printer_, x, y, *a, **kw):
            return _accepted_measurement(
                x, y, probe_trigger_z=20.0, nozzle_contact_z=0.0,
                contact_mode='bootstrap')
        orig = nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair
        nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = fake_measure
        try:
            coord.cmd_z_offset_calibrate(fake.FakeGCmd({}))
        finally:
            nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = orig
        self.assertEqual(coord.z_offset_state, 'complete')
        self.assertAlmostEqual(probe_obj.probe_offsets.z_offset, 20.0, places=9)


class LoadCellRejectionGatingTest(unittest.TestCase):
    """§9: a rejected measurement must never stage a Z-offset, transition
    to COMPLETE, or call SAVE_CONFIG. Also verifies the coordinator's own
    envelope/quality/repeatability config values are passed through to
    measure_probe_nozzle_pair() unchanged (not re-derived a second way)."""

    def test_rejected_measurement_is_not_applied_or_staged(self):
        z_probe = FakeZOffsetProbe(is_calibrated=True)
        probe_obj = FakeProbeObj(z_offset=0.25)
        zc = FakeZCompensate(110., 111.)
        printer, gcode, coord = _build(z_probe, probe_obj, zc)

        def fake_measure(printer_, x, y, *a, **kw):
            return _rejected_measurement(x, y)
        orig = nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair
        nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = fake_measure
        try:
            with self.assertRaises(fake.CommandError) as ctx:
                coord.cmd_z_offset_calibrate(fake.FakeGCmd({}))
        finally:
            nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = orig

        self.assertIn('measurement rejected', str(ctx.exception))
        self.assertIn('excessive_fit_delta', str(ctx.exception))
        self.assertAlmostEqual(probe_obj.probe_offsets.z_offset, 0.25, places=9)
        configfile = printer.lookup_object('configfile')
        self.assertEqual(configfile.set_calls, [])
        self.assertEqual(coord.z_offset_state, 'measurement_quality_failure')

    def test_capability_unqualified_error_sets_distinct_state(self):
        z_probe = FakeZOffsetProbe(is_calibrated=True)
        zc = FakeZCompensate(110., 111.)
        printer, gcode, coord = _build(z_probe, FakeProbeObj(), zc)

        def fake_measure(printer_, x, y, *a, **kw):
            raise printer_.command_error(
                "CONTACT_SAFETY_LIMIT_UNQUALIFIED: established_contact_"
                "margin_mm is not configured")
        orig = nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair
        nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = fake_measure
        try:
            with self.assertRaises(fake.CommandError):
                coord.cmd_z_offset_calibrate(fake.FakeGCmd({}))
        finally:
            nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = orig
        self.assertEqual(coord.z_offset_state, 'capability_unqualified')

    def test_envelope_and_quality_config_passed_through_unchanged(self):
        z_probe = FakeZOffsetProbe(is_calibrated=True)
        zc = FakeZCompensate(110., 111.)
        printer, gcode, coord = _build(
            z_probe, FakeProbeObj(), zc,
            config_overrides={
                'established_contact_margin_mm': '1.5',
                'bootstrap_contact_envelope_mm': '8.0',
                'max_abs_fit_delta': '0.3', 'min_accepted_samples': '2',
                'max_repeatability_range': '0.1', 'max_repeatability_stddev': '0.05'})
        seen = {}

        def fake_measure(printer_, x, y, *a, **kw):
            seen.update(kw)
            return _accepted_measurement(x, y, 1.0, 0.5)
        orig = nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair
        nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = fake_measure
        try:
            coord.cmd_z_offset_calibrate(fake.FakeGCmd({}))
        finally:
            nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = orig
        self.assertAlmostEqual(seen['established_contact_margin_mm'], 1.5, places=9)
        self.assertAlmostEqual(seen['bootstrap_contact_envelope_mm'], 8.0, places=9)
        self.assertAlmostEqual(seen['max_abs_fit_delta'], 0.3, places=9)
        self.assertEqual(seen['min_accepted_samples'], 2)
        self.assertAlmostEqual(seen['max_repeatability_range'], 0.1, places=9)
        self.assertAlmostEqual(seen['max_repeatability_stddev'], 0.05, places=9)

    def test_qualified_defaults_apply_with_zero_config_overrides(self):
        # Phase 2 mission §7/§8 qualification (2026-09-02): established_
        # contact_margin_mm/max_abs_fit_delta/min_accepted_samples/
        # max_repeatability_range/max_repeatability_stddev/bootstrap_
        # contact_envelope_mm now all have real qualified defaults (1.0,
        # 0.3, 2, 0.15, 0.06, 8.0) and no longer require a
        # qualification-only printer.cfg override to run a calibration at
        # all, in either ESTABLISHED or BOOTSTRAP mode.
        z_probe = FakeZOffsetProbe(is_calibrated=True)
        zc = FakeZCompensate(110., 111.)
        printer, gcode, coord = _build(z_probe, FakeProbeObj(), zc)
        seen = {}

        def fake_measure(printer_, x, y, *a, **kw):
            seen.update(kw)
            return _accepted_measurement(x, y, 1.0, 0.5)
        orig = nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair
        nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = fake_measure
        try:
            coord.cmd_z_offset_calibrate(fake.FakeGCmd({}))
        finally:
            nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = orig
        self.assertAlmostEqual(seen['established_contact_margin_mm'], 1.0, places=9)
        self.assertAlmostEqual(seen['max_abs_fit_delta'], 0.3, places=9)
        self.assertEqual(seen['min_accepted_samples'], 2)
        self.assertAlmostEqual(seen['max_repeatability_range'], 0.15, places=9)
        self.assertAlmostEqual(seen['max_repeatability_stddev'], 0.06, places=9)
        self.assertAlmostEqual(seen['bootstrap_contact_envelope_mm'], 8.0, places=9)
        self.assertEqual(coord.z_offset_state, 'complete')

    def test_diagnostics_recorded_in_status_after_success(self):
        z_probe = FakeZOffsetProbe(is_calibrated=True)
        zc = FakeZCompensate(110., 111.)
        printer, gcode, coord = _build(z_probe, FakeProbeObj(), zc)
        orig = nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair
        nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = \
            _stub_pair(2.0, 0.5)
        try:
            coord.cmd_z_offset_calibrate(fake.FakeGCmd({}))
        finally:
            nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = orig
        status = coord.get_status(0.)
        self.assertEqual(status['z_offset_contact_mode'], 'established')
        self.assertAlmostEqual(
            status['z_offset_predicted_nozzle_contact_z'], 2.0 - 1.795, places=9)
        self.assertEqual(status['z_offset_raw_probe_trigger_z'], 2.0)
        self.assertEqual(status['z_offset_accepted_count'], 1)


class ReferenceXYTest(unittest.TestCase):
    """§11: the canonical reference point now comes from this section's
    own reference_x/reference_y config, not [z_compensate]'s home_x/
    home_y - which stays only as a fallback for an older printer.cfg."""

    def test_configured_reference_xy_used_when_no_explicit_override(self):
        z_probe = FakeZOffsetProbe(is_calibrated=True)
        printer, gcode, coord = _build(
            z_probe, FakeProbeObj(), z_compensate=FakeZCompensate(110., 111.),
            config_overrides={'reference_x': '20', 'reference_y': '25'})
        seen = {}

        def fake_measure(printer_, x, y, *a, **kw):
            seen['xy'] = (x, y)
            return _accepted_measurement(x, y, 1.0, 0.5)
        orig = nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair
        nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = fake_measure
        try:
            coord.cmd_z_offset_calibrate(fake.FakeGCmd({}))
        finally:
            nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = orig
        self.assertEqual(seen['xy'], (20.0, 25.0))

    def test_explicit_gcmd_xy_overrides_configured_reference(self):
        z_probe = FakeZOffsetProbe(is_calibrated=True)
        printer, gcode, coord = _build(
            z_probe, FakeProbeObj(), z_compensate=FakeZCompensate(110., 111.),
            config_overrides={'reference_x': '20', 'reference_y': '25'})
        seen = {}

        def fake_measure(printer_, x, y, *a, **kw):
            seen['xy'] = (x, y)
            return _accepted_measurement(x, y, 1.0, 0.5)
        orig = nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair
        nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = fake_measure
        try:
            coord.cmd_z_offset_calibrate(fake.FakeGCmd({'X': '42', 'Y': '99'}))
        finally:
            nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = orig
        self.assertEqual(seen['xy'], (42.0, 99.0))

    def test_falls_back_to_z_compensate_home_xy_when_reference_unset(self):
        z_probe = FakeZOffsetProbe(is_calibrated=True)
        printer, gcode, coord = _build(
            z_probe, FakeProbeObj(), z_compensate=FakeZCompensate(110., 111.))
        seen = {}

        def fake_measure(printer_, x, y, *a, **kw):
            seen['xy'] = (x, y)
            return _accepted_measurement(x, y, 1.0, 0.5)
        orig = nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair
        nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = fake_measure
        try:
            coord.cmd_z_offset_calibrate(fake.FakeGCmd({}))
        finally:
            nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = orig
        self.assertEqual(seen['xy'], (110.0, 111.0))


class UpstreamFirstCleanupTest(unittest.TestCase):
    """Confirms the removed Python wrappers are genuinely gone, not just
    unregistered - a future re-addition should not silently reintroduce
    the exact duplication this cleanup removed."""

    def test_no_manual_method_handler_exists(self):
        printer, gcode, coord = _build()
        self.assertFalse(hasattr(coord, '_z_offset_calibrate_manual'))

    def test_no_pid_wrapper_methods_exist(self):
        printer, gcode, coord = _build()
        self.assertFalse(hasattr(coord, 'cmd_pid_calibrate_bed'))
        self.assertFalse(hasattr(coord, 'cmd_pid_calibrate_hotend'))

    def test_no_bed_mesh_wrapper_method_exists(self):
        printer, gcode, coord = _build()
        self.assertFalse(hasattr(coord, 'cmd_bed_mesh_calibrate'))

    def test_no_pid_or_bed_mesh_or_manual_commands_registered(self):
        # Mission §11/§12: NEBULAOS_AUTO_CALIBRATE and
        # NEBULAOS_CALIBRATION_CANCEL are real, legitimate NEW commands -
        # this test's job is only to confirm the REMOVED wrapper commands
        # (PID convenience macros, bed-mesh named-profile wrapper, manual
        # METHOD= passthrough) never come back, not that the command set
        # is frozen forever. NEBULAOS_AXIS_TWIST_CALIBRATE is gone
        # entirely (mission §21, final product decision) - automatic Axis
        # Twist is not supported; manual Axis Twist is pristine upstream
        # AXIS_TWIST_COMPENSATION_CALIBRATE, which needs no wrapper here.
        printer, gcode, coord = _build()
        self.assertEqual(
            sorted(gcode.commands.keys()),
            ['NEBULAOS_CALIBRATION_STATUS',
             '_NEBULAOS_AUTO_CALIBRATE',
             '_NEBULAOS_CALIBRATION_CANCEL',
             '_NEBULAOS_ESTEPS_CALIBRATE',
             '_NEBULAOS_ESTEPS_CANCEL',
             '_NEBULAOS_ESTEPS_CONFIRM',
             '_NEBULAOS_INPUT_SHAPER_CALIBRATE',
             '_NEBULAOS_INPUT_SHAPER_CANCEL',
             '_NEBULAOS_Z_OFFSET_CALIBRATE'])

    def test_no_axis_twist_calibrate_command_or_state(self):
        # Mission §21: NEBULAOS_AXIS_TWIST_CALIBRATE and every axis_twist_*
        # status field must be gone completely, not merely hard-blocked.
        printer, gcode, coord = _build()
        self.assertNotIn('NEBULAOS_AXIS_TWIST_CALIBRATE', gcode.commands)
        self.assertFalse(hasattr(coord, 'cmd_axis_twist_calibrate'))
        self.assertFalse(hasattr(coord, 'axis_twist_id'))
        self.assertFalse(hasattr(coord, 'axis_twist_x_state'))
        self.assertFalse(hasattr(coord, 'axis_twist_y_state'))
        status = coord.get_status(0.)
        self.assertNotIn('axis_twist_id', status)
        self.assertNotIn('axis_twist_x_state', status)
        self.assertNotIn('axis_twist_y_state', status)


class SimulateBootstrapTest(unittest.TestCase):
    """Phase 2 mission §8: SIMULATE_BOOTSTRAP=1 ENVELOPE=<mm> qualifies
    the BOOTSTRAP contact path on an already-calibrated unit - see
    nebulaos_calibration.py's _cmd_z_offset_calibrate_simulate_bootstrap()
    docstring for the full rationale."""

    def test_requires_envelope_parameter(self):
        z_probe = FakeZOffsetProbe(is_calibrated=True)
        probe_obj = FakeProbeObj(z_offset=1.795)
        zc = FakeZCompensate(110., 111.)
        printer, gcode, coord = _build(z_probe, probe_obj, zc)
        with self.assertRaises(fake.CommandError) as ctx:
            coord.cmd_z_offset_calibrate(
                fake.FakeGCmd({'SIMULATE_BOOTSTRAP': '1'}))
        self.assertIn('ENVELOPE', str(ctx.exception))

    def test_forces_bootstrap_classification_regardless_of_real_offset(self):
        # probe_obj has a real, credible, already-calibrated z_offset
        # (1.795) - SIMULATE_BOOTSTRAP must still force the BOOTSTRAP path,
        # and must never read this value at all.
        z_probe = FakeZOffsetProbe(is_calibrated=True)
        probe_obj = FakeProbeObj(z_offset=1.795)
        zc = FakeZCompensate(110., 111.)
        printer, gcode, coord = _build(z_probe, probe_obj, zc)
        seen = {}

        def fake_measure(printer_, x, y, probe_x_offset, probe_y_offset,
                          probe_z_offset, *a, **kw):
            seen['probe_z_offset'] = probe_z_offset
            seen['bootstrap_contact_envelope_mm'] = kw.get(
                'bootstrap_contact_envelope_mm')
            seen['established_contact_margin_mm'] = kw.get(
                'established_contact_margin_mm')
            return _accepted_measurement(
                x, y, probe_trigger_z=2.0, nozzle_contact_z=-5.0,
                contact_mode='bootstrap')
        orig = nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair
        nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = fake_measure
        try:
            coord.cmd_z_offset_calibrate(
                fake.FakeGCmd({'SIMULATE_BOOTSTRAP': '1', 'ENVELOPE': '7.0'}))
        finally:
            nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = orig
        self.assertEqual(seen['probe_z_offset'], 0.0)
        self.assertAlmostEqual(seen['bootstrap_contact_envelope_mm'], 7.0, places=9)
        # established_contact_margin_mm is not passed through by the
        # simulation call at all (bootstrap doesn't use it).
        self.assertIsNone(seen['established_contact_margin_mm'])
        # Real probe offset must be completely untouched.
        self.assertAlmostEqual(probe_obj.probe_offsets.z_offset, 1.795, places=9)

    def test_never_applies_or_stages_result(self):
        z_probe = FakeZOffsetProbe(is_calibrated=True)
        probe_obj = FakeProbeObj(z_offset=1.795)
        zc = FakeZCompensate(110., 111.)
        printer, gcode, coord = _build(z_probe, probe_obj, zc)

        def fake_measure(printer_, x, y, *a, **kw):
            return _accepted_measurement(
                x, y, probe_trigger_z=2.0, nozzle_contact_z=-5.0,
                contact_mode='bootstrap')
        orig = nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair
        nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = fake_measure
        try:
            coord.cmd_z_offset_calibrate(
                fake.FakeGCmd({'SIMULATE_BOOTSTRAP': '1', 'ENVELOPE': '7.0'}))
        finally:
            nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = orig
        self.assertAlmostEqual(probe_obj.probe_offsets.z_offset, 1.795, places=9)
        configfile = printer.lookup_object('configfile')
        self.assertEqual(configfile.set_calls, [])
        # And the REAL z_offset_* status namespace is completely untouched.
        self.assertEqual(coord.z_offset_state, 'idle')
        self.assertIsNone(coord.z_offset_result)
        self.assertEqual(coord.z_offset_id, 0)

    def test_result_recorded_in_bootstrap_sim_status_namespace(self):
        z_probe = FakeZOffsetProbe(is_calibrated=True)
        probe_obj = FakeProbeObj(z_offset=1.795)
        zc = FakeZCompensate(110., 111.)
        printer, gcode, coord = _build(z_probe, probe_obj, zc)

        def fake_measure(printer_, x, y, *a, **kw):
            return _accepted_measurement(
                x, y, probe_trigger_z=2.0, nozzle_contact_z=-5.0,
                contact_mode='bootstrap')
        orig = nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair
        nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = fake_measure
        try:
            coord.cmd_z_offset_calibrate(
                fake.FakeGCmd({'SIMULATE_BOOTSTRAP': '1', 'ENVELOPE': '7.0'}))
        finally:
            nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = orig
        status = coord.get_status(0.)
        self.assertEqual(status['bootstrap_sim_state'], 'complete')
        self.assertEqual(status['bootstrap_sim_id'], 1)
        self.assertAlmostEqual(status['bootstrap_sim_result'], 7.0, places=9)
        self.assertAlmostEqual(status['bootstrap_sim_envelope_mm'], 7.0, places=9)
        self.assertIsNone(status['bootstrap_sim_error'])

    def test_rejected_measurement_is_not_applied_and_records_error(self):
        z_probe = FakeZOffsetProbe(is_calibrated=True)
        probe_obj = FakeProbeObj(z_offset=1.795)
        zc = FakeZCompensate(110., 111.)
        printer, gcode, coord = _build(z_probe, probe_obj, zc)

        def fake_measure(printer_, x, y, *a, **kw):
            return _rejected_measurement(x, y)
        orig = nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair
        nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = fake_measure
        try:
            with self.assertRaises(fake.CommandError):
                coord.cmd_z_offset_calibrate(
                    fake.FakeGCmd({'SIMULATE_BOOTSTRAP': '1', 'ENVELOPE': '7.0'}))
        finally:
            nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = orig
        status = coord.get_status(0.)
        self.assertEqual(status['bootstrap_sim_state'], 'error')
        self.assertIsNotNone(status['bootstrap_sim_error'])
        self.assertIsNone(status['bootstrap_sim_result'])
        self.assertAlmostEqual(probe_obj.probe_offsets.z_offset, 1.795, places=9)

    def test_unexpected_non_bootstrap_classification_is_an_internal_error(self):
        # Defense in depth: if measure_probe_nozzle_pair ever returned
        # something other than 'bootstrap' for a probe_z_offset=0.0 call,
        # that's a real bug in this simulation path, not a normal
        # rejection - must raise clearly rather than silently accept.
        z_probe = FakeZOffsetProbe(is_calibrated=True)
        probe_obj = FakeProbeObj(z_offset=1.795)
        zc = FakeZCompensate(110., 111.)
        printer, gcode, coord = _build(z_probe, probe_obj, zc)

        def fake_measure(printer_, x, y, *a, **kw):
            return _accepted_measurement(
                x, y, probe_trigger_z=2.0, nozzle_contact_z=0.5,
                contact_mode='established')
        orig = nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair
        nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = fake_measure
        try:
            with self.assertRaises(fake.CommandError) as ctx:
                coord.cmd_z_offset_calibrate(
                    fake.FakeGCmd({'SIMULATE_BOOTSTRAP': '1', 'ENVELOPE': '7.0'}))
        finally:
            nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = orig
        self.assertIn('internal error', str(ctx.exception))


class StatusTest(unittest.TestCase):
    def test_get_status_reflects_last_result(self):
        z_probe = FakeZOffsetProbe(is_calibrated=True)
        zc = FakeZCompensate(110., 111.)
        printer, gcode, coord = _build(z_probe, FakeProbeObj(), zc)
        orig = nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair
        nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = \
            _stub_pair(1.5, 0.5)
        try:
            coord.cmd_z_offset_calibrate(fake.FakeGCmd({}))
        finally:
            nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = orig
        status = coord.get_status(0.)
        self.assertEqual(status['z_offset_state'], 'complete')
        self.assertEqual(status['z_offset_id'], 1)
        self.assertAlmostEqual(status['z_offset_result'], 1.0, places=9)
        self.assertIsNone(status['z_offset_error'])

    def test_status_command_does_not_raise_when_idle(self):
        printer, gcode, coord = _build()
        coord.cmd_calibration_status(fake.FakeGCmd())  # must not raise


# ----------------------------------------------------------------------
# NEBULAOS_AUTO_CALIBRATE / NEBULAOS_CALIBRATION_CANCEL (mission §11/§12)
# ----------------------------------------------------------------------
import os
import shutil
import tempfile

from . import nebulaos_calibration_journal as calibration_journal


class RestartTriggered(Exception):
    """Simulates SAVE_CONFIG's real behavior: in production, klippy
    restarts and cmd_auto_calibrate() never returns from that call at all
    (see its own comment on why the line after it is unreachable in
    practice) - the real success signal a test can observe is what got
    written to the journal and to configfile.set_calls BEFORE this point,
    not a normal return from cmd_auto_calibrate() itself."""
    pass


class DispatchingFakeGCode(fake.FakeGCode):
    """Unlike the plain FakeGCode (inert - records but never executes),
    this dispatches each script's first word to self.commands if a
    handler is registered there, parsing simple KEY=value params.
    Required because cmd_auto_calibrate() deliberately calls OTHER
    commands via run_script_from_command() exactly like a real user would
    - including _NEBULAOS_NOZZLE_CLEAN, which in production is registered
    by a different class (z_compensate.py's ZCompensate) sharing the same
    real gcode object, and _NEBULAOS_Z_OFFSET_CALIBRATE, which IS this same
    coordinator's own already-tested command."""

    def run_script_from_command(self, script):
        self.scripts_run.append(script)
        parts = script.split()
        name = parts[0]
        if name == 'SAVE_CONFIG':
            raise RestartTriggered()
        handler = self.commands.get(name)
        if handler is None:
            return
        params = {}
        for tok in parts[1:]:
            if '=' in tok:
                k, v = tok.split('=', 1)
                params[k] = v
        handler(fake.FakeGCmd(params))


class FakeBedMesh:
    """Stands in for the real registered 'bed_mesh' object. profile_name
    is settable directly by a test to simulate BED_MESH_CALIBRATE having
    already run (that upstream algorithm is out of this module's own test
    scope - see nebulaos_calibration.py's own header). `profiles` models
    upstream's own SAVED-profiles dict (populated from configfile on every
    boot regardless of which one, if any, is currently ACTIVE) - defaults
    to containing profile_name itself (matching real behavior right after
    a calibration run, before any restart); a test proving the post-
    restart "saved but not active" case passes profile_name='' with an
    explicit non-empty `profiles` instead."""

    def __init__(self, profile_name='default', profiles=None):
        self.profile_name = profile_name
        if profiles is not None:
            self.profiles = profiles
        elif profile_name:
            self.profiles = {profile_name: {'points': []}}
        else:
            self.profiles = {}

    def get_status(self, eventtime):
        return {'profile_name': self.profile_name, 'profiles': self.profiles}


def _build_auto_calibrate(z_offset_probe=None, probe_obj=None,
                           bed_mesh=None,
                           config_overrides=None):
    printer = fake.FakePrinter()
    gcode = DispatchingFakeGCode()
    printer.add_object('gcode', gcode)
    printer.add_object('probe', probe_obj if probe_obj is not None else FakeProbeObj())
    printer.add_object('configfile', FakeConfigFile())
    if z_offset_probe is not None:
        printer.add_object('nebulaos_z_offset_probe', z_offset_probe)
    printer.add_object('bed_mesh', bed_mesh if bed_mesh is not None else FakeBedMesh())

    values = dict(config_overrides or {})
    values.setdefault('reference_x', '20')
    values.setdefault('reference_y', '25')
    config = fake.FakeConfig(values, section='nebulaos_calibration', printer=printer)
    coordinator = nebulaos_calibration.NebulaOSCalibration(config)
    config.assert_all_consumed()
    return printer, gcode, coordinator


class AutoCalibrateTest(unittest.TestCase):
    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()
        self.journal_path = os.path.join(self.tmpdir, 'journal.json')

    def tearDown(self):
        shutil.rmtree(self.tmpdir, ignore_errors=True)

    def _build(self, nozzle_clean_ok=True, extra_overrides=None):
        overrides = {'journal_path': self.journal_path}
        overrides.update(extra_overrides or {})
        z_probe = FakeZOffsetProbe(is_calibrated=True)
        probe_obj = FakeProbeObj(z_offset=1.795)
        printer, gcode, coord = _build_auto_calibrate(
            z_probe, probe_obj, config_overrides=overrides)

        def fake_nozzle_clean(gcmd):
            if not nozzle_clean_ok:
                raise fake.CommandError("_NEBULAOS_NOZZLE_CLEAN: simulated failure")
        gcode.commands['_NEBULAOS_NOZZLE_CLEAN'] = fake_nozzle_clean

        orig = nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair
        nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = \
            _stub_pair(probe_trigger_z=2.5, nozzle_contact_z=0.831)
        self.addCleanup(
            setattr, nebulaos_calibration.nebulaos_probe_pair,
            'measure_probe_nozzle_pair', orig)
        return printer, gcode, coord

    def test_full_happy_path_runs_stages_in_order_and_commits(self):
        printer, gcode, coord = self._build()
        with self.assertRaises(RestartTriggered):
            coord.cmd_auto_calibrate(fake.FakeGCmd({}))

        script_names = [s.split()[0] for s in gcode.scripts_run]
        self.assertEqual(script_names, [
            'G28', 'M140', '_NEBULAOS_NOZZLE_CLEAN',
            'M104', 'M190', 'M109', 'G4',
            '_NEBULAOS_Z_OFFSET_CALIBRATE', 'BED_MESH_CALIBRATE',
            'SAVE_CONFIG',
        ])
        self.assertNotIn('PID_CALIBRATE', script_names)
        # Bed preheated before nozzle_clean; establish_thermal_state
        # confirms exact temps (nozzle at z_offset_reference_temp 150C,
        # bed at pid_bed_target 65C).
        self.assertEqual(gcode.scripts_run[1], 'M140 S65.0')
        self.assertEqual(gcode.scripts_run[3], 'M104 S150.0')
        self.assertEqual(gcode.scripts_run[4], 'M190 S65.0')
        self.assertEqual(gcode.scripts_run[5], 'M109 S150.0')

        journal = calibration_journal.read_journal(path=self.journal_path)
        self.assertEqual(journal['state'], calibration_journal.STATE_COMMIT_REQUESTED)
        self.assertTrue(journal['commit_requested'])
        self.assertTrue(journal['restart_pending'])
        self.assertTrue(journal['verification_pending'])
        self.assertAlmostEqual(
            journal['expected_values']['bltouch.z_offset'], 1.669, places=3)
        self.assertEqual(journal['expected_values']['bed_mesh.profile'], 'default')
        self.assertEqual(coord.auto_calibrate_state, 'running')

    def test_preflight_rejects_missing_load_cell_before_any_motion(self):
        printer, gcode, coord = self._build()
        printer.objects.pop('nebulaos_z_offset_probe')
        with self.assertRaises(fake.CommandError):
            coord.cmd_auto_calibrate(fake.FakeGCmd({}))
        self.assertEqual(gcode.scripts_run, [])
        self.assertEqual(coord.auto_calibrate_state, 'error')
        journal = calibration_journal.read_journal(path=self.journal_path)
        self.assertEqual(journal['state'], calibration_journal.STATE_ERROR)
        self.assertFalse(journal['commit_requested'])

    def test_nozzle_clean_failure_aborts_before_heating_to_calibration_temp(self):
        printer, gcode, coord = self._build(nozzle_clean_ok=False)
        with self.assertRaises(fake.CommandError):
            coord.cmd_auto_calibrate(fake.FakeGCmd({}))
        script_names = [s.split()[0] for s in gcode.scripts_run]
        self.assertEqual(
            script_names, ['G28', 'M140', '_NEBULAOS_NOZZLE_CLEAN'])
        self.assertNotIn('PID_CALIBRATE', script_names)
        journal = calibration_journal.read_journal(path=self.journal_path)
        self.assertEqual(journal['state'], calibration_journal.STATE_ERROR)
        self.assertFalse(journal['commit_requested'])
        self.assertFalse(journal['restart_pending'])

    def test_z_offset_failure_never_reaches_bed_mesh_or_commit(self):
        printer, gcode, coord = self._build()
        orig = nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair
        nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = \
            lambda printer_, x, y, *a, **kw: _rejected_measurement(x, y)
        try:
            with self.assertRaises(fake.CommandError):
                coord.cmd_auto_calibrate(fake.FakeGCmd({}))
        finally:
            nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = orig
        script_names = [s.split()[0] for s in gcode.scripts_run]
        self.assertNotIn('BED_MESH_CALIBRATE', script_names)
        self.assertNotIn('SAVE_CONFIG', script_names)
        journal = calibration_journal.read_journal(path=self.journal_path)
        self.assertEqual(journal['state'], calibration_journal.STATE_ERROR)

    def test_final_validation_fails_closed_with_no_active_mesh_profile(self):
        printer, gcode, coord = self._build()
        printer.objects['bed_mesh'] = FakeBedMesh(profile_name=None)
        with self.assertRaises(fake.CommandError) as ctx:
            coord.cmd_auto_calibrate(fake.FakeGCmd({}))
        self.assertIn('final validation', str(ctx.exception))
        self.assertNotIn('SAVE_CONFIG', [s.split()[0] for s in gcode.scripts_run])
        journal = calibration_journal.read_journal(path=self.journal_path)
        self.assertEqual(journal['state'], calibration_journal.STATE_ERROR)
        self.assertFalse(journal['commit_requested'])

    def test_cancel_before_running_is_rejected(self):
        printer, gcode, coord = self._build()
        with self.assertRaises(fake.CommandError):
            coord.cmd_calibration_cancel(fake.FakeGCmd({}))

    def test_cancel_takes_effect_at_next_stage_boundary(self):
        printer, gcode, coord = self._build()
        # Cancel arrives from a stub registered on G28 itself - simulates
        # NEBULAOS_CALIBRATION_CANCEL being called by a separate, real
        # gcode dispatch while the 'home' stage is still in progress
        # (cmd_auto_calibrate() only checks the flag at the NEXT stage
        # boundary - immediately after G28 returns, before starting
        # nozzle_clean - never mid-stage). auto_calibrate_state must already
        # read 'running' at the point CANCEL is accepted, matching a real
        # concurrent CANCEL call's own preflight check.
        def cancel_during_home(gcmd):
            self.assertEqual(coord.auto_calibrate_state, 'running')
            coord.cmd_calibration_cancel(fake.FakeGCmd({}))
        gcode.commands['G28'] = cancel_during_home

        with self.assertRaises(fake.CommandError) as ctx:
            coord.cmd_auto_calibrate(fake.FakeGCmd({}))
        self.assertIn('cancelled', str(ctx.exception))
        script_names = [s.split()[0] for s in gcode.scripts_run]
        self.assertEqual(script_names, ['G28'])
        self.assertEqual(coord.auto_calibrate_state, 'cancelled')
        journal = calibration_journal.read_journal(path=self.journal_path)
        self.assertEqual(journal['state'], calibration_journal.STATE_CANCELLED)

    def test_cannot_start_second_run_while_one_is_in_progress(self):
        printer, gcode, coord = self._build()
        coord.auto_calibrate_state = 'running'
        with self.assertRaises(fake.CommandError):
            coord.cmd_auto_calibrate(fake.FakeGCmd({}))


class NozzleContaminationFixOrderingTest(unittest.TestCase):
    """Mission root-cause fix (2026-09-02/03): a hot nozzle-clean cycle
    biases the load-cell Z-offset contact low by ~0.6mm (nozzle ooze/
    contamination) - see _evidence/phase2-live-full-stack-closure-
    20260902-180602/07-nozzle-contamination-test/ for the live A/B proof.
    Fix is sequencing/thermal-reference only: localized_z_offset must be
    measured at z_offset_reference_temp (150C default), never at the
    extruder's full printing temperature. The contact algorithm and safety
    envelope are untouched by this fix - these tests only prove command
    ORDER and TEMPERATURE, nothing about touch_probe() or
    measure_probe_nozzle_pair()."""

    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()
        self.journal_path = os.path.join(self.tmpdir, 'journal.json')

    def tearDown(self):
        shutil.rmtree(self.tmpdir, ignore_errors=True)

    def _build(self):
        overrides = {'journal_path': self.journal_path}
        z_probe = FakeZOffsetProbe(is_calibrated=True)
        probe_obj = FakeProbeObj(z_offset=1.155)
        printer, gcode, coord = _build_auto_calibrate(
            z_probe, probe_obj, config_overrides=overrides)
        gcode.commands['_NEBULAOS_NOZZLE_CLEAN'] = lambda gcmd: None
        orig = nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair
        nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = \
            _stub_pair(probe_trigger_z=2.5, nozzle_contact_z=0.831)
        self.addCleanup(
            setattr, nebulaos_calibration.nebulaos_probe_pair,
            'measure_probe_nozzle_pair', orig)
        return printer, gcode, coord

    def test_exact_stage_order_nozzle_clean_to_localized_z_offset(self):
        printer, gcode, coord = self._build()
        with self.assertRaises(RestartTriggered):
            coord.cmd_auto_calibrate(fake.FakeGCmd({}))
        scripts = gcode.scripts_run
        script_names = [s.split()[0] for s in scripts]
        nozzle_clean_idx = script_names.index('_NEBULAOS_NOZZLE_CLEAN')
        z_offset_idx = script_names.index('_NEBULAOS_Z_OFFSET_CALIBRATE')
        sub_sequence = script_names[nozzle_clean_idx:z_offset_idx + 1]
        self.assertEqual(sub_sequence, [
            '_NEBULAOS_NOZZLE_CLEAN',
            'M104', 'M190', 'M109', 'G4',
            '_NEBULAOS_Z_OFFSET_CALIBRATE',
        ])

    def test_reference_temp_is_150_not_230(self):
        printer, gcode, coord = self._build()
        with self.assertRaises(RestartTriggered):
            coord.cmd_auto_calibrate(fake.FakeGCmd({}))
        m104 = [s for s in gcode.scripts_run if s.startswith('M104')]
        m109 = [s for s in gcode.scripts_run if s.startswith('M109')]
        self.assertEqual(m104, ['M104 S150.0'])
        self.assertEqual(m109, ['M109 S150.0'])

    def test_no_230_reheat_between_nozzle_clean_and_z_offset(self):
        printer, gcode, coord = self._build()
        with self.assertRaises(RestartTriggered):
            coord.cmd_auto_calibrate(fake.FakeGCmd({}))
        scripts = gcode.scripts_run
        nozzle_clean_idx = scripts.index('_NEBULAOS_NOZZLE_CLEAN')
        z_offset_idx = scripts.index('_NEBULAOS_Z_OFFSET_CALIBRATE')
        between = scripts[nozzle_clean_idx + 1:z_offset_idx]
        hotend_commands = [s for s in between if s.startswith(('M104', 'M109'))]
        self.assertEqual(hotend_commands, ['M104 S150.0', 'M109 S150.0'])
        self.assertNotIn('230', ' '.join(hotend_commands))

    def test_no_pid_in_auto_calibrate_sequence(self):
        printer, gcode, coord = self._build()
        with self.assertRaises(RestartTriggered):
            coord.cmd_auto_calibrate(fake.FakeGCmd({}))
        pid_calls = [s for s in gcode.scripts_run
                     if s.startswith('PID_CALIBRATE')]
        self.assertEqual(pid_calls, [])

    def test_bed_stays_at_intended_calibration_temp(self):
        # Bed reference is unaffected by the nozzle-temperature fix.
        printer, gcode, coord = self._build()
        with self.assertRaises(RestartTriggered):
            coord.cmd_auto_calibrate(fake.FakeGCmd({}))
        m140 = [s for s in gcode.scripts_run if s.startswith('M140')]
        m190 = [s for s in gcode.scripts_run if s.startswith('M190')]
        self.assertEqual(m140, ['M140 S65.0'])
        self.assertEqual(m190, ['M190 S65.0'])

    def test_z_offset_reference_temp_is_configurable(self):
        printer, gcode, coord = _build_auto_calibrate(
            FakeZOffsetProbe(is_calibrated=True),
            FakeProbeObj(z_offset=1.155),
            config_overrides={'journal_path': self.journal_path,
                               'z_offset_reference_temp': '160'})
        gcode.commands['_NEBULAOS_NOZZLE_CLEAN'] = lambda gcmd: None
        orig = nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair
        nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = \
            _stub_pair(probe_trigger_z=2.5, nozzle_contact_z=0.831)
        try:
            with self.assertRaises(RestartTriggered):
                coord.cmd_auto_calibrate(fake.FakeGCmd({}))
        finally:
            nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = orig
        m104 = [s for s in gcode.scripts_run if s.startswith('M104')]
        self.assertEqual(m104, ['M104 S160.0'])


class PostRestartVerificationTest(unittest.TestCase):
    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()
        self.journal_path = os.path.join(self.tmpdir, 'journal.json')

    def tearDown(self):
        shutil.rmtree(self.tmpdir, ignore_errors=True)

    def _write_pending_journal(self, expected_values):
        j = calibration_journal.new_journal(1, 'auto_calibrate', now=0.0)
        calibration_journal.mark_commit_requested(j, expected_values, now=1.0)
        calibration_journal.write_journal(j, path=self.journal_path)
        return j

    def test_matching_config_after_restart_marks_complete(self):
        self._write_pending_journal(
            {'bltouch.z_offset': 1.234, 'bed_mesh.profile': 'default'})
        probe_obj = FakeProbeObj(z_offset=1.234)
        printer, gcode, coord = _build_auto_calibrate(
            probe_obj=probe_obj, bed_mesh=FakeBedMesh(profile_name='default'),
            config_overrides={'journal_path': self.journal_path})
        printer.send_event('klippy:ready')
        journal = calibration_journal.read_journal(path=self.journal_path)
        self.assertEqual(journal['state'], calibration_journal.STATE_COMPLETE)
        self.assertFalse(journal['verification_pending'])
        self.assertEqual(coord.auto_calibrate_state, calibration_journal.STATE_COMPLETE)

    def test_mismatched_z_offset_after_restart_marks_error(self):
        self._write_pending_journal(
            {'bltouch.z_offset': 1.234, 'bed_mesh.profile': 'default'})
        probe_obj = FakeProbeObj(z_offset=9.999)
        printer, gcode, coord = _build_auto_calibrate(
            probe_obj=probe_obj, bed_mesh=FakeBedMesh(profile_name='default'),
            config_overrides={'journal_path': self.journal_path})
        printer.send_event('klippy:ready')
        journal = calibration_journal.read_journal(path=self.journal_path)
        self.assertEqual(journal['state'], calibration_journal.STATE_ERROR)
        self.assertIn('bltouch.z_offset', journal['error'])

    def test_mismatched_bed_mesh_profile_after_restart_marks_error(self):
        self._write_pending_journal(
            {'bltouch.z_offset': 1.234, 'bed_mesh.profile': 'default'})
        probe_obj = FakeProbeObj(z_offset=1.234)
        printer, gcode, coord = _build_auto_calibrate(
            probe_obj=probe_obj, bed_mesh=FakeBedMesh(profile_name='some_other_profile'),
            config_overrides={'journal_path': self.journal_path})
        printer.send_event('klippy:ready')
        journal = calibration_journal.read_journal(path=self.journal_path)
        self.assertEqual(journal['state'], calibration_journal.STATE_ERROR)
        self.assertIn('bed_mesh.profile', journal['error'])

    def test_saved_but_not_currently_active_profile_still_verifies_complete(self):
        # Root-caused live 2026-09-03: upstream bed_mesh does not
        # auto-load any saved profile as ACTIVE at klippy startup on its
        # own (that's a print-time step, e.g. START_PRINT's own explicit
        # BED_MESH_PROFILE LOAD=<name>) - a real AUTO_CALIBRATE run saved
        # a genuine mesh under 'default' (present in bed_mesh's own
        # `profiles` status field after restart) but profile_name/
        # mesh_matrix stayed empty since nothing had loaded it yet. This
        # must verify as COMPLETE, not error - the original check (which
        # compared against profile_name) produced a false-negative here.
        self._write_pending_journal(
            {'bltouch.z_offset': 1.234, 'bed_mesh.profile': 'default'})
        probe_obj = FakeProbeObj(z_offset=1.234)
        bed_mesh = FakeBedMesh(
            profile_name='', profiles={'default': {'points': [[1, 2], [3, 4]]}})
        printer, gcode, coord = _build_auto_calibrate(
            probe_obj=probe_obj, bed_mesh=bed_mesh,
            config_overrides={'journal_path': self.journal_path})
        printer.send_event('klippy:ready')
        journal = calibration_journal.read_journal(path=self.journal_path)
        self.assertEqual(journal['state'], calibration_journal.STATE_COMPLETE)
        self.assertIsNone(journal['error'])

    def test_profile_missing_entirely_from_saved_profiles_marks_error(self):
        self._write_pending_journal(
            {'bltouch.z_offset': 1.234, 'bed_mesh.profile': 'default'})
        probe_obj = FakeProbeObj(z_offset=1.234)
        bed_mesh = FakeBedMesh(profile_name='', profiles={})
        printer, gcode, coord = _build_auto_calibrate(
            probe_obj=probe_obj, bed_mesh=bed_mesh,
            config_overrides={'journal_path': self.journal_path})
        printer.send_event('klippy:ready')
        journal = calibration_journal.read_journal(path=self.journal_path)
        self.assertEqual(journal['state'], calibration_journal.STATE_ERROR)
        self.assertIn('bed_mesh.profile', journal['error'])

    def test_no_journal_file_is_a_silent_no_op(self):
        printer, gcode, coord = _build_auto_calibrate(
            config_overrides={'journal_path': self.journal_path})
        printer.send_event('klippy:ready')  # must not raise
        self.assertIsNone(calibration_journal.read_journal(path=self.journal_path))

    def test_already_complete_journal_is_left_untouched(self):
        j = calibration_journal.new_journal(1, 'auto_calibrate', now=0.0)
        calibration_journal.mark_commit_requested(j, {}, now=1.0)
        calibration_journal.mark_verification_result(
            j, success=True, result={}, error=None, now=2.0)
        calibration_journal.write_journal(j, path=self.journal_path)
        before = calibration_journal.read_journal(path=self.journal_path)
        printer, gcode, coord = _build_auto_calibrate(
            config_overrides={'journal_path': self.journal_path})
        printer.send_event('klippy:ready')
        after = calibration_journal.read_journal(path=self.journal_path)
        self.assertEqual(before, after)


# ----------------------------------------------------------------------
# NEBULAOS_INPUT_SHAPER_CALIBRATE (mission §13)
# ----------------------------------------------------------------------
class FakeAxisShaperParams:
    def __init__(self, shaper_type='mzv', shaper_freq=0.0):
        self.shaper_type = shaper_type
        self.shaper_freq = shaper_freq


class FakeAxisShaper:
    def __init__(self, axis, shaper_type='mzv', shaper_freq=0.0):
        self.axis = axis
        self.params = FakeAxisShaperParams(shaper_type, shaper_freq)


class FakeInputShaper:
    """Stands in for the real registered 'input_shaper' object. Its own
    x/y AxisInputShaper-alikes start with shaper_freq=0.0 (upstream's own
    real default when nothing has ever been calibrated) - a test's fake
    SHAPER_CALIBRATE handler mutates .params in place, exactly like real
    ShaperCalibrate.apply_params()'s SET_INPUT_SHAPER call does."""

    def __init__(self):
        self.x = FakeAxisShaper('x')
        self.y = FakeAxisShaper('y')
        self.z = FakeAxisShaper('z')

    def get_shapers(self):
        return [self.x, self.y, self.z]


def _build_input_shaper_calibrate(
        config_overrides=None, shaper_calibrate_ok=True,
        result_x=('mzv', 45.3), result_y=('ei', 38.7),
        include_resonance_tester=True, include_input_shaper=True):
    printer = fake.FakePrinter()
    gcode = DispatchingFakeGCode()
    printer.add_object('gcode', gcode)
    printer.add_object('probe', FakeProbeObj())
    printer.add_object('configfile', FakeConfigFile())
    if include_resonance_tester:
        printer.add_object('resonance_tester', object())
    input_shaper_obj = FakeInputShaper()
    if include_input_shaper:
        printer.add_object('input_shaper', input_shaper_obj)

    def fake_shaper_calibrate(gcmd):
        if not shaper_calibrate_ok:
            raise fake.CommandError("SHAPER_CALIBRATE: simulated failure")
        axis = (gcmd.get('AXIS', '') or '').upper()
        if axis == 'X':
            input_shaper_obj.x.params.shaper_type = result_x[0]
            input_shaper_obj.x.params.shaper_freq = result_x[1]
        elif axis == 'Y':
            input_shaper_obj.y.params.shaper_type = result_y[0]
            input_shaper_obj.y.params.shaper_freq = result_y[1]
    gcode.commands['SHAPER_CALIBRATE'] = fake_shaper_calibrate

    values = dict(config_overrides or {})
    values.setdefault('reference_x', '20')
    values.setdefault('reference_y', '25')
    config = fake.FakeConfig(values, section='nebulaos_calibration', printer=printer)
    coordinator = nebulaos_calibration.NebulaOSCalibration(config)
    config.assert_all_consumed()
    return printer, gcode, coordinator, input_shaper_obj


class InputShaperCalibrateTest(unittest.TestCase):
    """NEBULAOS_INPUT_SHAPER_CALIBRATE's real, 3-call guided workflow for
    a MOVABLE clip-on accelerometer (corrected live 2026-09-03 - the
    operator confirmed the sensor is not fixed-mounted, contradicting an
    earlier draft's assumption): plain call (home) -> CONTINUE=1 (measure
    X, ask for the bed move) -> CONTINUE=1 (measure Y, commit)."""

    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()
        self.journal_path = os.path.join(self.tmpdir, 'input_shaper_journal.json')

    def tearDown(self):
        shutil.rmtree(self.tmpdir, ignore_errors=True)

    def _build(self, **kwargs):
        overrides = {'input_shaper_journal_path': self.journal_path}
        overrides.update(kwargs.pop('config_overrides', None) or {})
        return _build_input_shaper_calibrate(
            config_overrides=overrides, **kwargs)

    def test_first_call_homes_and_asks_for_toolhead_mount(self):
        printer, gcode, coord, _ = self._build()
        coord.cmd_input_shaper_calibrate(fake.FakeGCmd({}))
        self.assertEqual(gcode.scripts_run, ['G28'])
        self.assertEqual(coord.input_shaper_state, 'awaiting_x_mount')
        journal = calibration_journal.read_journal(path=self.journal_path)
        self.assertEqual(journal['stage'], 'home')

    def test_second_call_measures_x_and_asks_for_bed_move(self):
        printer, gcode, coord, _ = self._build(result_x=('mzv', 45.3))
        coord.cmd_input_shaper_calibrate(fake.FakeGCmd({}))
        coord.cmd_input_shaper_calibrate(fake.FakeGCmd({'CONTINUE': '1'}))
        self.assertEqual(gcode.scripts_run, ['G28', 'SHAPER_CALIBRATE AXIS=X'])
        self.assertEqual(coord.input_shaper_state, 'awaiting_y_mount')
        journal = calibration_journal.read_journal(path=self.journal_path)
        self.assertEqual(journal['stage'], 'measure_x')

    def test_third_call_measures_y_and_commits(self):
        printer, gcode, coord, _ = self._build(
            result_x=('mzv', 45.3), result_y=('ei', 38.7))
        coord.cmd_input_shaper_calibrate(fake.FakeGCmd({}))
        coord.cmd_input_shaper_calibrate(fake.FakeGCmd({'CONTINUE': '1'}))
        with self.assertRaises(RestartTriggered):
            coord.cmd_input_shaper_calibrate(fake.FakeGCmd({'CONTINUE': '1'}))
        self.assertEqual(gcode.scripts_run,
                          ['G28', 'SHAPER_CALIBRATE AXIS=X',
                           'SHAPER_CALIBRATE AXIS=Y', 'SAVE_CONFIG'])
        journal = calibration_journal.read_journal(path=self.journal_path)
        self.assertEqual(
            journal['state'], calibration_journal.STATE_COMMIT_REQUESTED)
        self.assertEqual(journal['expected_values'], {
            'input_shaper.shaper_type_x': 'mzv',
            'input_shaper.shaper_freq_x': 45.3,
            'input_shaper.shaper_type_y': 'ei',
            'input_shaper.shaper_freq_y': 38.7,
        })
        self.assertEqual(coord.input_shaper_stage, 'commit')

    def test_continue_without_a_start_is_rejected(self):
        printer, gcode, coord, _ = self._build()
        with self.assertRaises(fake.CommandError):
            coord.cmd_input_shaper_calibrate(fake.FakeGCmd({'CONTINUE': '1'}))
        self.assertEqual(gcode.scripts_run, [])

    def test_missing_resonance_tester_raises_before_any_motion(self):
        printer, gcode, coord, _ = self._build(include_resonance_tester=False)
        with self.assertRaises(fake.CommandError):
            coord.cmd_input_shaper_calibrate(fake.FakeGCmd({}))
        self.assertEqual(gcode.scripts_run, [])
        self.assertEqual(coord.input_shaper_state, 'error')

    def test_missing_input_shaper_object_raises_before_any_motion(self):
        printer, gcode, coord, _ = self._build(include_input_shaper=False)
        with self.assertRaises(fake.CommandError):
            coord.cmd_input_shaper_calibrate(fake.FakeGCmd({}))
        self.assertEqual(gcode.scripts_run, [])

    def test_zero_freq_result_on_x_is_rejected_before_asking_for_bed_move(self):
        printer, gcode, coord, _ = self._build(result_x=('mzv', 0.0))
        coord.cmd_input_shaper_calibrate(fake.FakeGCmd({}))
        with self.assertRaises(fake.CommandError):
            coord.cmd_input_shaper_calibrate(fake.FakeGCmd({'CONTINUE': '1'}))
        self.assertEqual(coord.input_shaper_state, 'error')
        journal = calibration_journal.read_journal(path=self.journal_path)
        self.assertEqual(journal['state'], calibration_journal.STATE_ERROR)

    def test_zero_freq_result_on_y_is_rejected_before_commit(self):
        printer, gcode, coord, _ = self._build(result_y=('mzv', 0.0))
        coord.cmd_input_shaper_calibrate(fake.FakeGCmd({}))
        coord.cmd_input_shaper_calibrate(fake.FakeGCmd({'CONTINUE': '1'}))
        with self.assertRaises(fake.CommandError):
            coord.cmd_input_shaper_calibrate(fake.FakeGCmd({'CONTINUE': '1'}))
        self.assertNotIn('SAVE_CONFIG', gcode.scripts_run)
        self.assertEqual(coord.input_shaper_state, 'error')
        journal = calibration_journal.read_journal(path=self.journal_path)
        self.assertEqual(journal['state'], calibration_journal.STATE_ERROR)

    def test_shaper_calibrate_failure_on_x_never_asks_for_bed_move(self):
        printer, gcode, coord, _ = self._build(shaper_calibrate_ok=False)
        coord.cmd_input_shaper_calibrate(fake.FakeGCmd({}))
        with self.assertRaises(fake.CommandError):
            coord.cmd_input_shaper_calibrate(fake.FakeGCmd({'CONTINUE': '1'}))
        self.assertEqual(gcode.scripts_run, ['G28', 'SHAPER_CALIBRATE AXIS=X'])
        self.assertEqual(coord.input_shaper_state, 'error')
        journal = calibration_journal.read_journal(path=self.journal_path)
        self.assertEqual(journal['state'], calibration_journal.STATE_ERROR)

    def test_cannot_start_second_run_while_awaiting_a_sensor_move(self):
        printer, gcode, coord, _ = self._build()
        coord.cmd_input_shaper_calibrate(fake.FakeGCmd({}))
        with self.assertRaises(fake.CommandError):
            coord.cmd_input_shaper_calibrate(fake.FakeGCmd({}))
        self.assertEqual(gcode.scripts_run, ['G28'])

    def test_can_start_a_new_run_after_a_real_restart(self):
        # A real restart replaces the whole process - the in-memory
        # input_shaper_state on the OLD coordinator instance staying at
        # whatever it was mid-flight (here 'awaiting_y_mount', since
        # nothing along the commit/SAVE_CONFIG path updates it further -
        # matching cmd_auto_calibrate's own identical behavior) is
        # correct and expected, not a bug: __init__ on the NEW instance
        # naturally resets to 'idle', which is what makes a next run
        # startable, not calling the old instance again.
        printer, gcode, coord, _ = self._build(
            result_x=('mzv', 45.3), result_y=('ei', 38.7))
        coord.cmd_input_shaper_calibrate(fake.FakeGCmd({}))
        coord.cmd_input_shaper_calibrate(fake.FakeGCmd({'CONTINUE': '1'}))
        with self.assertRaises(RestartTriggered):
            coord.cmd_input_shaper_calibrate(fake.FakeGCmd({'CONTINUE': '1'}))
        self.assertEqual(coord.input_shaper_state, 'awaiting_y_mount')

        fresh_printer, fresh_gcode, fresh_coord, _ = self._build(
            result_x=('mzv', 45.3), result_y=('ei', 38.7))
        self.assertEqual(fresh_coord.input_shaper_state, 'idle')
        fresh_coord.cmd_input_shaper_calibrate(fake.FakeGCmd({}))
        self.assertEqual(fresh_gcode.scripts_run, ['G28'])
        self.assertEqual(fresh_coord.input_shaper_state, 'awaiting_x_mount')

    def test_status_fields_present_and_idle_by_default(self):
        printer, gcode, coord, _ = self._build()
        status = coord.get_status(0.0)
        self.assertEqual(status['input_shaper_state'], 'idle')
        self.assertIsNone(status['input_shaper_stage'])
        self.assertIsNone(status['input_shaper_error'])
        self.assertIsNone(status['input_shaper_result'])


class InputShaperExplicitParamsTest(unittest.TestCase):
    """Mission Phase 2, overnight 2026-09-03/04 prep: this guided workflow
    must be able to own its own explicit FREQ_END=/ACCEL_PER_HZ= instead
    of accidentally depending on whatever [resonance_tester] itself
    happens to default to - proves the EXACT upstream command string
    generated in both the unset (unchanged, live-proven-safe) case and
    the explicit-parameters case. No resonance/shaper analysis is
    duplicated here - only command-string generation is tested."""

    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()
        self.journal_path = os.path.join(self.tmpdir, 'input_shaper_journal.json')

    def tearDown(self):
        shutil.rmtree(self.tmpdir, ignore_errors=True)

    def _build(self, shaper_freq_end=None, shaper_accel_per_hz=None, **kwargs):
        overrides = {'input_shaper_journal_path': self.journal_path}
        if shaper_freq_end is not None:
            overrides['shaper_freq_end'] = shaper_freq_end
        if shaper_accel_per_hz is not None:
            overrides['shaper_accel_per_hz'] = shaper_accel_per_hz
        overrides.update(kwargs.pop('config_overrides', None) or {})
        return _build_input_shaper_calibrate(config_overrides=overrides, **kwargs)

    def test_default_unset_produces_the_exact_bare_command(self):
        # Byte-for-byte the same command this workflow has always sent -
        # proves the new options are a pure no-op when left unconfigured.
        printer, gcode, coord, _ = self._build()
        self.assertEqual(
            coord._build_shaper_calibrate_script('x'), 'SHAPER_CALIBRATE AXIS=X')
        self.assertEqual(
            coord._build_shaper_calibrate_script('y'), 'SHAPER_CALIBRATE AXIS=Y')

    def test_freq_end_only(self):
        printer, gcode, coord, _ = self._build(shaper_freq_end='80')
        self.assertEqual(
            coord._build_shaper_calibrate_script('x'),
            'SHAPER_CALIBRATE AXIS=X FREQ_END=80.0')

    def test_accel_per_hz_only(self):
        printer, gcode, coord, _ = self._build(shaper_accel_per_hz='50')
        self.assertEqual(
            coord._build_shaper_calibrate_script('x'),
            'SHAPER_CALIBRATE AXIS=X ACCEL_PER_HZ=50.0')

    def test_both_params_together_x_and_y(self):
        printer, gcode, coord, _ = self._build(
            shaper_freq_end='80', shaper_accel_per_hz='50')
        self.assertEqual(
            coord._build_shaper_calibrate_script('x'),
            'SHAPER_CALIBRATE AXIS=X FREQ_END=80.0 ACCEL_PER_HZ=50.0')
        self.assertEqual(
            coord._build_shaper_calibrate_script('y'),
            'SHAPER_CALIBRATE AXIS=Y FREQ_END=80.0 ACCEL_PER_HZ=50.0')

    def test_full_guided_run_sends_the_explicit_command_for_both_axes(self):
        # End-to-end: the actual gcode scripts run_script_from_command
        # receives during a real 3-call guided sequence, not just the
        # builder in isolation.
        printer, gcode, coord, _ = self._build(
            shaper_freq_end='80', shaper_accel_per_hz='50',
            result_x=('mzv', 55.1), result_y=('mzv', 34.9))
        coord.cmd_input_shaper_calibrate(fake.FakeGCmd({}))
        coord.cmd_input_shaper_calibrate(fake.FakeGCmd({'CONTINUE': '1'}))
        with self.assertRaises(RestartTriggered):
            coord.cmd_input_shaper_calibrate(fake.FakeGCmd({'CONTINUE': '1'}))
        self.assertEqual(gcode.scripts_run, [
            'G28',
            'SHAPER_CALIBRATE AXIS=X FREQ_END=80.0 ACCEL_PER_HZ=50.0',
            'SHAPER_CALIBRATE AXIS=Y FREQ_END=80.0 ACCEL_PER_HZ=50.0',
            'SAVE_CONFIG',
        ])

    def test_config_values_are_not_silently_reused_as_defaults_elsewhere(self):
        # shaper_freq_end/shaper_accel_per_hz must only ever affect the
        # command this workflow itself builds - never [resonance_tester]'s
        # own config, which this module never touches.
        printer, gcode, coord, _ = self._build(
            shaper_freq_end='80', shaper_accel_per_hz='50')
        self.assertFalse(hasattr(coord, 'max_freq'))
        self.assertFalse(hasattr(coord, 'accel_per_hz'))


class InputShaperPostRestartVerificationTest(unittest.TestCase):
    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()
        self.journal_path = os.path.join(self.tmpdir, 'input_shaper_journal.json')

    def tearDown(self):
        shutil.rmtree(self.tmpdir, ignore_errors=True)

    def _write_pending_journal(self, expected_values):
        j = calibration_journal.new_journal(1, 'input_shaper_calibrate', now=0.0)
        calibration_journal.mark_commit_requested(j, expected_values, now=1.0)
        calibration_journal.write_journal(j, path=self.journal_path)
        return j

    def test_matching_shaper_params_after_restart_marks_complete(self):
        self._write_pending_journal({
            'input_shaper.shaper_type_x': 'mzv',
            'input_shaper.shaper_freq_x': 45.3,
            'input_shaper.shaper_type_y': 'ei',
            'input_shaper.shaper_freq_y': 38.7,
        })
        printer, gcode, coord, input_shaper_obj = _build_input_shaper_calibrate(
            config_overrides={'input_shaper_journal_path': self.journal_path})
        input_shaper_obj.x.params.shaper_type = 'mzv'
        input_shaper_obj.x.params.shaper_freq = 45.3
        input_shaper_obj.y.params.shaper_type = 'ei'
        input_shaper_obj.y.params.shaper_freq = 38.7
        printer.send_event('klippy:ready')
        journal = calibration_journal.read_journal(path=self.journal_path)
        self.assertEqual(journal['state'], calibration_journal.STATE_COMPLETE)
        self.assertEqual(coord.input_shaper_state, calibration_journal.STATE_COMPLETE)

    def test_mismatched_shaper_freq_after_restart_marks_error(self):
        self._write_pending_journal({
            'input_shaper.shaper_type_x': 'mzv',
            'input_shaper.shaper_freq_x': 45.3,
        })
        printer, gcode, coord, input_shaper_obj = _build_input_shaper_calibrate(
            config_overrides={'input_shaper_journal_path': self.journal_path})
        input_shaper_obj.x.params.shaper_type = 'mzv'
        input_shaper_obj.x.params.shaper_freq = 12.0
        printer.send_event('klippy:ready')
        journal = calibration_journal.read_journal(path=self.journal_path)
        self.assertEqual(journal['state'], calibration_journal.STATE_ERROR)
        self.assertIn('shaper_freq_x', journal['error'])

    def test_missing_input_shaper_object_after_restart_marks_error(self):
        self._write_pending_journal({'input_shaper.shaper_type_x': 'mzv'})
        printer, gcode, coord, _ = _build_input_shaper_calibrate(
            config_overrides={'input_shaper_journal_path': self.journal_path},
            include_input_shaper=False)
        printer.send_event('klippy:ready')
        journal = calibration_journal.read_journal(path=self.journal_path)
        self.assertEqual(journal['state'], calibration_journal.STATE_ERROR)

    def test_auto_calibrate_and_input_shaper_journals_are_independent(self):
        # A pending auto_calibrate journal at the DEFAULT journal_path must
        # not be disturbed by input-shaper verification, and vice versa -
        # they are two separate files by design (see input_shaper_journal_
        # path's own __init__ comment).
        # 1.795 matches FakeProbeObj()'s own default z_offset - _build_
        # input_shaper_calibrate() below has no probe_obj= parameter, so
        # this test uses that same default rather than introducing one.
        auto_journal_path = os.path.join(self.tmpdir, 'auto_journal.json')
        aj = calibration_journal.new_journal(1, 'auto_calibrate', now=0.0)
        calibration_journal.mark_commit_requested(
            aj, {'bltouch.z_offset': 1.795, 'bed_mesh.profile': 'default'},
            now=1.0)
        calibration_journal.write_journal(aj, path=auto_journal_path)
        self._write_pending_journal({
            'input_shaper.shaper_type_x': 'mzv',
            'input_shaper.shaper_freq_x': 45.3,
        })
        printer, gcode, coord, input_shaper_obj = _build_input_shaper_calibrate(
            config_overrides={
                'input_shaper_journal_path': self.journal_path,
                'journal_path': auto_journal_path,
            })
        printer.add_object('bed_mesh', FakeBedMesh(profile_name='default'))
        input_shaper_obj.x.params.shaper_type = 'mzv'
        input_shaper_obj.x.params.shaper_freq = 45.3
        printer.send_event('klippy:ready')
        auto_journal = calibration_journal.read_journal(path=auto_journal_path)
        input_shaper_journal = calibration_journal.read_journal(path=self.journal_path)
        self.assertEqual(auto_journal['state'], calibration_journal.STATE_COMPLETE)
        self.assertEqual(input_shaper_journal['state'], calibration_journal.STATE_COMPLETE)
        self.assertEqual(coord.auto_calibrate_state, calibration_journal.STATE_COMPLETE)
        self.assertEqual(coord.input_shaper_state, calibration_journal.STATE_COMPLETE)


# ----------------------------------------------------------------------
# NEBULAOS_ESTEPS_CALIBRATE (mission §14)
# ----------------------------------------------------------------------
class FakeStepper:
    def __init__(self, rotation_distance):
        self._rotation_distance = rotation_distance

    def get_rotation_distance(self):
        return (self._rotation_distance, 200)


class FakeExtruderStepper:
    def __init__(self, rotation_distance):
        self.stepper = FakeStepper(rotation_distance)


class FakeExtruderObj:
    """Stands in for the real registered extruder object
    (kinematics/extruder.py's PrinterExtruder) - models only
    extruder_stepper.stepper.get_rotation_distance(), the one real seam
    nebulaos_calibration.py's E-Steps workflow reads directly (matching
    upstream's own SET_EXTRUDER_ROTATION_DISTANCE, which reads/writes the
    exact same attribute)."""

    def __init__(self, rotation_distance=7.5, has_stepper=True):
        self.extruder_stepper = (
            FakeExtruderStepper(rotation_distance) if has_stepper else None)


def _build_esteps(extruder_obj=None, config_overrides=None):
    printer, gcode, coord = _build(config_overrides=config_overrides)
    printer.add_object(
        'extruder', extruder_obj if extruder_obj is not None else FakeExtruderObj())
    return printer, gcode, coord


class EStepsStartTest(unittest.TestCase):
    def test_happy_path_heats_and_waits_for_continue(self):
        printer, gcode, coord = _build_esteps()
        coord.cmd_esteps_calibrate(fake.FakeGCmd({}))
        self.assertEqual(coord.esteps_state, 'awaiting_continue')
        self.assertEqual(coord.esteps_id, 1)
        self.assertEqual(coord.esteps_extruder_name, 'extruder')
        self.assertEqual(gcode.scripts_run, ['M104 S200.0', 'M109 S200.0'])

    def test_missing_extruder_object_rejected(self):
        printer, gcode, coord = _build()  # no 'extruder' object added
        with self.assertRaises(fake.CommandError):
            coord.cmd_esteps_calibrate(fake.FakeGCmd({}))
        self.assertEqual(coord.esteps_state, 'idle')

    def test_extruder_without_rotation_distance_stepper_rejected(self):
        printer, gcode, coord = _build_esteps(
            FakeExtruderObj(has_stepper=False))
        with self.assertRaises(fake.CommandError):
            coord.cmd_esteps_calibrate(fake.FakeGCmd({}))

    def test_cannot_start_a_second_run_while_awaiting_continue(self):
        printer, gcode, coord = _build_esteps()
        coord.cmd_esteps_calibrate(fake.FakeGCmd({}))
        with self.assertRaises(fake.CommandError):
            coord.cmd_esteps_calibrate(fake.FakeGCmd({}))

    def test_cannot_start_a_second_run_while_awaiting_measurement(self):
        printer, gcode, coord = _build_esteps()
        coord.cmd_esteps_calibrate(fake.FakeGCmd({}))
        coord.cmd_esteps_calibrate(fake.FakeGCmd({'CONTINUE': '1'}))
        with self.assertRaises(fake.CommandError):
            coord.cmd_esteps_calibrate(fake.FakeGCmd({}))

    def test_custom_temp_is_honored(self):
        printer, gcode, coord = _build_esteps(
            config_overrides={'esteps_temp': '215'})
        coord.cmd_esteps_calibrate(fake.FakeGCmd({}))
        self.assertEqual(gcode.scripts_run, ['M104 S215.0', 'M109 S215.0'])

    def test_explicit_extruder_param_is_used_and_recorded(self):
        printer, gcode, coord = _build(config_overrides=None)
        printer.add_object('extruder1', FakeExtruderObj())
        coord.cmd_esteps_calibrate(fake.FakeGCmd({'EXTRUDER': 'extruder1'}))
        self.assertEqual(coord.esteps_extruder_name, 'extruder1')


class EStepsExtrudeTest(unittest.TestCase):
    def _started(self, extruder_obj=None, config_overrides=None):
        printer, gcode, coord = _build_esteps(extruder_obj, config_overrides)
        coord.cmd_esteps_calibrate(fake.FakeGCmd({}))
        gcode.scripts_run = []  # isolate CONTINUE=1's own scripts
        return printer, gcode, coord

    def test_continue_without_start_is_rejected(self):
        printer, gcode, coord = _build_esteps()
        with self.assertRaises(fake.CommandError):
            coord.cmd_esteps_calibrate(fake.FakeGCmd({'CONTINUE': '1'}))

    def test_continue_extrudes_exactly_the_commanded_length(self):
        printer, gcode, coord = self._started()
        coord.cmd_esteps_calibrate(fake.FakeGCmd({'CONTINUE': '1'}))
        self.assertEqual(coord.esteps_state, 'awaiting_measurement')
        self.assertEqual(gcode.scripts_run,
                          ['M82', 'G92 E0', 'G1 E100.0000 F300'])
        self.assertEqual(coord.esteps_commanded_length, 100.)

    def test_measurement_prompt_can_be_closed_for_on_screen_entry(self):
        # D3: GuppyScreen's E-Steps panel takes the measurement itself; the
        # prompt must offer a button that only closes the dialog.
        printer, gcode, coord = self._started()
        raw = []
        gcode.respond_raw = raw.append
        coord.cmd_esteps_calibrate(fake.FakeGCmd({'CONTINUE': '1'}))
        self.assertIn('// action:prompt_button Enter measurement|'
                      'RESPOND TYPE=command MSG=action:prompt_end', raw)
        self.assertIn('// action:prompt_button Cancel|_NEBULAOS_ESTEPS_CANCEL', raw)
        self.assertEqual(coord.esteps_state, 'awaiting_measurement')

    def test_records_old_rotation_distance_before_extruding(self):
        printer, gcode, coord = self._started(FakeExtruderObj(rotation_distance=7.5))
        coord.cmd_esteps_calibrate(fake.FakeGCmd({'CONTINUE': '1'}))
        self.assertAlmostEqual(coord.esteps_old_rotation_distance, 7.5, places=9)

    def test_custom_commanded_length_and_speed_are_honored(self):
        printer, gcode, coord = self._started(
            config_overrides={'esteps_commanded_length_mm': '50',
                               'esteps_extrude_speed_mm_s': '2'})
        coord.cmd_esteps_calibrate(fake.FakeGCmd({'CONTINUE': '1'}))
        self.assertEqual(gcode.scripts_run,
                          ['M82', 'G92 E0', 'G1 E50.0000 F120'])

    def test_cannot_continue_twice(self):
        printer, gcode, coord = self._started()
        coord.cmd_esteps_calibrate(fake.FakeGCmd({'CONTINUE': '1'}))
        with self.assertRaises(fake.CommandError):
            coord.cmd_esteps_calibrate(fake.FakeGCmd({'CONTINUE': '1'}))


class EStepsApplyTest(unittest.TestCase):
    def _awaiting_measurement(self, extruder_obj=None, config_overrides=None):
        printer, gcode, coord = _build_esteps(extruder_obj, config_overrides)
        coord.cmd_esteps_calibrate(fake.FakeGCmd({}))
        coord.cmd_esteps_calibrate(fake.FakeGCmd({'CONTINUE': '1'}))
        gcode.scripts_run = []  # isolate MEASURED='s own scripts
        return printer, gcode, coord

    def test_measured_without_continue_is_rejected(self):
        printer, gcode, coord = _build_esteps()
        with self.assertRaises(fake.CommandError):
            coord.cmd_esteps_calibrate(fake.FakeGCmd({'MEASURED': '100'}))

    def test_exact_measurement_leaves_rotation_distance_unchanged(self):
        printer, gcode, coord = self._awaiting_measurement(
            FakeExtruderObj(rotation_distance=7.5))
        coord.cmd_esteps_calibrate(fake.FakeGCmd({'MEASURED': '100'}))
        self.assertEqual(coord.esteps_state, 'awaiting_confirmation')
        self.assertAlmostEqual(coord.esteps_new_rotation_distance, 7.5, places=9)
        coord.cmd_esteps_confirm(fake.FakeGCmd({}))
        self.assertEqual(coord.esteps_state, 'complete')
        self.assertIn('SET_EXTRUDER_ROTATION_DISTANCE EXTRUDER=extruder '
                       'DISTANCE=7.500000', gcode.scripts_run)

    def test_under_extrusion_formula_matches_upstream_measure_and_trim(self):
        # Upstream's own documented relationship:
        # new = old * actual / commanded. 95mm actually came out of a
        # commanded 100mm -> rotation_distance must SHRINK proportionally.
        printer, gcode, coord = self._awaiting_measurement(
            FakeExtruderObj(rotation_distance=7.5))
        coord.cmd_esteps_calibrate(fake.FakeGCmd({'MEASURED': '95'}))
        self.assertAlmostEqual(
            coord.esteps_new_rotation_distance, 7.5 * 95. / 100., places=9)

    def test_over_extrusion_grows_rotation_distance(self):
        printer, gcode, coord = self._awaiting_measurement(
            FakeExtruderObj(rotation_distance=7.5))
        coord.cmd_esteps_calibrate(fake.FakeGCmd({'MEASURED': '108'}))
        self.assertAlmostEqual(
            coord.esteps_new_rotation_distance, 7.5 * 108. / 100., places=9)

    def test_result_is_staged_for_save_config(self):
        printer, gcode, coord = self._awaiting_measurement(
            FakeExtruderObj(rotation_distance=7.5))
        coord.cmd_esteps_calibrate(fake.FakeGCmd({'MEASURED': '95'}))
        self.assertEqual(coord.esteps_state, 'awaiting_confirmation')
        configfile = printer.lookup_object('configfile')
        self.assertEqual(configfile.set_calls, [])
        coord.cmd_esteps_confirm(fake.FakeGCmd({}))
        self.assertEqual(coord.esteps_state, 'complete')
        self.assertEqual(len(configfile.set_calls), 1)
        section, option, value = configfile.set_calls[0]
        self.assertEqual(section, 'extruder')
        self.assertEqual(option, 'rotation_distance')
        self.assertAlmostEqual(float(value), 7.5 * 95. / 100., places=5)

    def test_implausible_correction_is_rejected_and_not_applied(self):
        # 50mm measured vs 100mm commanded is a 50% implied correction -
        # exceeds the default 30% sanity ceiling.
        printer, gcode, coord = self._awaiting_measurement(
            FakeExtruderObj(rotation_distance=7.5))
        with self.assertRaises(fake.CommandError) as ctx:
            coord.cmd_esteps_calibrate(fake.FakeGCmd({'MEASURED': '50'}))
        self.assertIn('esteps_max_correction_ratio', str(ctx.exception))
        self.assertEqual(coord.esteps_state, 'error')
        self.assertEqual(gcode.scripts_run, [])
        configfile = printer.lookup_object('configfile')
        self.assertEqual(configfile.set_calls, [])

    def test_custom_max_correction_ratio_is_honored(self):
        printer, gcode, coord = self._awaiting_measurement(
            FakeExtruderObj(rotation_distance=7.5),
            config_overrides={'esteps_max_correction_ratio': '0.6'})
        # 50% correction now passes under a 60% ceiling.
        coord.cmd_esteps_calibrate(fake.FakeGCmd({'MEASURED': '50'}))
        self.assertEqual(coord.esteps_state, 'awaiting_confirmation')
        coord.cmd_esteps_confirm(fake.FakeGCmd({}))
        self.assertEqual(coord.esteps_state, 'complete')

    def test_zero_measured_is_rejected(self):
        printer, gcode, coord = self._awaiting_measurement()
        with self.assertRaises(fake.CommandError):
            coord.cmd_esteps_calibrate(fake.FakeGCmd({'MEASURED': '0'}))

    def test_negative_measured_is_rejected(self):
        printer, gcode, coord = self._awaiting_measurement()
        with self.assertRaises(fake.CommandError):
            coord.cmd_esteps_calibrate(fake.FakeGCmd({'MEASURED': '-5'}))

    def test_cannot_apply_twice(self):
        printer, gcode, coord = self._awaiting_measurement()
        coord.cmd_esteps_calibrate(fake.FakeGCmd({'MEASURED': '100'}))
        with self.assertRaises(fake.CommandError):
            coord.cmd_esteps_calibrate(fake.FakeGCmd({'MEASURED': '100'}))

    def test_can_start_a_new_run_after_completion(self):
        printer, gcode, coord = self._awaiting_measurement()
        coord.cmd_esteps_calibrate(fake.FakeGCmd({'MEASURED': '100'}))
        coord.cmd_esteps_confirm(fake.FakeGCmd({}))
        coord.cmd_esteps_calibrate(fake.FakeGCmd({}))  # must not raise
        self.assertEqual(coord.esteps_state, 'awaiting_continue')
        self.assertEqual(coord.esteps_id, 2)


class EStepsStatusTest(unittest.TestCase):
    def test_status_fields_present_and_idle_by_default(self):
        printer, gcode, coord = _build_esteps()
        status = coord.get_status(0.)
        self.assertEqual(status['esteps_state'], 'idle')
        self.assertEqual(status['esteps_id'], 0)
        self.assertIsNone(status['esteps_error'])
        self.assertIsNone(status['esteps_commanded_length'])
        self.assertIsNone(status['esteps_measured_length'])
        self.assertIsNone(status['esteps_old_rotation_distance'])
        self.assertIsNone(status['esteps_new_rotation_distance'])

    def test_status_reflects_completed_run(self):
        printer, gcode, coord = _build_esteps(FakeExtruderObj(rotation_distance=7.5))
        coord.cmd_esteps_calibrate(fake.FakeGCmd({}))
        coord.cmd_esteps_calibrate(fake.FakeGCmd({'CONTINUE': '1'}))
        coord.cmd_esteps_calibrate(fake.FakeGCmd({'MEASURED': '95'}))
        status = coord.get_status(0.)
        self.assertEqual(status['esteps_state'], 'awaiting_confirmation')
        coord.cmd_esteps_confirm(fake.FakeGCmd({}))
        status = coord.get_status(0.)
        self.assertEqual(status['esteps_state'], 'complete')
        self.assertEqual(status['esteps_commanded_length'], 100.)
        self.assertEqual(status['esteps_measured_length'], 95.)
        self.assertAlmostEqual(status['esteps_old_rotation_distance'], 7.5, places=9)
        self.assertAlmostEqual(
            status['esteps_new_rotation_distance'], 7.5 * 95. / 100., places=9)


class EStepsCancelTest(unittest.TestCase):
    def test_cancel_from_awaiting_continue_resets_to_idle(self):
        printer, gcode, coord = _build_esteps()
        coord.cmd_esteps_calibrate(fake.FakeGCmd({}))
        self.assertEqual(coord.esteps_state, 'awaiting_continue')
        coord.cmd_esteps_cancel(fake.FakeGCmd({}))
        self.assertEqual(coord.esteps_state, 'cancelled')

    def test_cancel_from_awaiting_measurement_preserves_original_rd(self):
        printer, gcode, coord = _build_esteps(
            FakeExtruderObj(rotation_distance=7.5))
        coord.cmd_esteps_calibrate(fake.FakeGCmd({}))
        coord.cmd_esteps_calibrate(fake.FakeGCmd({'CONTINUE': '1'}))
        self.assertEqual(coord.esteps_state, 'awaiting_measurement')
        coord.cmd_esteps_cancel(fake.FakeGCmd({}))
        self.assertEqual(coord.esteps_state, 'cancelled')
        self.assertNotIn('SET_EXTRUDER_ROTATION_DISTANCE', ''.join(gcode.scripts_run))

    def test_cancel_from_awaiting_confirmation_preserves_original_rd(self):
        printer, gcode, coord = _build_esteps(
            FakeExtruderObj(rotation_distance=7.5))
        coord.cmd_esteps_calibrate(fake.FakeGCmd({}))
        coord.cmd_esteps_calibrate(fake.FakeGCmd({'CONTINUE': '1'}))
        coord.cmd_esteps_calibrate(fake.FakeGCmd({'MEASURED': '95'}))
        self.assertEqual(coord.esteps_state, 'awaiting_confirmation')
        gcode.scripts_run = []
        coord.cmd_esteps_cancel(fake.FakeGCmd({}))
        self.assertEqual(coord.esteps_state, 'cancelled')
        configfile = printer.lookup_object('configfile')
        self.assertEqual(configfile.set_calls, [])

    def test_cancel_when_idle_is_rejected(self):
        printer, gcode, coord = _build_esteps()
        with self.assertRaises(fake.CommandError):
            coord.cmd_esteps_cancel(fake.FakeGCmd({}))

    def test_can_start_new_run_after_cancel(self):
        printer, gcode, coord = _build_esteps()
        coord.cmd_esteps_calibrate(fake.FakeGCmd({}))
        coord.cmd_esteps_cancel(fake.FakeGCmd({}))
        coord.cmd_esteps_calibrate(fake.FakeGCmd({}))
        self.assertEqual(coord.esteps_state, 'awaiting_continue')
        self.assertEqual(coord.esteps_id, 2)


class EStepsConfirmTest(unittest.TestCase):
    def _awaiting_confirmation(self, rd=7.5):
        printer, gcode, coord = _build_esteps(
            FakeExtruderObj(rotation_distance=rd))
        coord.cmd_esteps_calibrate(fake.FakeGCmd({}))
        coord.cmd_esteps_calibrate(fake.FakeGCmd({'CONTINUE': '1'}))
        coord.cmd_esteps_calibrate(fake.FakeGCmd({'MEASURED': '95'}))
        gcode.scripts_run = []
        return printer, gcode, coord

    def test_confirm_applies_and_persists(self):
        printer, gcode, coord = self._awaiting_confirmation()
        coord.cmd_esteps_confirm(fake.FakeGCmd({}))
        self.assertEqual(coord.esteps_state, 'complete')
        set_rd = [s for s in gcode.scripts_run
                  if 'SET_EXTRUDER_ROTATION_DISTANCE' in s]
        self.assertEqual(len(set_rd), 1)
        configfile = printer.lookup_object('configfile')
        self.assertEqual(len(configfile.set_calls), 1)
        save_cmds = [s for s in gcode.scripts_run if s == 'SAVE_CONFIG']
        self.assertEqual(len(save_cmds), 1)

    def test_confirm_when_not_awaiting_confirmation_is_rejected(self):
        printer, gcode, coord = _build_esteps()
        with self.assertRaises(fake.CommandError):
            coord.cmd_esteps_confirm(fake.FakeGCmd({}))

    def test_confirm_when_awaiting_measurement_is_rejected(self):
        printer, gcode, coord = _build_esteps()
        coord.cmd_esteps_calibrate(fake.FakeGCmd({}))
        coord.cmd_esteps_calibrate(fake.FakeGCmd({'CONTINUE': '1'}))
        with self.assertRaises(fake.CommandError):
            coord.cmd_esteps_confirm(fake.FakeGCmd({}))


# ----------------------------------------------------------------------
# G83a (Run 13): load-cell guard popup. The guided calibration itself is
# tested in test_nebulaos_load_cell_calibration.py (G86).
# ----------------------------------------------------------------------
class FakeLoadCell:
    """Upstream LoadCell's public calibration API, with scripted readings."""
    def __init__(self, readings, calibrated=False, counts_per_gram=None):
        self.readings = list(readings)
        self.calibrated = calibrated
        self.counts_per_gram = counts_per_gram
        self.set_calls = []
        self.tare_calls = []

    def avg_counts(self):
        return self.readings.pop(0)

    def saturation_range(self):
        return -0x800000, 0x7FFFFF

    def counts_to_percent(self, counts):
        return counts / 0x7FFFFF * 100.

    def is_calibrated(self):
        return self.calibrated

    def get_counts_per_gram(self):
        return self.counts_per_gram

    def set_calibration(self, cpg, tare):
        self.set_calls.append((cpg, tare))

    def tare(self, counts):
        self.tare_calls.append(counts)


class FakeLoadCellProbe(FakeZOffsetProbe):
    def __init__(self, load_cell):
        super().__init__(is_calibrated=load_cell.calibrated)
        self.load_cell = load_cell

    def get_load_cell(self):
        return self.load_cell


class FakePrintStats:
    def __init__(self, state='standby'):
        self.state = state

    def get_status(self, eventtime):
        return {'state': self.state}


def _build_load_cell(readings, print_state='standby', **kw):
    lc = FakeLoadCell(readings, **kw)
    printer, gcode, coord = _build(z_offset_probe=FakeLoadCellProbe(lc),
                                   z_compensate=FakeZCompensate(20., 25.))
    printer.add_object('print_stats', FakePrintStats(print_state))
    prompts = []
    coord._send_action_prompt = lambda title, lines, buttons=None: \
        prompts.append((title, list(lines), list(buttons or [])))
    return printer, gcode, coord, lc, prompts


class LoadCellGuardTest(unittest.TestCase):
    def test_z_offset_refuses_and_offers_calibration(self):
        printer, gcode, coord, lc, prompts = _build_load_cell([])
        with self.assertRaises(fake.CommandError) as ctx:
            coord.cmd_z_offset_calibrate(fake.FakeGCmd({}))
        self.assertIn('NEBULAOS_LOAD_CELL_CALIBRATE', str(ctx.exception))
        self.assertEqual(prompts[-1][0], 'Load cell not calibrated')
        self.assertIn(('Calibrate load cell', 'NEBULAOS_LOAD_CELL_CALIBRATE'),
                      prompts[-1][2])

    def test_calibrated_cell_gets_no_popup(self):
        printer, gcode, coord, lc, prompts = _build_load_cell(
            [], calibrated=True, counts_per_gram=85.7)
        coord._offer_load_cell_calibration()   # direct call still works
        prompts.clear()
        self.assertTrue(coord.get_status(0.)['load_cell_calibrated'])


if __name__ == '__main__':
    unittest.main()
