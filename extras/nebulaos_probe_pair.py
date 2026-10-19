# NebulaOS automatic probe/nozzle pairing primitive (Phase 2 calibration-
# framework mission; bounded-descent contact-safety rewrite, corrected).
#
# The shared math behind NEBULAOS_Z_OFFSET_CALIBRATE: at a physical bed
# point P, take a FRESH automatic BLTouch probe reading, then move the
# actual nozzle over the exact same P and take one or more validated,
# envelope-bounded HX711 contact readings, and derive the probe's true
# Z-offset from the two raw readings.
#
# Derived directly from pinned upstream Klipper (58bd67db...)'s own
# arithmetic, not invented: klippy/extras/probe.py's
# ProbeCommandHelper.probe_calibrate_finalize() computes
#     z_offset = offsets[2] - mpresult.bed_z + ppos.bed_z
# where ppos.bed_z = raw_trigger_z - current_z_offset (manual_probe.py's
# create_probe_result) and mpresult.bed_z is the RAW toolhead Z a human (or,
# here, the load cell) accepts as nozzle-bed contact. Substituting and
# simplifying, the current z_offset cancels algebraically:
#     z_offset_new = raw_probe_trigger_z - raw_nozzle_contact_z
# both measured in the SAME toolhead-Z coordinate frame. Because both
# readings are taken back-to-back in whatever gcode-offset frame happens to
# be active at the time, any currently-active SET_GCODE_OFFSET Z also
# cancels the same way - this result does not depend on it.
#
# ---------------------------------------------------------------------
# Bounded-descent contact-safety envelope (Phase 2 mission - CORRECTED)
# ---------------------------------------------------------------------
# A prior version of this file treated the raw BLTouch trigger position
# (ProbeResult.test_z) itself as the "predicted surface" the nozzle would
# hit, and bounded the descent as test_z - max_contact_descent_mm. That is
# wrong, and dangerously so: pinned upstream Klipper's own
# manual_probe.py:create_probe_result() defines
#     bed_z = test_z - z_offset
# test_z is the toolhead Z at PROBE trigger; bed_z is the ESTIMATED
# NOZZLE-CONTACT Z - upstream's own name for exactly the quantity this
# envelope needs to bound against. The two differ by the probe's own
# z_offset, which on this printer's real captured hardware state is
# ~1.795mm - far larger than any sane descent margin. Anchoring the
# envelope to test_z directly was therefore off by the ENTIRE probe
# offset, not a rounding error.
#
# The complication upstream's formula does not have to deal with: this
# workflow's whole job is to MEASURE a new z_offset, so the CURRENT one is
# only ever a prior estimate, of unknown and workflow-dependent quality.
# Two physically distinct cases follow, and the envelope is derived
# differently in each - see _resolve_envelope() below for the exact
# formulas, and PairedMeasurement.contact_mode/predicted_nozzle_contact_z
# for how a caller can tell which one ran.
#
#   ESTABLISHED (a credible existing probe_z_offset is already active):
#     predicted_nozzle_contact_z = raw_probe_trigger_z - probe_z_offset
#     commanded_floor_z = predicted_nozzle_contact_z
#                          - established_contact_margin_mm
#     established_contact_margin_mm is a SMALL, separately-configured
#     bound on how much further than the (imperfect but credible)
#     predicted plane the nozzle may still be commanded to travel - it
#     must not, and does not, also carry the probe's own Z offset the way
#     the old max_contact_descent_mm accidentally did.
#
#   BOOTSTRAP / VIRGIN (no credible existing probe_z_offset - e.g. the
#   real factory default of exactly 0.000, which is a "not yet
#   calibrated" marker, not a physical claim that probe and nozzle
#   trigger at the same Z):
#     commanded_floor_z = starting_nozzle_z - bootstrap_contact_envelope_mm
#     starting_nozzle_z is the toolhead's own actual, known, currently-
#     measured Z right before the descent begins - not a prediction at
#     all, just "how far below where the nozzle already safely is are we
#     willing to blindly search." bootstrap_contact_envelope_mm is a
#     SEPARATE config value from down_min_z (never silently reused as one)
#     and requires its own explicit hardware qualification before a
#     virgin printer's first automatic Z-offset calibration can run at
#     all - there is no invented default for it either.
#
# Both established_contact_margin_mm and bootstrap_contact_envelope_mm are
# read from the CALLER (nebulaos_calibration.py's own config - a
# calibration-acceptance concern, not a raw motion-primitive one), not
# from the ZOffsetProbe instance. Neither has a production default; this
# function fails closed with CONTACT_SAFETY_LIMIT_UNQUALIFIED, before any
# nozzle contact motion, when the one the current case needs is unset.
#
# Every individual contact is recorded as a ContactSample - accepted or
# rejected, always appended, never silently dropped - and the aggregate
# RepeatabilityResult/PairedMeasurement.accepted flag is the ONLY thing a
# caller may use to decide whether to stage a Z-offset, transition to
# COMPLETE, or call SAVE_CONFIG (see nebulaos_calibration.py).
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import collections
import math

from . import probe as probe_module

# A live probe z_offset within this many mm of exactly zero is treated as
# "not yet calibrated" (the real, tracked factory default is the literal
# string "0.000" - see NebulaOS-firmware's migrate_config_ownership.py
# FACTORY_DEFAULTS), not as a physically-zero offset that happens to be
# correct. Small enough that no genuine calibration result could ever
# land inside it by chance.
_CREDIBLE_Z_OFFSET_EPSILON_MM = 0.0005

ContactSample = collections.namedtuple('ContactSample', [
    'sample_index', 'starting_z', 'commanded_floor_z',
    'raw_trigger_z', 'fitted_contact_z', 'fit_delta',
    'fit_valid', 'motion_safe', 'accepted', 'rejection_reason',
    # Contact-force telemetry (Phase 2 mission, §1) - see
    # nebulaos_z_offset_probe.py's own _fit_contact_z() comment for exact
    # derivation and honest limitations of each. peak_force_g/
    # peak_force_time/force_at_trigger_g/tare_counts come straight from
    # ZOffsetProbe.get_status() for THIS sample; predicted_to_raw_
    # trigger_depth and remaining_margin_to_floor are derived here from
    # values already computed in this same function.
    'peak_force_g', 'peak_force_time', 'force_at_trigger_g', 'tare_counts',
    'predicted_to_raw_trigger_depth', 'remaining_margin_to_floor'])

RepeatabilityResult = collections.namedtuple('RepeatabilityResult', [
    'sample_count', 'accepted_count', 'samples',
    'mean', 'minimum', 'maximum', 'range', 'stddev',
    'accepted', 'rejection_reason'])

PairedMeasurement = collections.namedtuple('PairedMeasurement', [
    'x', 'y', 'contact_id', 'contact_mode',
    'predicted_nozzle_contact_z', 'commanded_floor_z',
    'raw_probe_trigger_z', 'raw_nozzle_contact_z',
    'repeatability', 'probe_z_offset',
    'accepted', 'rejection_reason',
    # Configured contact-safety values in effect for this run (constant
    # across every sample) - echoed here purely for reporting/
    # characterization convenience, per §1's own list.
    'trigger_force', 'force_safety_limit', 'contact_speed'])


def _is_credible_probe_z_offset(probe_z_offset):
    """False for the real factory default (exactly 0.000) and anything
    close enough to it to be indistinguishable from "never calibrated" -
    see this module's own header for why 0.000 must never be treated as a
    valid physical prior for the ESTABLISHED envelope case."""
    return (math.isfinite(probe_z_offset)
            and abs(probe_z_offset) > _CREDIBLE_Z_OFFSET_EPSILON_MM)


def _require_safety_limits(printer, x, y, probe_z_offset, is_established,
                            established_contact_margin_mm,
                            bootstrap_contact_envelope_mm,
                            pro_cnt, max_abs_fit_delta, min_accepted_samples,
                            max_repeatability_range,
                            max_repeatability_stddev):
    """Fail-closed preflight: refuses ANY nozzle contact motion (called
    before the nozzle is ever moved toward the bed, and before the fresh
    CR-Touch reading too - the ESTABLISHED/BOOTSTRAP choice and the
    corresponding required config are both already knowable from
    probe_z_offset alone) unless every safety-critical constant this run
    actually needs is configured. No invented production defaults."""
    if is_established:
        if established_contact_margin_mm is None:
            raise printer.command_error(
                "CONTACT_SAFETY_LIMIT_UNQUALIFIED: established_contact_"
                "margin_mm is not configured - the nozzle-contact safety "
                "envelope cannot be derived for this ESTABLISHED "
                "calibration (existing probe z_offset=%.5f is credible), "
                "refusing nozzle contact motion at X=%.3f Y=%.3f"
                % (probe_z_offset, x, y))
    else:
        if bootstrap_contact_envelope_mm is None:
            raise printer.command_error(
                "CONTACT_SAFETY_LIMIT_UNQUALIFIED: bootstrap_contact_"
                "envelope_mm is not configured - the existing probe "
                "z_offset=%.5f is not a credible physical prior (BOOTSTRAP/"
                "VIRGIN calibration), and no separately-qualified "
                "bootstrap envelope is configured. Refusing nozzle contact "
                "motion at X=%.3f Y=%.3f - this does NOT fall back to "
                "down_min_z." % (probe_z_offset, x, y))
    if max_abs_fit_delta is None:
        raise printer.command_error(
            "CONTACT_SAFETY_LIMIT_UNQUALIFIED: max_abs_fit_delta is not "
            "configured - the measurement-quality gate cannot be applied, "
            "refusing nozzle contact motion at X=%.3f Y=%.3f" % (x, y))
    if pro_cnt > 1 and (min_accepted_samples is None
                         or max_repeatability_range is None
                         or max_repeatability_stddev is None):
        raise printer.command_error(
            "CONTACT_SAFETY_LIMIT_UNQUALIFIED: repeatability validation "
            "bounds (min_accepted_samples/max_repeatability_range/"
            "max_repeatability_stddev) are not fully configured for "
            "pro_cnt=%d - refusing nozzle contact motion at X=%.3f Y=%.3f"
            % (pro_cnt, x, y))


def measure_probe_nozzle_pair(printer, x, y, probe_x_offset, probe_y_offset,
                               probe_z_offset,
                               horizontal_move_z, z_offset_probe,
                               down_min_z, pro_cnt=1,
                               travel_speed=None, probe_lift_speed=None,
                               established_contact_margin_mm=None,
                               bootstrap_contact_envelope_mm=None,
                               max_abs_fit_delta=None,
                               min_accepted_samples=None,
                               max_repeatability_range=None,
                               max_repeatability_stddev=None,
                               arming=None):
    """Take one bounded, quality-gated, optionally-repeated (probe, nozzle)
    measurement at bed point (x, y). Returns a PairedMeasurement whose
    `accepted` flag is the single authoritative answer to "may a caller
    stage this result" - see this module's own header comment.

    probe_x_offset/probe_y_offset/probe_z_offset: the registered probe's
    own CURRENT, live configured offsets (probe_obj.get_offsets()).
    probe_z_offset is the prior this calibration run is trying to refine -
    see this module's header for the ESTABLISHED/BOOTSTRAP split it
    determines.

    z_offset_probe: a nebulaos_z_offset_probe.ZOffsetProbe instance (or
    anything exposing the same touch_probe(down_min_z, pro_cnt=1,
    minimum_allowed_z=...)/get_status(eventtime) contract). Owns only the
    raw motion primitive - the envelope derivation lives entirely in this
    function now, not on that object.

    down_min_z: the same absolute stepper-limit-relative depth floor
    touch_probe() has always accepted - kept as an independent, coarser
    safety layer underneath the derived envelope (the tighter of the two
    always wins - see touch_probe()'s own comment).

    established_contact_margin_mm/bootstrap_contact_envelope_mm: the two
    envelope constants (Phase 2 mission, corrected) - exactly one is
    required depending on whether probe_z_offset is credible, and
    whichever is required fails closed with CONTACT_SAFETY_LIMIT_
    UNQUALIFIED when unset. No production default is invented for either.

    max_abs_fit_delta/min_accepted_samples/max_repeatability_range/
    max_repeatability_stddev: measurement-quality and repeatability
    acceptance bounds (Phase 2 mission, §7/§9/§10) - unchanged from
    before, still fail-closed, still optional, still no invented default.

    travel_speed/probe_lift_speed: motion speeds for the repositioning
    moves; None lets toolhead.manual_move fall back to whatever speed was
    last used. Callers should normally pass explicit values.

    Raises the same exceptions run_single_probe()/touch_probe() would
    raise on a HARD failure path (unhomed axis, sensor error, insufficient
    fit data, no valid trigger before the commanded floor, etc.) -
    deliberately does not catch or reinterpret those; only a QUALITY
    rejection (excessive fit delta, out-of-envelope result, failed
    repeatability) is represented as `accepted=False` in the returned
    PairedMeasurement rather than an exception, since the contact itself
    completed safely and the toolhead is already lifted clear.
    """
    toolhead = printer.lookup_object('toolhead')
    gcode = printer.lookup_object('gcode')
    probe_obj = printer.lookup_object('probe')
    reactor = printer.get_reactor()

    is_established = _is_credible_probe_z_offset(probe_z_offset)
    _require_safety_limits(
        printer, x, y, probe_z_offset, is_established,
        established_contact_margin_mm, bootstrap_contact_envelope_mm,
        pro_cnt, max_abs_fit_delta, min_accepted_samples,
        max_repeatability_range, max_repeatability_stddev)

    def hover():
        toolhead.manual_move([None, None, horizontal_move_z], probe_lift_speed)

    # Fresh CR-Touch measurement at physical P - needed for BOTH cases'
    # diagnostics (and for the ESTABLISHED envelope's own prediction), and
    # for the final probe_z_offset arithmetic either way.
    hover()
    toolhead.manual_move([x - probe_x_offset, y - probe_y_offset, None],
                         travel_speed)
    probe_gcmd = gcode.create_gcode_command("", "", {})
    ppos = probe_module.run_single_probe(probe_obj, probe_gcmd)
    raw_probe_trigger_z = ppos.test_z

    if not math.isfinite(raw_probe_trigger_z):
        raise printer.command_error(
            "measure_probe_nozzle_pair: CR-Touch reading at X=%.3f Y=%.3f "
            "is not finite (%r) - cannot derive a bounded-descent "
            "envelope, refusing nozzle contact" % (x, y, raw_probe_trigger_z))

    # Move the NOZZLE (toolhead origin, no offset) to the exact same
    # physical (x, y) before computing/using either envelope formula -
    # the BOOTSTRAP case's own floor is relative to the nozzle's real
    # position at this point, not the probe's.
    hover()
    toolhead.manual_move([x, y, None], travel_speed)
    starting_nozzle_z = toolhead.get_position()[2]

    if is_established:
        contact_mode = 'established'
        predicted_nozzle_contact_z = raw_probe_trigger_z - probe_z_offset
        commanded_floor_z = (predicted_nozzle_contact_z
                              - established_contact_margin_mm)
    else:
        contact_mode = 'bootstrap'
        predicted_nozzle_contact_z = None
        commanded_floor_z = starting_nozzle_z - bootstrap_contact_envelope_mm

    contact_id = reactor.monotonic()
    samples = []
    accepted_zs = []
    for i in range(pro_cnt):
        starting_z = toolhead.get_position()[2]
        raw_z = fitted_z = fit_delta = None
        fit_valid = motion_safe = accepted = False
        rejection_reason = None
        try:
            # arming (G83b-auto, Run 14): the probe's plan for this
            # canonical paired path; None keeps the calibrated default.
            touch_kwargs = {} if arming is None else {'arming': arming}
            fitted_z = z_offset_probe.touch_probe(
                down_min_z, pro_cnt=1, minimum_allowed_z=commanded_floor_z,
                **touch_kwargs)
        except Exception as e:
            samples.append(ContactSample(
                sample_index=i, starting_z=starting_z,
                commanded_floor_z=commanded_floor_z,
                raw_trigger_z=None, fitted_contact_z=None, fit_delta=None,
                fit_valid=False, motion_safe=False, accepted=False,
                rejection_reason="contact_error: %s"
                                  % (str(e).strip() or e.__class__.__name__,),
                peak_force_g=None, peak_force_time=None,
                force_at_trigger_g=None, tare_counts=None,
                predicted_to_raw_trigger_depth=None,
                remaining_margin_to_floor=None))
            raise

        status = z_offset_probe.get_status(reactor.monotonic())
        raw_z = status['last_raw_trigger_z']
        fit_delta = status['last_fit_delta']
        peak_force_g = status['last_peak_force_g']
        peak_force_time = status['last_peak_force_time']
        force_at_trigger_g = status['last_force_at_trigger_g']
        tare_counts = status['last_tare_counts']
        fit_valid = (raw_z is not None and fitted_z is not None
                     and fit_delta is not None
                     and math.isfinite(raw_z) and math.isfinite(fitted_z)
                     and math.isfinite(fit_delta))
        motion_safe = fit_valid and raw_z >= commanded_floor_z - 1e-6
        if not fit_valid:
            rejection_reason = "non_finite_or_missing_fit_result"
        elif not motion_safe:
            rejection_reason = "result_outside_commanded_envelope"
        elif abs(fit_delta) > max_abs_fit_delta:
            rejection_reason = ("excessive_fit_delta(%.4f>%.4f)"
                                 % (abs(fit_delta), max_abs_fit_delta))
        else:
            accepted = True

        predicted_to_raw_trigger_depth = None
        remaining_margin_to_floor = None
        if raw_z is not None and math.isfinite(raw_z):
            if predicted_nozzle_contact_z is not None:
                predicted_to_raw_trigger_depth = (
                    predicted_nozzle_contact_z - raw_z)
            remaining_margin_to_floor = raw_z - commanded_floor_z

        samples.append(ContactSample(
            sample_index=i, starting_z=starting_z,
            commanded_floor_z=commanded_floor_z,
            raw_trigger_z=raw_z, fitted_contact_z=fitted_z,
            fit_delta=fit_delta, fit_valid=fit_valid,
            motion_safe=motion_safe, accepted=accepted,
            rejection_reason=rejection_reason,
            peak_force_g=peak_force_g, peak_force_time=peak_force_time,
            force_at_trigger_g=force_at_trigger_g, tare_counts=tare_counts,
            predicted_to_raw_trigger_depth=predicted_to_raw_trigger_depth,
            remaining_margin_to_floor=remaining_margin_to_floor))
        if accepted:
            accepted_zs.append(fitted_z)

    # Leave the toolhead lifted clear of the bed before handing control
    # back - the caller may be about to traverse to a different XY point.
    hover()

    valid_count = len(accepted_zs)
    mean = minimum = maximum = rng = stddev = None
    if valid_count:
        mean = sum(accepted_zs) / valid_count
        minimum = min(accepted_zs)
        maximum = max(accepted_zs)
        rng = maximum - minimum
        if valid_count > 1:
            variance = sum((v - mean) ** 2 for v in accepted_zs) / (valid_count - 1)
            stddev = math.sqrt(variance)
        else:
            stddev = 0.0

    repeat_accepted = True
    repeat_rejection = None
    if valid_count == 0:
        repeat_accepted = False
        repeat_rejection = "no_accepted_samples"
    elif min_accepted_samples is not None and valid_count < min_accepted_samples:
        repeat_accepted = False
        repeat_rejection = ("insufficient_accepted_samples(%d<%d)"
                             % (valid_count, min_accepted_samples))
    elif max_repeatability_range is not None and rng is not None \
            and rng > max_repeatability_range:
        repeat_accepted = False
        repeat_rejection = ("repeatability_range_exceeded(%.4f>%.4f)"
                             % (rng, max_repeatability_range))
    elif max_repeatability_stddev is not None and stddev is not None \
            and stddev > max_repeatability_stddev:
        repeat_accepted = False
        repeat_rejection = ("repeatability_stddev_exceeded(%.4f>%.4f)"
                             % (stddev, max_repeatability_stddev))

    repeatability = RepeatabilityResult(
        sample_count=len(samples), accepted_count=valid_count,
        samples=samples, mean=mean, minimum=minimum, maximum=maximum,
        range=rng, stddev=stddev, accepted=repeat_accepted,
        rejection_reason=repeat_rejection)

    probe_z_offset_result = None
    if repeat_accepted:
        probe_z_offset_result = raw_probe_trigger_z - mean

    # Configured values echoed for reporting convenience (§1) - read once
    # here rather than per-sample, since they cannot change mid-run.
    final_status = z_offset_probe.get_status(reactor.monotonic())

    return PairedMeasurement(
        x=x, y=y, contact_id=contact_id, contact_mode=contact_mode,
        predicted_nozzle_contact_z=predicted_nozzle_contact_z,
        commanded_floor_z=commanded_floor_z,
        raw_probe_trigger_z=raw_probe_trigger_z,
        raw_nozzle_contact_z=mean,
        repeatability=repeatability,
        probe_z_offset=probe_z_offset_result,
        accepted=repeat_accepted, rejection_reason=repeat_rejection,
        trigger_force=final_status['trigger_force'],
        force_safety_limit=final_status['force_safety_limit'],
        contact_speed=final_status['contact_speed'])
