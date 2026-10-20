# NebulaOS Z-offset contact probe — nozzle load cell adapter for Z-offset calibration
#
# Uses the nozzle load cell (HX711 strain gauge) for Z-offset calibration ONLY.
# BLTouch remains the global Klipper probe for Z homing, bed mesh, and
# probe:z_virtual_endstop. This module does NOT register as a global probe —
# no printer.add_object('probe', self), no ProbeCommandHelper, no HomingViaProbeHelper.
#
# Consumed exclusively by z_compensate.py's Z_OFFSET_CALIBRATION command, which calls
# touch_probe(down_min_z, pro_cnt) to get a single averaged Z contact position.
#
# Architecture: creates its own HX711 sensor and upstream LoadCell, wrapping upstream
# Klipper's load_cell + trigger_analog primitives. Zero host core patches
# (HOST_KLIPPER_CORE_PATCHES=0). The LoadCell wrapper provides LOAD_CELL_READ,
# LOAD_CELL_TARE, LOAD_CELL_CALIBRATE, and LOAD_CELL_DIAGNOSTIC G-code commands
# under this section's name for zero-motion sensor qualification.
#
# Safety model derived from upstream 58bd67db load_cell_probe.py. Key differences
# from upstream documented inline. ContinuousTareFilter is intentionally omitted:
# our use case is single-shot Z-offset calibration with a fresh tare before each
# contact, not continuous probing where drift compensation matters. Each touch_probe
# call performs its own tare via the sample collector, making drift filtering
# redundant for this application.
#
# Contact position accuracy: uses upstream LCBestFit piecewise least-squares
# analysis on ascent (retract) samples to interpolate the true contact Z,
# compensating for MCU trigger latency. Reuses LCBestFit, _lookup_z_pos,
# FIT_MIN_POINTS, and ASCENT_DATA_WINDOW_SECONDS directly from upstream
# load_cell_probe.py. TappingMove is not reused (coupled to global probe
# infrastructure); only the minimal ascent-collection and fit-analysis logic
# is adapted here.
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging
import math
from . import hx71x, load_cell, probe, trigger_analog
from . import nebulaos_raw_contact as raw_contact
from .load_cell_probe import (LCBestFit, _lookup_z_pos,
                               FIT_MIN_POINTS, ASCENT_DATA_WINDOW_SECONDS)

# MCU SOS filter scaled to "fractional grams" for consistent sensor precision.
# Upstream 58bd load_cell_probe.py defines this identically.
FRAC_GRAMS_CONV = 32768.0


class ZOffsetProbe:
    def __init__(self, config):
        self._printer = config.get_printer()
        self._name = config.get_name()

        sensors = {}
        sensors.update(hx71x.HX71X_SENSOR_TYPES)
        sensor_class = config.getchoice('sensor_type', sensors)
        self._sensor = sensor_class(config)

        self._load_cell = load_cell.LoadCell(config, self._sensor)

        self._mcu_trigger_analog = trigger_analog.MCU_trigger_analog(
            self._sensor)
        mcu = self._sensor.get_mcu()
        cmd_queue = self._mcu_trigger_analog.get_dispatch().get_command_queue()
        sos_filter = trigger_analog.MCU_SosFilter(mcu, cmd_queue, 4)
        self._mcu_trigger_analog.setup_sos_filter(sos_filter)

        probe.LookupZSteppers(
            config, self._mcu_trigger_analog.get_dispatch().add_stepper)

        self._z_min_position = probe.lookup_minimum_z(config)

        self._contact_speed = config.getfloat(
            'contact_speed', default=2.0, minval=0.1, maxval=10.)
        self._retract_dist = config.getfloat(
            'retract_dist', default=2.0, minval=0.5, maxval=10.)
        self._retract_speed = config.getfloat(
            'retract_speed', default=10.0, minval=1., maxval=50.)
        self._trigger_force = config.getfloat(
            'trigger_force', default=75., minval=10., maxval=250.)
        self._force_safety_limit = config.getfloat(
            'force_safety_limit', default=2000., minval=100., maxval=10000.)
        self._tare_time = config.getfloat(
            'tare_time', default=4. / 60., minval=0.01, maxval=1.0)

        self._best_fit = LCBestFit(self._printer)

        # G83b weight-free contact (docs/NEBULAOS_RAW_CONTACT_BRIEF.md in the
        # firmware repo). Run 14 (G83b-auto): the default is 'auto' -
        # resolved per caller by raw_contact.resolve_arming(): the unit's own
        # grams calibration first, a QUALIFIED raw profile only for the
        # canonical paired Z-offset path, otherwise a refusal. With
        # QUALIFIED_PROFILES empty this is today's calibrated behaviour.
        # Never write this option into the image-owned z_offset_probe.cfg:
        # configfile refuses an autosaved option that an include also sets.
        self._contact_mode = config.getchoice(
            'contact_detection_mode',
            {m: m for m in raw_contact.MODES}, raw_contact.MODE_AUTO)
        # Raw arming in use for the current touch (an explicit plan from the
        # caller; None = calibrated force) and the last resolved mode.
        self._active_profile = None
        self._effective_mode = None
        # Idle window before a raw touch: long enough for validate_idle()
        # (>= 20 samples) and a meaningful noise/drift estimate.
        self._raw_health_time = config.getfloat(
            'raw_health_time', default=0.75, minval=0.3, maxval=5.)
        self._raw_profile_id = config.get('raw_contact_profile_id', None)
        # Run 15: the persisted unloaded reference R for a unit WITHOUT a
        # grams calibration (brief 5.1.5), captured by
        # NEBULAOS_RAW_CONTACT_REFERENCE into the SAVE_CONFIG autosave
        # block. Never in an image-owned cfg (configfile include conflict).
        self._raw_reference = config.getint('raw_reference_counts', None)
        self._last_contact_rejection_reason = None
        self._last_raw_peak_delta_counts = None
        self._raw_diag = None
        gcode = self._printer.lookup_object('gcode')
        gcode.register_command(
            'NEBULAOS_RAW_CONTACT_DIAGNOSTIC', self.cmd_raw_contact_diagnostic,
            desc=self.cmd_raw_contact_diagnostic_help)
        gcode.register_command(
            'NEBULAOS_RAW_CONTACT_REFERENCE', self.cmd_raw_contact_reference,
            desc=self.cmd_raw_contact_reference_help)

        self._last_raw_trigger_z = None
        self._last_fitted_contact_z = None
        self._last_fit_delta = None
        # Contact-force telemetry (Phase 2 mission, §1) - see
        # _fit_contact_z()'s own comment for exactly how these are derived
        # from the SAME sample stream already collected for the fit, and
        # the honest limitations of each.
        self._last_peak_force_g = None
        self._last_peak_force_time = None
        self._last_force_at_trigger_g = None
        self._last_tare_counts = None

    def _get_safety_range(self):
        counts_per_gram = self._load_cell.get_counts_per_gram()
        zero = self._load_cell.get_reference_tare_counts()
        safety_counts = int(counts_per_gram * self._force_safety_limit)
        safety_min = int(zero - safety_counts)
        safety_max = int(zero + safety_counts)
        sensor_min, sensor_max = self._sensor.get_range()
        if safety_min <= sensor_min or safety_max >= sensor_max:
            raise self._printer.command_error(
                "%s: force_safety_limit exceeds sensor range" % self._name)
        return safety_min, safety_max

    def _get_grams_per_count(self):
        counts_per_gram = self._load_cell.get_counts_per_gram()
        if counts_per_gram >= (1 << 29):
            raise OverflowError(
                "%s: counts_per_gram value is too large to filter"
                % self._name)
        return 1. / counts_per_gram

    def get_load_cell(self):
        """The upstream LoadCell this probe owns (G83a: the guided
        NEBULAOS_LOAD_CELL_CALIBRATE flow uses its public calibration API)."""
        return self._load_cell

    def plan_arming(self, caller, profile=None):
        """The arming for a touch by `caller`: (ARM_CALIBRATED, None),
        (ARM_RAW, profile) or a refusal (None, (code, message)). profile=
        is an explicit stage-C candidate (supervised comparison only); it
        never comes from a registry lookup and needs the calibration."""
        if profile is not None:
            problems = raw_contact.validate_profile(profile)
            if problems:
                return None, (raw_contact.RAW_BAD_PROFILE, "; ".join(problems))
            if not self._load_cell.is_calibrated():
                return None, (raw_contact.NOT_CALIBRATED,
                              "the comparison needs the calibrated load cell")
            return raw_contact.ARM_RAW, profile
        # A persisted unloaded reference for uncalibrated units does not
        # exist yet (brief 5.1.5): auto/raw cannot resolve to raw without one.
        return raw_contact.resolve_arming(
            self._contact_mode, caller, self._load_cell.is_calibrated(),
            self._raw_profile_id,
            reference_counts=self._raw_reference_counts())

    def contact_ready(self, caller):
        """The guard's predicate (brief E.1): may `caller` touch the bed?
        True for the unit's grams calibration, or a qualified raw profile
        plus a reference on the paired path. Never fakes is_calibrated."""
        kind, _ = self.plan_arming(caller)
        return kind is not None

    def _raw_reference_counts(self):
        """The independent unloaded reference the raw safety window is
        anchored to (never the fresh tare): the calibration's zero, else the
        persisted reference of an uncalibrated unit, else None (no raw)."""
        if self._load_cell.is_calibrated():
            return self._load_cell.get_reference_tare_counts()
        return self._raw_reference

    def _refuse(self, code, message):
        self._last_contact_rejection_reason = code
        raise self._printer.command_error("%s: %s" % (self._name, message))

    def _arm_raw(self, profile):
        """Raw-count arming (G83b-auto, architecture review Run 14):
        1. dedicated idle window -> health check against the profile, the
           median is the tare; a failed check REFUSES (no fallback);
        2. the fresh tare must be within max_reference_drift_counts of the
           independent reference R;
        3. MCU raw window R +- safety_counts, strictly inside the sensor;
        4. offset -tare, scale 1 (integer: no fixed-point bits), trigger
           abs_ge trigger_counts. abs_ge rather than gt with
           scale=polarity: an opposite-sign event can only stop early."""
        reference = self._raw_reference_counts()
        if reference is None:
            self._refuse(raw_contact.RAW_NO_REFERENCE,
                         "raw contact has no unloaded reference - nozzle "
                         "contact refused")
        # Only samples after every queued move (travel, previous retract):
        # a bed still settling must not become the zero (review note 1).
        toolhead = self._printer.lookup_object('toolhead')
        start = toolhead.get_last_move_time()
        collector = self._load_cell.get_collector()
        collector.start_collecting(min_time=start)
        samples, errors = collector.collect_until(start + self._raw_health_time)
        stats, reasons = raw_contact.validate_idle(
            samples, errors, self._load_cell.saturation_range(),
            profile['expected_sps'], profile['sps_tolerance'])
        if not reasons and stats['sigma'] > profile['max_idle_sigma']:
            # the trigger must sit 10x above the idle noise: a lower
            # TRIGGER_COUNTS is refused here (review note 4)
            reasons.append('noise_above_profile (sigma %.0f counts, the '
                           'trigger needs 10x that)' % (stats['sigma'],))
        if not reasons and stats['drift_per_s'] is not None and \
                abs(stats['drift_per_s']) > profile['max_idle_drift_per_s']:
            reasons.append('drift_above_profile')
        self._raw_diag = dict(stats)
        self._raw_diag['reasons'] = list(reasons)
        if reasons:
            self._refuse(raw_contact.RAW_UNHEALTHY,
                         "load cell not healthy before contact (%s) - nozzle "
                         "contact refused" % (", ".join(reasons),))
        tare = int(stats['median'])
        if abs(tare - reference) > profile['max_reference_drift_counts']:
            self._refuse(raw_contact.RAW_REFERENCE_DRIFT,
                         "the empty reading moved %d counts from the reference "
                         "(limit %d) - something presses on the bed? nozzle "
                         "contact refused" % (tare - reference,
                                              profile['max_reference_drift_counts']))
        safety_min = int(reference - profile['safety_counts'])
        safety_max = int(reference + profile['safety_counts'])
        sensor_min, sensor_max = self._sensor.get_range()
        if safety_min <= sensor_min or safety_max >= sensor_max:
            self._refuse(raw_contact.RAW_BAD_PROFILE,
                         "raw safety window exceeds the sensor range")
        self._load_cell.tare(tare)
        self._mcu_trigger_analog.set_raw_range(safety_min, safety_max)
        sos_filter = self._mcu_trigger_analog.get_sos_filter()
        sos_filter.set_offset_scale(-tare, 1)
        self._mcu_trigger_analog.set_trigger("abs_ge",
                                             int(profile['trigger_counts']))
        self._active_profile = profile
        self._effective_mode = raw_contact.MODE_RAW

    def _tare_and_arm(self, arming=None):
        """arming: None (every existing caller: calibrated force, as before)
        or a plan from plan_arming()."""
        self._active_profile = None
        self._effective_mode = None
        if arming is not None:
            kind, detail = arming
            if kind is None:
                self._refuse(detail[0], detail[1])
            if kind == raw_contact.ARM_RAW:
                return self._arm_raw(detail)
        elif self._contact_mode == raw_contact.MODE_RAW:
            refusal = raw_contact.raw_motion_refusal(self._raw_profile_id)
            if refusal is None:
                # A qualified profile exists - but stage B ships without the
                # count-domain arming (stage C/D evidence decides its values).
                refusal = ('raw_contact_arming_not_implemented',
                           "raw_contact arming is not enabled in this "
                           "version - nozzle contact refused")
            self._last_contact_rejection_reason = refusal[0]
            raise self._printer.command_error(
                "%s: %s" % (self._name, refusal[1]))
        if not self._load_cell.is_calibrated():
            raise self._printer.command_error(
                "%s: load cell not calibrated — run LOAD_CELL_CALIBRATE"
                " LOAD_CELL=%s to set counts_per_gram and"
                " reference_tare_counts before contact motion"
                % (self._name, self._name.split()[-1]))

        toolhead = self._printer.lookup_object('toolhead')
        print_time = toolhead.get_last_move_time()

        collector = self._load_cell.get_collector()
        collector.start_collecting(min_time=print_time)
        sps = self._sensor.get_samples_per_second()
        num_samples = max(2, math.ceil(sps * self._tare_time))
        samples, err = collector.collect_min(num_samples)
        if err:
            errors, overflows = err
            raise self._printer.command_error(
                "%s: sensor errors during tare: %i errors, %i overflows"
                % (self._name, errors, overflows))
        if not samples:
            raise self._printer.command_error(
                "%s: no samples collected during tare" % self._name)

        tare_counts = sum(s[2] for s in samples) / len(samples)
        self._load_cell.tare(int(tare_counts))

        safety_min, safety_max = self._get_safety_range()
        self._mcu_trigger_analog.set_raw_range(safety_min, safety_max)

        gpc = self._get_grams_per_count() * FRAC_GRAMS_CONV
        sos_filter = self._mcu_trigger_analog.get_sos_filter()
        sos_filter.set_offset_scale(int(-tare_counts), gpc)

        trigger_frac_grams = int(self._trigger_force * FRAC_GRAMS_CONV)
        self._mcu_trigger_analog.set_trigger("abs_ge", trigger_frac_grams)
        self._effective_mode = raw_contact.MODE_CALIBRATED

    def _start_fit_collector(self):
        toolhead = self._printer.lookup_object('toolhead')
        print_time = toolhead.get_last_move_time()
        collector = self._load_cell.get_collector()
        collector.start_collecting(min_time=print_time)
        return collector

    def _fit_contact_z(self, collector, raw_z, toolhead):
        """Retract while collecting load-cell samples, then fit to find the
        true contact Z position.

        Upstream 58bd parity: adapts TappingMove._analyze_ascent() +
        LCBestFit.find_best_fit(). The retract serves as the ascent move
        whose force-vs-position curve reveals the contact/free-air transition.

        Contact-force telemetry (Phase 2 mission, §1): `collector` was
        started in touch_probe() BEFORE the descent (probing_move) even
        began, and is drained here only after the retract completes - so
        `samples` (before it gets filtered down to `data`, the ascent-only
        window the FIT itself uses) already spans the FULL physical
        contact event: pre-contact descent, the trigger itself, any
        dwell, and the retract. Reused here, not re-collected - the same
        "existing collector/data path" this mission asks for, not a
        second load-cell implementation. Each sample is upstream
        load_cell.py's own [time, grams, counts, tare_counts] tuple
        (confirmed against the pinned source's _sensor_data_event, not
        assumed) - samples[i][1] is already force in grams, no conversion
        needed here.

        None of this runs before or influences phoming.probing_move()'s
        own trigger decision in any way - it is pure post-hoc extraction
        from data the trigger algorithm never sees.

        Two honest, bounded limitations, stated rather than hidden:
          - "force at trigger" is the LAST sample at or before
            ascent_start_time (the moment probing_move() returned) - a
            sample-period-resolution approximation of the true trigger
            instant, not a hardware-timestamped exact reading. The
            uncertainty window is one sample period (1/sensor sample
            rate) - typically ~12ms for an 80Hz HX711.
          - "peak force" is the single largest |grams| sample anywhere in
            this same continuous window (which will normally land at or
            just after the trigger, before force drops off during
            retract) - not a continuously-sampled true physical peak,
            since the sensor itself only samples at its own fixed rate.
        """
        ascent_start_time = toolhead.get_last_move_time()

        lift_pos = list(toolhead.get_position())
        lift_pos[2] += self._retract_dist
        toolhead.manual_move(lift_pos, self._retract_speed)

        move_end = toolhead.get_last_move_time()
        results = collector.collect_until(move_end)
        samples, err = results
        if err:
            errors, overflows = err
            raise self._printer.command_error(
                "%s: sensor errors during ascent: %i errors, %i overflows"
                % (self._name, errors, overflows))

        self._last_peak_force_g = None
        self._last_peak_force_time = None
        self._last_force_at_trigger_g = None
        self._last_tare_counts = None
        self._last_raw_peak_delta_counts = None
        # Raw arming in use: polarity from the plan's profile (never a
        # registry lookup - a candidate is not in any registry).
        raw_mode = self._active_profile is not None
        polarity = self._active_profile['polarity'] if raw_mode else None
        fit_mode = raw_contact.MODE_RAW if raw_mode else raw_contact.MODE_CALIBRATED
        if samples:
            self._last_tare_counts = samples[0][3]
            pre_trigger = [s for s in samples if s[0] <= ascent_start_time]
            if raw_mode:
                # Raw count telemetry; grams too when the cell happens to be
                # calibrated (the supervised comparison), never invented.
                peak_sample = max(samples, key=lambda s: abs(s[2] - s[3]))
                self._last_raw_peak_delta_counts = peak_sample[2] - peak_sample[3]
                self._last_peak_force_time = peak_sample[0]
                if peak_sample[1] is not None:
                    self._last_peak_force_g = peak_sample[1]
                if pre_trigger and pre_trigger[-1][1] is not None:
                    self._last_force_at_trigger_g = pre_trigger[-1][1]
            else:
                peak_sample = max(samples, key=lambda s: abs(s[1]))
                self._last_peak_force_g = peak_sample[1]
                self._last_peak_force_time = peak_sample[0]
                if pre_trigger:
                    self._last_force_at_trigger_g = pre_trigger[-1][1]

        data = []
        for s in samples:
            if s[0] >= ascent_start_time and \
               s[0] <= ascent_start_time + ASCENT_DATA_WINDOW_SECONDS:
                data.append((raw_contact.fit_magnitude(
                                 s, fit_mode, polarity),
                             _lookup_z_pos(toolhead, s[0])))

        # Phase 2 hot-reliability investigation (qualification-only diagnostic,
        # not a normal-operation payload): dump the exact ascent-window samples
        # the fit is about to run on, plus the full pre-filter collector output,
        # unconditionally and before either failure branch below, so a failing
        # hot touch leaves the raw (grams, z) curve in klippy.log instead of
        # only the aggregate "insufficient ascent samples" verdict.
        # logging.info, not logging.debug: this device's S55klipper init
        # script starts klippy without -v, so the root logger level is
        # INFO (see klippy.py's own debuglevel default) and DEBUG records
        # are silently dropped before ever reaching klippy.log.
        logging.info(
            "%s: raw collector samples (%d, ascent_start=%.4f window=%.4f): %s",
            self._name, len(samples), ascent_start_time,
            ASCENT_DATA_WINDOW_SECONDS, samples)
        logging.info(
            "%s: ascent fit window data (%d points, grams/z pairs): %s",
            self._name, len(data), data)

        if len(data) < 2 * FIT_MIN_POINTS:
            raise self._printer.command_error(
                "%s: insufficient ascent samples (%d total, need >= %d"
                " each) for piecewise fit"
                % (self._name, len(data), 2 * FIT_MIN_POINTS))

        z_contact, n_below, n_above, depress_slope = \
            self._best_fit.find_best_fit(data)

        if n_below < FIT_MIN_POINTS or n_above < FIT_MIN_POINTS:
            raise self._printer.command_error(
                "%s: insufficient ascent samples (%d below, %d above,"
                " need >= %d each) for piecewise fit"
                % (self._name, n_below, n_above, FIT_MIN_POINTS))

        logging.info(
            "%s fit: n_below=%d n_above=%d z_contact=%.4f raw=%.4f"
            " delta=%.4f depress_slope=%.4f",
            self._name, n_below, n_above, z_contact, raw_z,
            raw_z - z_contact, depress_slope)

        return z_contact

    def touch_probe(self, down_min_z, pro_cnt=1, minimum_allowed_z=None,
                    arming=None):
        # down_min_z: positive depth below Z=0 (e.g. 10 means Z=-10).
        # Negate to get the target Z coordinate, then clamp to stepper limit.
        #
        # minimum_allowed_z (Phase 2 mission, bounded-descent envelope):
        # an OPTIONAL additional floor, independent of down_min_z, supplied
        # by a caller that has already derived a physically-grounded bound
        # for THIS contact - either a predicted nozzle-contact plane plus a
        # small margin (an ESTABLISHED calibration, with a credible prior
        # probe z_offset) or a bounded search below a known starting
        # position (a BOOTSTRAP/virgin calibration, with no credible prior)
        # - see nebulaos_probe_pair.py's own header for both formulas. When
        # given, the actual commanded
        # contact target never exceeds it either - the tighter (higher,
        # i.e. shallower) of the two floors always wins, so this can only
        # ever make a contact SHALLOWER than down_min_z alone would allow,
        # never deeper. This is a SEPARATE, independent safety layer from
        # force_safety_limit/trigger_force (the MCU-triggered stop below)
        # - neither replaces the other; if no valid trigger occurs before
        # this bound, the phoming probing-move call below raises its own
        # "Probe did not trigger" error, and this method does not retry
        # deeper.
        toolhead = self._printer.lookup_object('toolhead')
        phoming = self._printer.lookup_object('homing')
        z_floor = max(self._z_min_position, -down_min_z)
        results = []

        # A raw plan is used only for this call: every touch re-arms and
        # clears the previous arming first, so nothing leaks into a later call.
        for i in range(pro_cnt):
            self._tare_and_arm(arming)

            collector = self._start_fit_collector()

            contact_floor = z_floor if minimum_allowed_z is None \
                else max(z_floor, minimum_allowed_z)
            pos = list(toolhead.get_position())
            pos[2] = contact_floor
            epos = phoming.probing_move(
                self._mcu_trigger_analog, pos, self._contact_speed)

            raw_z = epos[2]
            fitted_z = self._fit_contact_z(collector, raw_z, toolhead)

            if not math.isfinite(raw_z) or not math.isfinite(fitted_z):
                raise self._printer.command_error(
                    "%s: contact result is not finite (raw=%r fitted=%r)"
                    % (self._name, raw_z, fitted_z))

            self._last_raw_trigger_z = raw_z
            self._last_fitted_contact_z = fitted_z
            self._last_fit_delta = raw_z - fitted_z

            results.append(fitted_z)

        if not results:
            raise self._printer.command_error(
                "%s: no probe results" % self._name)

        return sum(results) / len(results)

    def get_status(self, eventtime):
        trig_time = self._mcu_trigger_analog.get_last_trigger_time()
        calibrated = self._load_cell.is_calibrated()
        kind, _ = self.plan_arming(raw_contact.CALLER_Z_OFFSET_PAIR)
        return {
            'contact_detection_mode': self._contact_mode,
            # The arming the last touch actually used.
            'effective_contact_mode': self._effective_mode,
            # Ready for nozzle contact on the canonical Z-offset path.
            'contact_qualified': kind is not None,
            'raw_profile_id': self._raw_profile_id,
            'last_contact_rejection_reason': self._last_contact_rejection_reason,
            'last_raw_peak_delta_counts': self._last_raw_peak_delta_counts,
            'raw_diagnostic': self._raw_diag,
            'last_trigger_time': trig_time,
            'is_calibrated': self._load_cell.is_calibrated(),
            'last_raw_trigger_z': self._last_raw_trigger_z,
            'last_fitted_contact_z': self._last_fitted_contact_z,
            'last_fit_delta': self._last_fit_delta,
            'last_peak_force_g': self._last_peak_force_g,
            'last_peak_force_time': self._last_peak_force_time,
            'last_force_at_trigger_g': self._last_force_at_trigger_g,
            'last_tare_counts': self._last_tare_counts,
            'trigger_force': self._trigger_force,
            'force_safety_limit': self._force_safety_limit,
            'contact_speed': self._contact_speed,
        }

    cmd_raw_contact_diagnostic_help = (
        "No-motion load cell health check (G83b): idle noise, drift, sample "
        "rate, saturation and stuck-signal checks on raw ADC counts")

    def cmd_raw_contact_diagnostic(self, gcmd):
        """NO MOTION: collects SECONDS of idle samples and reports raw-count
        statistics. Never moves, never tares, never changes config."""
        seconds = gcmd.get_float('SECONDS', 2., minval=0.5, maxval=10.)
        print_stats = self._printer.lookup_object('print_stats', None)
        if print_stats is not None:
            state = print_stats.get_status(
                self._printer.get_reactor().monotonic()).get('state')
            if state in ('printing', 'paused'):
                raise gcmd.error("NEBULAOS_RAW_CONTACT_DIAGNOSTIC: not while "
                                 "a print is %s" % (state,))
        reactor = self._printer.get_reactor()
        collector = self._load_cell.get_collector()
        collector.start_collecting()
        reactor.pause(reactor.monotonic() + seconds)
        samples, errors = collector.stop_collecting()
        stats, reasons = raw_contact.validate_idle(
            samples, errors, self._load_cell.saturation_range(),
            self._sensor.get_samples_per_second())
        self._raw_diag = dict(stats)
        self._raw_diag['reasons'] = list(reasons)
        self._raw_diag['seconds'] = seconds
        if stats['n']:
            gcmd.respond_info(
                "raw contact diagnostic: %d samples, %.1f SPS (configured %.1f),"
                " median %d counts, noise sigma %.1f counts, peak-to-peak %d,"
                " drift %.1f counts/s, %d distinct values"
                % (stats['n'], stats['sps'] or 0.,
                   self._sensor.get_samples_per_second(), stats['median'],
                   stats['sigma'], stats['p2p'], stats['drift_per_s'] or 0.,
                   stats['unique']))
        gcmd.respond_info("raw contact diagnostic: %s"
                          % ("healthy" if not reasons
                             else "NOT healthy: " + ", ".join(reasons)))


# appended to ZOffsetProbe below (keeps the class body readable)
def _cmd_raw_contact_reference(self, gcmd):
    """NO MOTION: capture the unloaded reference R from an idle window on an
    empty bed and stage it in the SAVE_CONFIG autosave block (Klipper
    restarts). Refused on an unhealthy sensor."""
    print_stats = self._printer.lookup_object('print_stats', None)
    if print_stats is not None:
        state = print_stats.get_status(
            self._printer.get_reactor().monotonic()).get('state')
        if state in ('printing', 'paused'):
            raise gcmd.error("NEBULAOS_RAW_CONTACT_REFERENCE: not while a "
                             "print is %s" % (state,))
    seconds = gcmd.get_float('SECONDS', 2., minval=1., maxval=10.)
    toolhead = self._printer.lookup_object('toolhead')
    start = toolhead.get_last_move_time()
    collector = self._load_cell.get_collector()
    collector.start_collecting(min_time=start)
    samples, errors = collector.collect_until(start + seconds)
    stats, reasons = raw_contact.validate_idle(
        samples, errors, self._load_cell.saturation_range(),
        self._sensor.get_samples_per_second())
    if reasons:
        raise gcmd.error("NEBULAOS_RAW_CONTACT_REFERENCE: the load cell is "
                         "not healthy (%s) - nothing saved"
                         % (", ".join(reasons),))
    reference = int(stats['median'])
    configfile = self._printer.lookup_object('configfile')
    configfile.set(self._name, 'raw_reference_counts', reference)
    self._raw_reference = reference
    gcmd.respond_info("NEBULAOS_RAW_CONTACT_REFERENCE: empty-bed reference "
                      "%d counts (noise sigma %.0f, drift %.1f counts/s) - "
                      "saving, Klipper restarts"
                      % (reference, stats['sigma'], stats['drift_per_s'] or 0.))
    self._printer.lookup_object('gcode').run_script_from_command('SAVE_CONFIG')


ZOffsetProbe.cmd_raw_contact_reference = _cmd_raw_contact_reference
ZOffsetProbe.cmd_raw_contact_reference_help = (
    "No-motion: capture the empty-bed load cell reference for weight-free "
    "contact (G83b) and save it (SAVE_CONFIG). The bed must be empty.")


def load_config(config):
    return ZOffsetProbe(config)
