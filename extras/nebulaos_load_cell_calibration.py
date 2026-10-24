# NebulaOS guided load cell calibration with a water bottle (G86)
#
# The KE has ONE load cell, under the front-left bed screw; the bed rests on
# it and on spacers at the other three corners, and the Z-offset nozzle touch
# is right above it. This flow teaches the cell how many ADC counts one gram
# is, with a weighed 1.5 L water bottle standing centred on that screw:
#
#   1 empty bed      -> sensor health check + zero (stable reading)
#   2 bottle on      -> weight from the kitchen scale (1500 g pre-filled,
#                       +/- buttons so Mainsail needs no typing) -> counts/g
#   3 lift, put back -> the reading must agree with the entered weight
#   4 remove         -> the reading must return to zero
#   5 finger press   -> optional: shows the measured force
#   6 save           -> upstream set_calibration() + SAVE_CONFIG
#
# Why the checks look like this (measured on ke-dev, Run 13):
# - The Z-offset result does not depend on counts_per_gram (the upstream fit
#   is scale-invariant; 9 real contacts). The calibration only scales
#   trigger_force and force_safety_limit, so a few percent do not matter.
# - A bottle off the screw reads LOW -> the printer believes the cell is
#   less sensitive -> it stops a touch EARLIER: the safe direction.
# - A too-LOW typed weight is the dangerous mistake (150 instead of 1500 ->
#   10x the real force), so the entered weight has a fixed range.
# - A weight on the bed LOWERS the raw counts on the KE; every number shown
#   to the user is a magnitude.
#
# The tolerances are config options. They were set provisionally and then
# confirmed by the no-motion measurement on ke-dev, 2026-10-20 (a 1537 g
# bottle on a kitchen scale): exactly above the screw 9x 1637-1672 g
# (+-1.1%), a careless placement -4.5%, a rough one -22%, back to zero 2-8 g
# (7x); two full calibrations 90.34 and 87.91 counts/g (3% apart).
#
# Nothing here moves or heats the printer. Nothing is saved before step 6.
#
# G99 (Tim 2026-10-22): automatic load-cell contact is the default. This
# object also owns the user's choice (Calibration > Load Cell):
#   NEBULAOS_LOAD_CELL_MODE [MODE=auto|manual]  persist the choice (no
#       restart; no MODE: report). Manual = this bottle calibration, used
#       until automatic is selected again; saving a bottle calibration
#       (Apply & Save) selects manual.
#   NEBULAOS_LOAD_CELL_AUTO_CHECK  no motion, empty bed: sensor health +
#       the unloaded reference for automatic contact, stored in the same
#       state file (no SAVE_CONFIG, no restart).
# Status: mode, auto_check (never|ok|failed), auto_check_reason, auto_ready
# (see nebulaos_z_offset_probe.auto_status / nebulaos_load_cell_mode).
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging

from . import nebulaos_raw_contact as raw_contact

_MAX_ERROR_LEN = 200

# Flow states (stable strings: the screen and Mainsail read them).
IDLE = 'idle'
AWAITING_EMPTY = 'awaiting_empty'
AWAITING_WEIGHT = 'awaiting_weight'
AWAITING_REPLACE = 'awaiting_replace'
AWAITING_REMOVE = 'awaiting_remove'
AWAITING_FINGER = 'awaiting_finger'
AWAITING_CONFIRMATION = 'awaiting_confirmation'
COMPLETE = 'complete'
CANCELLED = 'cancelled'
ERROR = 'error'
ACTIVE = (AWAITING_EMPTY, AWAITING_WEIGHT, AWAITING_REPLACE, AWAITING_REMOVE,
          AWAITING_FINGER, AWAITING_CONFIRMATION)

TITLE = "Load Cell Calibration"
CMD = "NEBULAOS_LOAD_CELL_CALIBRATE"
MODE_CMD = "NEBULAOS_LOAD_CELL_MODE"
CHECK_CMD = "NEBULAOS_LOAD_CELL_AUTO_CHECK"
MANUAL_HINT = "use Calibration > Load Cell > Manual (bottle calibration)"
_AUTO_FIELDS = ('mode', 'mode_choice', 'auto_check', 'auto_check_reason',
                'auto_check_time', 'auto_ready', 'auto_profile',
                'auto_reference_counts', 'mode_warning')
CANCEL = "_NEBULAOS_LOAD_CELL_CANCEL"
ADJUST_STEPS = (-100, -10, -1, 1, 10, 100)

# Readable reasons for raw_contact.validate_idle() codes.
_SENSOR_PROBLEMS = raw_contact.IDLE_REASON_TEXT


def _sanitize_error(exc):
    text = str(exc).strip() or exc.__class__.__name__
    text = ' '.join(text.split())
    if len(text) > _MAX_ERROR_LEN:
        text = text[:_MAX_ERROR_LEN - 3].rstrip() + '...'
    return text


class NebulaOSLoadCellCalibration:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        # Agreed with Tim (Run 13): a 1.5 L bottle weighed on a kitchen
        # scale; only 1000-5000 g are accepted.
        self.min_grams = config.getfloat('min_grams', 1000., minval=500.)
        self.max_grams = config.getfloat('max_grams', 5000.,
                                         above=self.min_grams)
        self.default_grams = config.getfloat(
            'default_grams', 1500., minval=self.min_grams,
            maxval=self.max_grams)
        self.window_seconds = config.getfloat('window_seconds', 2.,
                                              minval=0.5, maxval=10.)
        # Above this the reading is "not stable" (ke-dev idle drift 24
        # counts/s; no false alarm in the 2026-10-20 calibrations).
        self.stable_drift = config.getfloat(
            'stable_drift_counts_per_s', 300., above=0.)
        # Lift-and-put-back must agree within this percent (measured: +-1.1%
        # careful, -4.5% careless; a clearly wrong placement is >10%).
        self.replace_tolerance = config.getfloat(
            'replace_tolerance_percent', 10., above=0., maxval=50.)
        # Grams the empty bed may differ from the zero after the bottle is
        # removed (measured: 2-8 g on ke-dev).
        self.zero_tolerance = config.getfloat(
            'zero_tolerance_grams', 30., above=0.)
        # A result this far from the previous calibration is shown as a
        # warning before Apply & Save (measured: rough vs exact placement
        # differs 22%; two careful runs 3%).
        self.previous_change = config.getfloat(
            'previous_change_percent', 25., above=0.)
        # An empty bed this far (grams, previous calibration) from the saved
        # zero gets a "something on the bed?" note (no false alarm on
        # 2026-10-20; a tramming shim shifted the zero ~50 g).
        self.empty_shift = config.getfloat('empty_shift_grams', 100., above=0.)

        self.run_id = 0
        self._reset(IDLE)
        self.gcode.register_command(
            '_' + CMD, self.cmd_calibrate, desc=self.cmd_calibrate_help)
        self.gcode.register_command(
            CANCEL, self.cmd_cancel, desc="Cancel the guided load cell "
            "calibration - nothing is changed")
        self.gcode.register_command(
            'NEBULAOS_LOAD_CELL_READ', self.cmd_read,
            desc="Read-only: the force on the front-left corner in grams "
            "(1 s average, current calibration). Never moves anything.")
        self.gcode.register_command(
            MODE_CMD, self.cmd_mode, desc=self.cmd_mode_help)
        self.gcode.register_command(
            CHECK_CMD, self.cmd_auto_check, desc=self.cmd_auto_check_help)

    def _reset(self, state):
        self.state = state
        self.error = None
        self.problem = None
        self.tare_counts = None
        self.pending_grams = self.default_grams
        self.grams = None
        self.counts_per_gram = None
        self.replace_grams = None
        self.zero_grams = None
        self.finger_grams = None
        self.old_counts_per_gram = None
        self.old_tare_counts = None
        self.change_percent = None

    def get_status(self, eventtime):
        return {
            'id': self.run_id,
            'state': self.state,
            # G91: a guided flow is running or waiting for the user.
            'busy': self.state in ACTIVE,
            'error': self.error,
            'problem': self.problem,
            'calibrated': self._calibrated(),
            'pending_grams': self.pending_grams,
            'min_grams': self.min_grams,
            'max_grams': self.max_grams,
            'grams': self.grams,
            'counts_per_gram': self.counts_per_gram,
            'replace_grams': self.replace_grams,
            'zero_grams': self.zero_grams,
            'finger_grams': self.finger_grams,
            'old_counts_per_gram': self.old_counts_per_gram,
            'change_percent': self.change_percent,
            # The screen shows the same warning as the prompt (G90).
            'change_warning': (self.change_percent is not None
                               and abs(self.change_percent) > self.previous_change),
            **self._auto_status(),
        }

    def _probe(self):
        probe = self.printer.lookup_object('nebulaos_z_offset_probe', None)
        if probe is None or not hasattr(probe, 'auto_status'):
            return None
        return probe

    def _auto_status(self):
        """G99 fields; all None without a load cell probe."""
        probe = self._probe()
        if probe is None:
            return {k: None for k in _AUTO_FIELDS}
        try:
            return probe.auto_status()
        except Exception:
            return {k: None for k in _AUTO_FIELDS}

    # ------------------------------------------------------------------
    # sensor access
    # ------------------------------------------------------------------
    def _load_cell(self):
        probe = self.printer.lookup_object('nebulaos_z_offset_probe', None)
        if probe is None or not hasattr(probe, 'get_load_cell'):
            raise self.printer.command_error(
                "%s: no [nebulaos_z_offset_probe] load cell is configured on "
                "this printer" % (CMD,))
        return probe.get_load_cell()

    def _calibrated(self):
        try:
            return bool(self._load_cell().is_calibrated())
        except Exception:
            return None

    def _window(self, lc):
        """window_seconds of raw samples: (stats, reasons, median counts).
        reasons is validate_idle()'s list - empty means a healthy sensor."""
        collector = lc.get_collector()
        collector.start_collecting()
        self.reactor.pause(self.reactor.monotonic() + self.window_seconds)
        samples, errors = collector.stop_collecting()
        stats, reasons = raw_contact.validate_idle(
            samples, errors, lc.saturation_range(),
            lc.get_sensor().get_samples_per_second())
        return stats, reasons, stats['median']

    def _sensor_problem(self, reasons):
        return ("The load cell sensor is not working properly (%s). Check the "
                "cable under the front-left corner."
                % (", ".join(_SENSOR_PROBLEMS.get(r, r) for r in reasons),))

    def _unstable(self, stats):
        drift = stats['drift_per_s']
        return drift is not None and abs(drift) > self.stable_drift

    def _grams(self, counts):
        return abs(counts - self.tare_counts) / self.counts_per_gram

    def _refuse_if_printing(self):
        print_stats = self.printer.lookup_object('print_stats', None)
        if print_stats is None:
            return
        state = print_stats.get_status(self.reactor.monotonic()).get('state')
        if state in ('printing', 'paused'):
            raise self.printer.command_error(
                "%s: not possible while a print is %s" % (CMD, state))

    def _require(self, *states):
        if self.state not in states:
            raise self.printer.command_error(
                "%s: that step is not possible now (state=%s) - start a "
                "calibration with %s" % (CMD, self.state, CMD))

    # ------------------------------------------------------------------
    # prompts (same text on the screen and in Mainsail)
    # ------------------------------------------------------------------
    def _prompt(self, lines, buttons):
        if self.problem:
            lines = ["Problem: " + self.problem, ""] + list(lines)
        self.gcode.respond_raw("// action:prompt_end")
        self.gcode.respond_raw("// action:prompt_begin %s" % (TITLE,))
        for line in lines:
            self.gcode.respond_raw("// action:prompt_text %s" % (line,))
        for label, command in list(buttons) + [("Cancel", CANCEL)]:
            self.gcode.respond_raw(
                "// action:prompt_button %s|%s" % (label, command))
        self.gcode.respond_raw("// action:prompt_show")

    def _dismiss(self):
        self.gcode.respond_raw("// action:prompt_end")

    def _show(self):
        if self.state == AWAITING_EMPTY:
            self._prompt(
                ["Step 1 of 5: remove everything from the bed.",
                 "The nozzle must not touch the bed.",
                 "Then tap Check."],
                [("Check", CMD + " CHECK=1")])
        elif self.state == AWAITING_WEIGHT:
            g = self.pending_grams
            self._prompt(
                ["Step 2 of 5: fill a 1.5 L bottle with water and weigh it",
                 "on a kitchen scale.",
                 "Put the bottle with its centre roughly above the",
                 "front-left screw head of the bed.",
                 "Set the weight from the scale with the buttons (or type",
                 "%s GRAMS=<g>), then tap Weigh." % (CMD,),
                 "Weight: %.0f g" % (g,)],
                [("%+d" % (d,), "%s ADJUST=%d" % (CMD, d))
                 for d in ADJUST_STEPS]
                + [("Weigh %.0f g" % (g,), CMD + " WEIGH=1")])
        elif self.state == AWAITING_REPLACE:
            self._prompt(
                ["Step 3 of 5: lift the bottle and put it back in the same",
                 "place, centred above the front-left screw head.",
                 "Then tap Continue."],
                [("Continue", CMD + " NEXT=1")])
        elif self.state == AWAITING_REMOVE:
            lines = []
            if self.replace_grams is not None:
                lines.append("Printer measures %.0f g, your scale said %.0f g."
                             % (self.replace_grams, self.grams))
            lines += ["Step 4 of 5: remove the bottle from the bed.",
                      "Then tap Continue."]
            self._prompt(lines, [("Continue", CMD + " NEXT=1")])
        elif self.state == AWAITING_FINGER:
            lines = ["Step 5 of 5 (optional): press the front-left corner of",
                     "the bed gently with a finger and tap Measure while",
                     "pressing. The printer shows the force."]
            if self.finger_grams is not None:
                lines.append("Printer measures %.0f g." % (self.finger_grams,))
            self._prompt(lines, [("Measure", CMD + " MEASURE=1"),
                                 ("Continue", CMD + " NEXT=1")])
        elif self.state == AWAITING_CONFIRMATION:
            lines = ["Calibration done: %.2f counts per gram." % (
                self.counts_per_gram,)]
            if self.old_counts_per_gram:
                lines.append("Previous calibration: %.2f counts per gram "
                             "(%+.0f%%)." % (self.old_counts_per_gram,
                                             self.change_percent))
                if abs(self.change_percent) > self.previous_change:
                    lines.append("This is very different from before. Only "
                                 "save if the bottle stood right and its "
                                 "weight is correct.")
            lines.append("Apply & Save restarts Klipper.")
            self._prompt(lines, [("Apply & Save", CMD + " ACCEPT=1")])

    def _problem(self, text):
        """A step check failed: stay in the step, say why, retry."""
        self.problem = text
        self.gcode.respond_info("%s: %s" % (CMD, text))
        self._show()

    # ------------------------------------------------------------------
    # commands
    # ------------------------------------------------------------------
    cmd_calibrate_help = (
        "Guided load cell calibration with a water bottle - call with no "
        "params to start; CHECK=1 (empty bed), ADJUST=<+-g> / WEIGH=1 or "
        "GRAMS=<g> (bottle on), NEXT=1 (next step), MEASURE=1 (finger "
        "test), ACCEPT=1 (save)")

    def cmd_calibrate(self, gcmd):
        grams = gcmd.get_float('GRAMS', None)
        adjust = gcmd.get_int('ADJUST', None)
        if gcmd.get_int('ACCEPT', 0):
            self._accept()
        elif grams is not None:
            self._weigh(grams)
        elif adjust is not None:
            self._adjust(adjust)
        elif gcmd.get_int('WEIGH', 0):
            self._weigh(self.pending_grams)
        elif gcmd.get_int('CHECK', 0) or gcmd.get_int('TARE', 0):
            self._check_empty()
        elif gcmd.get_int('MEASURE', 0):
            self._measure_finger()
        elif gcmd.get_int('NEXT', 0):
            self._next()
        else:
            self._start()

    def _start(self):
        self._refuse_if_printing()
        if self.state in ACTIVE:
            # Reconnect / second interface: show where we are, do not restart.
            self._show()
            return
        lc = self._load_cell()
        self.run_id += 1
        self._reset(AWAITING_EMPTY)
        if lc.is_calibrated():
            self.old_counts_per_gram = lc.get_counts_per_gram()
            self.old_tare_counts = lc.get_reference_tare_counts()
        self._show()

    def _check_empty(self):
        self._require(AWAITING_EMPTY, AWAITING_WEIGHT)
        self._refuse_if_printing()
        lc = self._load_cell()
        self.state = AWAITING_EMPTY
        stats, reasons, median = self._window(lc)
        if reasons:
            return self._problem(self._sensor_problem(reasons))
        if self._unstable(stats):
            return self._problem(
                "The reading is not stable. Remove everything from the bed, "
                "do not touch the printer, and tap Check again.")
        self.tare_counts = int(median)
        self.problem = None
        self.state = AWAITING_WEIGHT
        if self.old_counts_per_gram and self.old_tare_counts is not None:
            shift = (abs(self.tare_counts - self.old_tare_counts)
                     / self.old_counts_per_gram)
            if shift > self.empty_shift:
                # Not a refusal: the zero is what it is now. Say it once.
                self.gcode.respond_info(
                    "%s: the empty bed reads %.0f g away from the last saved "
                    "zero - if something is on the bed, remove it and tap "
                    "Check again." % (CMD, shift))
        self._show()

    def _adjust(self, delta):
        self._require(AWAITING_WEIGHT)
        self.pending_grams = min(self.max_grams,
                                 max(self.min_grams, self.pending_grams + delta))
        self.problem = None
        self._show()

    def _weigh(self, grams):
        self._require(AWAITING_WEIGHT)
        self._refuse_if_printing()
        if not (self.min_grams <= grams <= self.max_grams):
            return self._problem(
                "Enter the weight in grams, between %.0f and %.0f."
                % (self.min_grams, self.max_grams))
        self.pending_grams = grams
        lc = self._load_cell()
        stats, reasons, median = self._window(lc)
        if raw_contact.SATURATED in reasons:
            return self._problem("The weight is too heavy. Use a lighter one.")
        if reasons:
            return self._problem(self._sensor_problem(reasons))
        if self._unstable(stats):
            return self._problem(
                "The reading is not stable. Wait until the bottle stands "
                "still, then tap Weigh again.")
        delta = abs(int(median) - self.tare_counts)
        if delta == 0:
            return self._problem(
                "The printer did not feel the weight. Is the bottle on the "
                "bed? Is the sensor connected?")
        # Upstream's own rule (LOAD_CELL_CALIBRATE): >= 1% of full scale.
        change = abs(lc.counts_to_percent(median)
                     - lc.counts_to_percent(self.tare_counts))
        if change < 1.:
            return self._problem(
                "The printer barely felt the weight (%.2f%%, 1%% is needed). "
                "Check the bottle stands centred above the front-left screw "
                "head, or use a heavier weight (for example two bottles)."
                % (change,))
        self.grams = grams
        self.counts_per_gram = float(delta) / grams
        self.problem = None
        self.state = AWAITING_REPLACE
        self._show()

    def _next(self):
        self._require(AWAITING_REPLACE, AWAITING_REMOVE, AWAITING_FINGER)
        if self.state == AWAITING_FINGER:
            return self._finish()
        lc = self._load_cell()
        stats, reasons, median = self._window(lc)
        if reasons:
            return self._problem(self._sensor_problem(reasons))
        if self._unstable(stats):
            return self._problem(
                "The reading is not stable. Wait until nothing moves, then "
                "tap Continue again.")
        measured = self._grams(median)
        if self.state == AWAITING_REPLACE:
            self.replace_grams = measured
            diff = abs(measured - self.grams) / self.grams * 100.
            if diff > self.replace_tolerance:
                return self._problem(
                    "The printer measures %.0f g, your scale said %.0f g. Is "
                    "the bottle touching something? Put it back centred above "
                    "the front-left screw head and tap Continue again."
                    % (measured, self.grams))
            self.problem = None
            self.state = AWAITING_REMOVE
            return self._show()
        self.zero_grams = measured
        if measured > self.zero_tolerance:
            return self._problem(
                "The bed does not return to zero (%.0f g left). Check that "
                "nothing is stuck on or under the bed, then tap Continue "
                "again." % (measured,))
        self.problem = None
        self.state = AWAITING_FINGER
        self._show()

    def _measure_finger(self):
        self._require(AWAITING_FINGER)
        stats, reasons, median = self._window(self._load_cell())
        if reasons and reasons != [raw_contact.STUCK]:
            return self._problem(self._sensor_problem(reasons))
        self.finger_grams = self._grams(median)
        self.problem = None
        self._show()

    def _finish(self):
        if self.old_counts_per_gram:
            self.change_percent = (self.counts_per_gram
                                   / self.old_counts_per_gram - 1.) * 100.
        self.problem = None
        self.state = AWAITING_CONFIRMATION
        self._show()

    def _accept(self):
        self._require(AWAITING_CONFIRMATION)
        self._refuse_if_printing()
        lc = self._load_cell()
        try:
            # Upstream's own persistence: live values + the SAVE_CONFIG
            # autosave block of [nebulaos_z_offset_probe].
            lc.set_calibration(self.counts_per_gram, self.tare_counts)
            lc.tare(self.tare_counts)
            # G99: a saved bottle calibration means Manual - "the manual
            # value is used until automatic is selected again" (Tim).
            probe = self._probe()
            if probe is not None:
                warning = probe.set_load_cell_mode(raw_contact.USER_MANUAL)
                if warning:
                    self.gcode.respond_info("%s: %s" % (CMD, warning))
        except Exception as e:
            self.state = ERROR
            self.error = _sanitize_error(e)
            self._dismiss()
            raise
        self.state = COMPLETE
        self._dismiss()
        self._record("Manual (bottle): %.1f counts per gram"
                     % (self.counts_per_gram,))
        self.gcode.respond_info(
            "%s: %.5f counts per gram, zero %d - saving, Klipper restarts"
            % (CMD, self.counts_per_gram, self.tare_counts))
        self.gcode.run_script_from_command('SAVE_CONFIG')

    def _record(self, summary):
        """G105: a new load cell result - history 'load_cell' (clears its
        Redo). Owned by [nebulaos_calibration]; never fails this flow."""
        cal = self.printer.lookup_object('nebulaos_calibration', None)
        if cal is None or not hasattr(cal, 'record_flow'):
            return
        try:
            cal.record_flow('load_cell', summary)
        except Exception:
            logging.exception("nebulaos_load_cell_calibration: calibration "
                              "history not updated")

    def cmd_cancel(self, gcmd):
        if self.state not in ACTIVE:
            raise gcmd.error("No load cell calibration is in progress")
        self._reset(CANCELLED)
        self._dismiss()
        gcmd.respond_info("Load cell calibration cancelled - nothing was "
                          "changed.")

    def cmd_read(self, gcmd):
        lc = self._load_cell()
        if not lc.is_calibrated():
            raise gcmd.error("NEBULAOS_LOAD_CELL_READ: the load cell is not "
                             "calibrated yet - run %s" % (CMD,))
        tare = lc.get_tare_counts()
        if tare is None:
            tare = lc.get_reference_tare_counts()
        counts = lc.avg_counts()
        gcmd.respond_info("Load cell: %.0f g on the front-left corner"
                          % (abs(counts - tare) / lc.get_counts_per_gram(),))


    # ------------------------------------------------------------------
    # G99: automatic | manual
    # ------------------------------------------------------------------
    cmd_mode_help = (
        "Load cell contact mode: MODE=auto (automatic, no bottle - the "
        "default) or MODE=manual (the bottle calibration). No MODE: report. "
        "Saved at once, no restart.")

    def _require_probe(self, cmd):
        probe = self._probe()
        if probe is None:
            raise self.printer.command_error(
                "%s: no [nebulaos_z_offset_probe] load cell is configured on "
                "this printer" % (cmd,))
        return probe

    def cmd_mode(self, gcmd):
        probe = self._require_probe(MODE_CMD)
        mode = gcmd.get('MODE', None)
        if mode is not None:
            mode = mode.strip().lower()
            if mode not in raw_contact.USER_MODES:
                raise gcmd.error("%s: MODE must be auto or manual" % (MODE_CMD,))
            warning = probe.set_load_cell_mode(mode)
            if warning:
                gcmd.respond_info("%s: %s" % (MODE_CMD, warning))
        st = probe.auto_status()
        if st['mode'] == raw_contact.USER_AUTO:
            if st['auto_ready']:
                detail = "ready (automatic check passed)"
            elif st['auto_check'] == 'failed':
                detail = ("NOT ready - the automatic check failed (%s); run "
                          "%s again on an empty bed or %s"
                          % (st['auto_check_reason'], CHECK_CMD, MANUAL_HINT))
            else:
                detail = ("the automatic check runs at the next Z-offset "
                          "calibration (or run %s on an empty bed)"
                          % (CHECK_CMD,))
        else:
            detail = ("bottle calibration %s" % (
                "present" if self._calibrated() else
                "MISSING - run %s or select MODE=auto" % (CMD,)))
        forced = st['mode'] != st['mode_choice']
        gcmd.respond_info("%s: %s%s - %s" % (
            MODE_CMD, st['mode'],
            " (forced by contact_detection_mode in the config)" if forced
            else "", detail))

    cmd_auto_check_help = (
        "No motion: automatic load cell check on an EMPTY bed - sensor "
        "health and the unloaded reference for automatic contact (saved "
        "at once, no restart)")

    def cmd_auto_check(self, gcmd):
        probe = self._require_probe(CHECK_CMD)
        self._refuse_if_printing()
        if self.state in ACTIVE:
            raise gcmd.error("%s: the bottle calibration is running - finish "
                             "or cancel it first" % (CHECK_CMD,))
        ok, reason = probe.run_auto_check()
        if not ok:
            raise gcmd.error("%s: automatic check FAILED - %s. Remove "
                             "everything from the bed and try again, or %s."
                             % (CHECK_CMD, reason, MANUAL_HINT))
        self._record("Automatic check passed")
        st = probe.auto_status()
        gcmd.respond_info(
            "%s: passed - empty-bed reference %d counts saved%s%s"
            % (CHECK_CMD, st['auto_reference_counts'],
               "" if st['mode'] == raw_contact.USER_AUTO else
               " (mode is manual: select automatic to use it)",
               " - " + st['mode_warning'] if st['mode_warning'] else ""))


def load_config(config):
    return NebulaOSLoadCellCalibration(config)
