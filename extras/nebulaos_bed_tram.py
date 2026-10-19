# NebulaOS guided bed tramming (G87, simplified version agreed with Tim)
#
# One instruction at a time, the same prompts on the screen and in Mainsail:
#   measure the four corners (upstream SCREWS_TILT_CALCULATE, 3 samples,
#   median) -> a corner within `tolerance` is OK and is never selected again
#   -> ONE action for the worst remaining corner -> the user confirms ->
#   measure again -> done when all four are OK, then offer Z offset + bed
#   mesh -> stop with an explanation when a corner does not improve after
#   `max_tries` adjustments.
#
# Mounts (asked once, remembered):
#   rigid      - stock KE spacers: add a printed shim; reference = the
#                highest corner, shim = highest - corner, rounded DOWN to the
#                shim step so a suggestion never overshoots.
#   adjustable - rubber or springs: turn the screw; upstream's direction and
#                amount with screw1 (front-left) as the base, so the
#                load-cell corner is never turned.
# The load cell sits under the FRONT-LEFT screw: a shim there changes the
# load cell mounting, so the flow ends by asking for a load cell
# calibration (G86).
#
# Upstream Klipper is unchanged: [screws_tilt_adjust] is configured in
# calibration.cfg (screw1..4 = front-left, front-right, rear-right,
# rear-left, as nozzle positions over each screw for the probe).
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import json
import math
import os

from . import nebulaos_calibration_journal as journal

TITLE = "Bed Tramming"
CMD = "NEBULAOS_BED_TRAM"
CANCEL = "_NEBULAOS_BED_TRAM_CANCEL"
CORNERS = ('front left', 'front right', 'rear right', 'rear left')
LOAD_CELL_CORNER = 0
RIGID = 'rigid'
ADJUSTABLE = 'adjustable'
MOUNTS = (RIGID, ADJUSTABLE)

IDLE = 'idle'
AWAITING_MOUNT = 'awaiting_mount'
AWAITING_ADJUSTMENT = 'awaiting_adjustment'
COMPLETE = 'complete'
STOPPED = 'stopped'
CANCELLED = 'cancelled'
ERROR = 'error'
ACTIVE = (AWAITING_MOUNT, AWAITING_ADJUSTMENT)

DEFAULT_STATE_PATH = journal.DEFAULT_JOURNAL_DIR + "/bed_tram.json"


def deviations(zs, mount, base_index=0):
    """How far each corner is from where it should be, in mm (>= 0).
    rigid: below the highest corner (what a shim must add);
    adjustable: away from the base corner (what a turn must correct)."""
    if mount == RIGID:
        top = max(zs)
        return [top - z for z in zs]
    base = zs[base_index]
    return [abs(z - base) for z in zs]


def shim_mm(deviation, step, max_shim):
    """Shim thickness for one round: a whole number of `step`, never more
    than the deviation (no overshoot) and never more than `max_shim`."""
    n = int(math.floor(deviation / step + 1e-9))
    return min(n * step, max_shim)


def turn_words(adjust):
    """'HH:MM' from upstream (turns:minutes) -> plain words."""
    turns, minutes = (int(p) for p in adjust.split(':'))
    eighths = int(round(minutes / 7.5))
    if eighths == 8:
        turns, eighths = turns + 1, 0
    parts = []
    if turns:
        parts.append("%d full turn%s" % (turns, "" if turns == 1 else "s"))
    if eighths:
        frac = {1: "an eighth", 2: "a quarter", 3: "three eighths",
                4: "a half", 5: "five eighths", 6: "three quarters",
                7: "seven eighths"}[eighths]
        parts.append(frac + " of a turn")
    return " and ".join(parts) or "a tiny bit"


class NebulaOSBedTram:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.tolerance = config.getfloat('tolerance', 0.10, above=0.,
                                         maxval=1.)
        self.shim_step = config.getfloat('shim_step', 0.10, above=0.,
                                         maxval=1.)
        self.max_shim = config.getfloat('max_shim', 0.50,
                                        minval=self.shim_step, maxval=2.)
        self.max_tries = config.getint('max_tries', 2, minval=1, maxval=5)
        # Less than this (mm) is "did not improve" after an adjustment.
        self.min_improvement = config.getfloat('min_improvement', 0.02,
                                               above=0.)
        self.samples = config.getint('samples', 3, minval=1, maxval=10)
        self.state_path = config.get('state_path', DEFAULT_STATE_PATH)
        self.mount = self._load_mount()
        self.run_id = 0
        self._reset(IDLE)
        # Private command; the public NEBULAOS_BED_TRAM is a gcode_macro in
        # calibration.cfg (Mainsail lists macros only).
        self.gcode.register_command('_' + CMD, self.cmd_tram,
                                    desc=self.cmd_tram_help)
        self.gcode.register_command(CANCEL, self.cmd_cancel,
                                    desc="Cancel guided bed tramming")

    def _reset(self, state):
        self.state = state
        self.error = None
        self.message = None
        self.zs = None
        self.devs = None
        self.accepted = [False] * 4
        self.tries = [0] * 4
        self.target = None
        self.target_before = None
        self.action = None
        self.load_cell_corner_changed = False

    # --- persistence of the mount choice -------------------------------
    def _load_mount(self):
        try:
            with open(self.state_path) as handle:
                mount = json.load(handle).get('mount')
        except (IOError, OSError, ValueError, AttributeError):
            return None
        return mount if mount in MOUNTS else None

    def _save_mount(self, mount):
        directory = os.path.dirname(self.state_path)
        if directory and not os.path.isdir(directory):
            os.makedirs(directory)
        journal.atomic_write_json(self.state_path + ".tmp", self.state_path,
                                  {'schema': 1, 'mount': mount})

    def get_status(self, eventtime):
        return {
            'id': self.run_id,
            'state': self.state,
            'error': self.error,
            'message': self.message,
            'mount': self.mount,
            'corners': list(CORNERS),
            'deviations': self.devs,
            'accepted': list(self.accepted),
            'target': (CORNERS[self.target] if self.target is not None
                       else None),
            'action': self.action,
            'load_cell_corner_changed': self.load_cell_corner_changed,
        }

    # --- prompts ---------------------------------------------------------
    def _prompt(self, lines, buttons, cancel=True):
        self.gcode.respond_raw("// action:prompt_end")
        self.gcode.respond_raw("// action:prompt_begin %s" % (TITLE,))
        for line in lines:
            self.gcode.respond_raw("// action:prompt_text %s" % (line,))
        if cancel:
            buttons = list(buttons) + [("Cancel", CANCEL)]
        for label, command in buttons:
            self.gcode.respond_raw("// action:prompt_button %s|%s"
                                   % (label, command))
        self.gcode.respond_raw("// action:prompt_show")

    def _corner_lines(self):
        lines = []
        for i, name in enumerate(CORNERS):
            if self.devs is None:
                break
            if self.accepted[i]:
                status = "OK"
            elif i == self.target:
                status = "adjust now"
            else:
                status = "later"
            lines.append("%s: %.2f mm - %s" % (name.capitalize(),
                                                self.devs[i], status))
        return lines

    def _show(self):
        if self.state == AWAITING_MOUNT:
            self._prompt(
                ["How is the bed mounted?",
                 "Rigid spacers: the stock KE (no adjustment knobs).",
                 "Rubber or springs: a common upgrade with screws you turn."],
                [("Rigid spacers", CMD + " MOUNT=rigid"),
                 ("Rubber or springs", CMD + " MOUNT=adjustable")])
        elif self.state == AWAITING_ADJUSTMENT:
            lines = ["Front is the side with the screen."]
            lines += self._corner_lines()
            lines += [""] + self.action_lines()
            self._prompt(lines, [("Done - measure again",
                                  CMD + " CONFIRM=1")])

    def action_lines(self):
        name = CORNERS[self.target].capitalize()
        if self.mount == RIGID:
            lines = ["%s: add a %.2f mm shim." % (name, self.action['shim']),
                     "Unscrew this corner's bed screw, put the printed shim "
                     "on the spacer, and screw it back firmly.",
                     "Do not touch the other corners."]
        else:
            lines = ["%s: turn the screw %s %s." % (
                         name, self.action['words'],
                         "clockwise" if self.action['sign'] == 'CW'
                         else "counter-clockwise"),
                     "Do not touch the other corners."]
        if self.target == LOAD_CELL_CORNER:
            lines.append("This corner carries the load cell: it must be "
                         "calibrated again at the end.")
        return lines

    # --- commands ----------------------------------------------------------
    cmd_tram_help = ("Guided bed tramming: measures the four corners and says "
                     "what to adjust, one corner at a time. MOUNT=rigid|"
                     "adjustable sets the bed mount; CONFIRM=1 after an "
                     "adjustment measures again")

    def cmd_tram(self, gcmd):
        mount = gcmd.get('MOUNT', None)
        if mount is not None:
            return self._set_mount(mount.lower(), gcmd)
        if gcmd.get_int('CONFIRM', 0):
            return self._confirm()
        self._start()

    def _refuse_if_printing(self):
        print_stats = self.printer.lookup_object('print_stats', None)
        if print_stats is None:
            return
        state = print_stats.get_status(self.reactor.monotonic()).get('state')
        if state in ('printing', 'paused'):
            raise self.printer.command_error(
                "%s: not possible while a print is %s" % (CMD, state))

    def _start(self):
        self._refuse_if_printing()
        if self.state in ACTIVE:
            return self._show()             # second interface / reconnect
        self.run_id += 1
        self._reset(AWAITING_MOUNT)
        if self.mount is None:
            return self._show()
        self._measure_and_decide()

    def _set_mount(self, mount, gcmd):
        if mount not in MOUNTS:
            raise gcmd.error("%s: MOUNT must be rigid or adjustable" % (CMD,))
        self.mount = mount
        self._save_mount(mount)
        if self.state == AWAITING_MOUNT:
            self._measure_and_decide()
        else:
            gcmd.respond_info("Bed mount set to %s." % (mount,))

    def _confirm(self):
        if self.state != AWAITING_ADJUSTMENT:
            raise self.printer.command_error(
                "%s CONFIRM=1: no adjustment is waiting (state=%s)"
                % (CMD, self.state))
        if self.target == LOAD_CELL_CORNER:
            self.load_cell_corner_changed = True
        self._measure_and_decide()

    def _measure(self):
        toolhead = self.printer.lookup_object('toolhead')
        homed = toolhead.get_status(self.reactor.monotonic()).get(
            'homed_axes', '')
        if not all(a in homed for a in 'xyz'):
            self.gcode.run_script_from_command("G28")
        self.gcode.run_script_from_command(
            "SCREWS_TILT_CALCULATE SAMPLES=%d SAMPLES_RESULT=median"
            % (self.samples,))
        tilt = self.printer.lookup_object('screws_tilt_adjust')
        results = tilt.get_status(self.reactor.monotonic()).get('results', {})
        corners = [results.get('screw%d' % (i + 1,)) for i in range(4)]
        if any(c is None for c in corners):
            raise self.printer.command_error(
                "%s: the measurement did not return all four corners" % (CMD,))
        return corners

    def _measure_and_decide(self):
        self._refuse_if_printing()
        try:
            corners = self._measure()
        except Exception as e:
            self.state = ERROR
            self.error = str(e)
            self.gcode.respond_raw("// action:prompt_end")
            raise
        self.zs = [c['z'] for c in corners]
        self.devs = deviations(self.zs, self.mount)
        previous = self.target
        if previous is not None:
            before = self.target_before
            if self.devs[previous] > before - self.min_improvement:
                self.tries[previous] += 1
            else:
                self.tries[previous] = 0
        for i, d in enumerate(self.devs):
            if d <= self.tolerance:
                self.accepted[i] = True
        open_corners = [i for i in range(4) if self.devs[i] > self.tolerance]
        if not open_corners:
            return self._complete()
        stuck = [i for i in open_corners if self.tries[i] >= self.max_tries]
        if stuck:
            return self._stop(
                "%s did not improve after %d adjustments. Check that the "
                "screw and spacer at this corner are tight and nothing is "
                "bent or loose (bed, gantry), then start again."
                % (CORNERS[stuck[0]].capitalize(), self.max_tries))
        candidates = [i for i in open_corners if not self.accepted[i]]
        if not candidates:
            names = ", ".join(CORNERS[i] for i in open_corners)
            return self._stop(
                "The %s corner was OK before but moved after another "
                "adjustment. Check that every bed screw is tight, then "
                "start again." % (names,))
        target = max(candidates, key=lambda i: self.devs[i])
        if self.mount == RIGID:
            shim = shim_mm(self.devs[target], self.shim_step, self.max_shim)
            self.action = {'shim': shim}
        else:
            result = corners[target]
            self.action = {'sign': result['sign'], 'adjust': result['adjust'],
                           'words': turn_words(result['adjust'])}
        self.target = target
        self.target_before = self.devs[target]
        self.state = AWAITING_ADJUSTMENT
        self.message = " ".join(self.action_lines())
        self._show()

    def _complete(self):
        self.state = COMPLETE
        self.target = None
        self.action = None
        lines = ["Bed tramming complete. No further screw or spacer "
                 "adjustments needed."] + self._corner_lines()
        buttons = []
        if self.load_cell_corner_changed:
            lines.append("The front-left corner (load cell) was changed: "
                         "calibrate the load cell now.")
            buttons.append(("Calibrate load cell",
                            "NEBULAOS_LOAD_CELL_CALIBRATE"))
        else:
            lines.append("Next: measure the Z offset and a fresh bed mesh.")
            buttons.append(("Z offset & bed mesh", "NEBULAOS_AUTO_CALIBRATE"))
        buttons.append(("Close", "RESPOND TYPE=command MSG=action:prompt_end"))
        self.message = lines[0]
        self._prompt(lines, buttons, cancel=False)

    def _stop(self, why):
        self.state = STOPPED
        self.message = why
        self._prompt(["Stopped: " + why] + self._corner_lines(),
                     [("Close", "RESPOND TYPE=command MSG=action:prompt_end")],
                     cancel=False)

    def cmd_cancel(self, gcmd):
        if self.state not in ACTIVE:
            raise gcmd.error("No bed tramming is in progress")
        self._reset(CANCELLED)
        self.gcode.respond_raw("// action:prompt_end")
        gcmd.respond_info("Bed tramming cancelled.")


def load_config(config):
    return NebulaOSBedTram(config)
