# NebulaOS Z offset adjustment, saved like OpenKE did (G92)
#
# The screen's Z offset buttons adjust the gcode Z offset (homing_origin.z,
# "babystepping"). Upstream keeps that in memory only, so it was lost on every
# Klipper restart. This module keeps ONE number - the adjusted total - in a
# small JSON file and applies it again when Klipper is ready:
#
#   NEBULAOS_Z_ADJUST DELTA=<mm>  adjust live (SET_GCODE_OFFSET Z_ADJUST=
#                                 MOVE=1, Z must be homed), save the new total
#   NEBULAOS_Z_ADJUST RESET=1     back to 0 (MOVE=1 if Z is homed), save 0
#   NEBULAOS_Z_ADJUST APPLY=1     apply the saved value again (absolute). The
#                                 firmware RESUME macro calls this after the
#                                 upstream RESUME: upstream restores the
#                                 gcode state saved at PAUSE, which would undo
#                                 an adjustment made while paused (C4).
#   NEBULAOS_Z_ADJUST             report the saved value
#
# What OpenKE did (save-zoffset.cfg: save_variables + a loader macro), with
# two differences (Tim 2026-10-21): the file is written atomically (tmp in
# the same directory + fsync + rename + directory fsync), never with
# Klipper's save_variables (which rewrites in place and refuses to start
# Klipper on a half-written file); and an unreadable file is 0 plus a
# console warning, never a config error - Klipper must always start.
#
# The value is reset to 0 when the load-cell Z-offset calibration saves a
# new probe z_offset (nebulaos_calibration.py calls reset_for_calibration()),
# because the adjustment was made relative to the old z_offset.
#
# Only the adjustment made with this command is saved. SET_GCODE_OFFSET is
# NOT wrapped: offsets set by macros, PLR or other extras are never saved.
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import json
import logging
import math
import os

from . import nebulaos_calibration_journal as journal

CMD = "NEBULAOS_Z_ADJUST"
KEY = 'z_adjust_mm'
# Top level of Moonraker's config root: visible (and editable) in Mainsail,
# user-owned, not inside config/nebulaos/ (image-materialized).
DEFAULT_PATH = "/opt/printer_data/config/z_offset_adjust.json"
# Live offset and saved value agree within this (float noise only).
_MATCH = 1e-5


def read_value(path, max_adjust):
    """(value_mm, warning). A missing file is 0 without a warning; anything
    unreadable, malformed, non-finite or out of range is 0 WITH a warning.
    Never raises: the file is user-editable in Mainsail and Klipper must
    always start (even a pathological edit - huge numbers, deep nesting)."""
    try:
        return _read_value(path, max_adjust)
    except Exception as e:
        return 0., "%s could not be used (%s)" % (path, e.__class__.__name__)


def _read_value(path, max_adjust):
    try:
        with open(path) as handle:
            data = json.load(handle)
    except (IOError, OSError) as e:
        if not os.path.exists(path):
            return 0., ''
        return 0., "cannot read %s (%s)" % (path, e)
    except ValueError as e:
        return 0., "%s is not valid JSON (%s)" % (path, e)
    try:
        value = float(data[KEY])
    except (TypeError, KeyError, ValueError):
        return 0., "%s has no number '%s'" % (path, KEY)
    if not math.isfinite(value):
        return 0., "%s: '%s' is not a finite number" % (path, KEY)
    if abs(value) > max_adjust:
        return 0., ("%s: %.3f mm is outside +/-%.2f mm"
                    % (path, value, max_adjust))
    return value, ''


def write_value(path, value):
    """Atomic: tmp in the same directory + fsync + os.replace + dir fsync."""
    directory = os.path.dirname(path)
    if directory:
        os.makedirs(directory, exist_ok=True)
    journal.atomic_write_json(path + ".tmp", path, {KEY: round(value, 6)})


class NebulaOSZAdjust:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.path = config.get('filename', DEFAULT_PATH)
        # A saved or requested total beyond this is refused (a damaged or
        # hand-edited file must never drive the nozzle into the bed).
        self.max_adjust = config.getfloat('max_adjust', 2.0, above=0.,
                                          maxval=5.)
        self.value, self.error = read_value(self.path, self.max_adjust)
        if self.error:
            logging.warning("nebulaos_z_adjust: %s - using 0", self.error)
        self.gcode_move = None
        # Saved value when the print was paused (MARK=1), None otherwise.
        self.pause_mark = None
        self._prev_apply_probe = None
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        self.gcode.register_command(CMD, self.cmd_z_adjust,
                                    desc=self.cmd_z_adjust_help)

    # --- status ------------------------------------------------------------
    def _live_z(self, eventtime=None):
        if self.gcode_move is None:
            return None
        if eventtime is None:
            eventtime = self.reactor.monotonic()
        return self.gcode_move.get_status(eventtime)['homing_origin'].z

    def get_status(self, eventtime):
        live = self._live_z(eventtime)
        return {
            'value': self.value,
            # True while the live gcode Z offset equals the saved value.
            'applied': live is not None and abs(live - self.value) < _MATCH,
            'error': self.error,
        }

    # --- startup -----------------------------------------------------------
    def _handle_ready(self):
        self.gcode_move = self.printer.lookup_object('gcode_move')
        self._wrap_apply_probe()
        self.reactor.register_callback(self._apply_at_ready)

    def _wrap_apply_probe(self):
        """C6: upstream Z_OFFSET_APPLY_PROBE (Mainsail's Z offset "Save")
        folds the live gcode offset - including this adjustment - into the
        probe z_offset for SAVE_CONFIG. The adjustment is then part of the
        z_offset, so the saved one becomes 0 (else it is applied twice)."""
        if self._prev_apply_probe is not None:
            return
        try:
            prev = self.gcode.register_command('Z_OFFSET_APPLY_PROBE', None)
        except Exception:
            logging.exception("nebulaos_z_adjust: Z_OFFSET_APPLY_PROBE lookup")
            return
        if prev is None:
            return
        self._prev_apply_probe = prev
        self.gcode.register_command(
            'Z_OFFSET_APPLY_PROBE', self._cmd_apply_probe,
            desc="Adjust the probe's z_offset (the saved NebulaOS Z offset "
                 "adjustment is folded in and set to 0)")

    def _cmd_apply_probe(self, gcmd):
        self._prev_apply_probe(gcmd)
        if self.value != 0.:
            self._save(0.)
            gcmd.respond_info(
                "%s: the Z offset adjustment is now part of the probe "
                "z_offset - saved adjustment set to 0 (run SAVE_CONFIG to "
                "keep it)" % (CMD,))

    def _apply_at_ready(self, eventtime):
        if self.error:
            self.gcode.respond_info(
                "!! %s: %s - Z offset adjustment set to 0" % (CMD, self.error))
        if self.value == 0.:
            return
        try:
            self.gcode.run_script("SET_GCODE_OFFSET Z=%.6f MOVE=0"
                                  % (self.value,))
        except Exception as e:
            self.error = "could not apply the saved value: %s" % (e,)
            logging.exception("nebulaos_z_adjust: apply at ready failed")
            self.gcode.respond_info("!! %s: %s" % (CMD, self.error))
            return
        self.gcode.respond_info("%s: Z offset adjustment %+.3f mm applied"
                                % (CMD, self.value))

    # --- helpers -----------------------------------------------------------
    def _z_homed(self):
        toolhead = self.printer.lookup_object('toolhead')
        homed = toolhead.get_status(self.reactor.monotonic()).get(
            'homed_axes', '')
        return 'z' in homed

    def _save(self, value):
        try:
            write_value(self.path, value)
        except (IOError, OSError) as e:
            self.error = "could not save %s: %s" % (self.path, e)
            raise self.printer.command_error(
                "%s: applied live but NOT saved - %s" % (CMD, self.error))
        self.value = value
        self.error = ''

    # --- API for other NebulaOS modules -----------------------------------
    def reset_for_calibration(self):
        """A new probe z_offset is about to be saved: the adjustment was
        relative to the old one. File and live offset to 0 (MOVE=0: the
        calibration owns the toolhead). Called from inside a gcode command;
        raises if the file cannot be written, so the caller stops before its
        SAVE_CONFIG instead of keeping a stale adjustment."""
        self.gcode.run_script_from_command("SET_GCODE_OFFSET Z=0 MOVE=0")
        self._save(0.)
        self.gcode.respond_info(
            "%s: Z offset adjustment reset to 0 (new Z offset calibration)"
            % (CMD,))

    def apply_saved(self, move=False):
        """Live gcode Z offset = the saved value (absolute). Inside a gcode
        command only."""
        self.gcode.run_script_from_command(
            "SET_GCODE_OFFSET Z=%.6f MOVE=%d" % (self.value, 1 if move else 0))

    # --- command -----------------------------------------------------------
    cmd_z_adjust_help = (
        "Z offset adjustment, saved and applied at every start: DELTA=<mm> "
        "adjusts (Z homed), RESET=1 sets 0, APPLY=1 applies the saved value "
        "again, MARK=1 / RESUME=1 for PAUSE / RESUME, no parameter reports "
        "it")

    def cmd_z_adjust(self, gcmd):
        delta = gcmd.get_float('DELTA', None)
        if delta is not None:
            return self._cmd_delta(gcmd, delta)
        if gcmd.get_int('RESET', 0):
            return self._cmd_reset(gcmd)
        if gcmd.get_int('MARK', 0):
            return self._cmd_mark(gcmd)
        if gcmd.get_int('RESUME', 0):
            return self._cmd_resume(gcmd)
        if gcmd.get_int('APPLY', 0):
            return self._cmd_apply(gcmd)
        live = self._live_z()
        gcmd.respond_info(
            "Z offset adjustment: %+.3f mm, saved%s"
            % (self.value, "" if live is None or abs(live - self.value) < _MATCH
               else " (live offset now %+.3f mm)" % (live,)))

    def _cmd_delta(self, gcmd, delta):
        if not math.isfinite(delta):
            raise gcmd.error("%s: DELTA must be a number" % (CMD,))
        if not self._z_homed():
            raise gcmd.error(
                "%s: home Z first (G28) - the adjustment moves the nozzle"
                % (CMD,))
        new_total = self._live_z() + delta
        if abs(new_total) > self.max_adjust:
            raise gcmd.error(
                "%s: %+.3f mm would be outside +/-%.2f mm - not changed"
                % (CMD, new_total, self.max_adjust))
        self.gcode.run_script_from_command(
            "SET_GCODE_OFFSET Z_ADJUST=%.6f MOVE=1" % (delta,))
        self._save(self._live_z())
        gcmd.respond_info("Z offset adjustment: %+.3f mm, saved"
                          % (self.value,))

    def _cmd_reset(self, gcmd):
        move = 1 if self._z_homed() else 0
        self.gcode.run_script_from_command(
            "SET_GCODE_OFFSET Z=0 MOVE=%d" % (move,))
        self._save(0.)
        gcmd.respond_info("Z offset adjustment: +0.000 mm, saved")

    def _cmd_apply(self, gcmd):
        homed = self._z_homed()
        move = gcmd.get_int('MOVE', 1 if homed else 0, minval=0, maxval=1)
        if move and not homed:
            raise gcmd.error("%s APPLY=1 MOVE=1: Z is not homed" % (CMD,))
        self.apply_saved(move=bool(move))
        gcmd.respond_info("Z offset adjustment %+.3f mm applied"
                          % (self.value,))


    # --- pause / resume (C4) -----------------------------------------------
    def _cmd_mark(self, gcmd):
        """PAUSE: remember the saved value. RESUME restores the gcode state
        saved at PAUSE (upstream) - only the screen adjustments made while
        paused are then added again, so offsets from Mainsail or the print
        file are never overwritten."""
        self.pause_mark = self.value

    def _cmd_resume(self, gcmd):
        mark, self.pause_mark = self.pause_mark, None
        if mark is None:
            return
        delta = self.value - mark
        if abs(delta) < _MATCH:
            return
        move = 1 if self._z_homed() else 0
        self.gcode.run_script_from_command(
            "SET_GCODE_OFFSET Z_ADJUST=%.6f MOVE=%d" % (delta, move))
        gcmd.respond_info("%s: %+.3f mm adjusted while paused - applied"
                          % (CMD, delta))


def load_config(config):
    return NebulaOSZAdjust(config)
