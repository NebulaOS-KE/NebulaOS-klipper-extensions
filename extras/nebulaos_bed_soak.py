# NebulaOS bed warp soak switch, remembered (Run 21, Tim 2026-10-28)
#
# [output_pin Bed_Warp_Stabilisation] (firmware print.cfg) is the Mainsail
# switch START_PRINT reads when the slicer passes no BED_SOAK: on = soak the
# bed before every print, off = never (BED_SOAK=1/0 from the slicer still
# wins for that one print). Upstream output_pin forgets its value at every
# Klipper start; this module keeps it in one small JSON file:
#
#   - SET_PIN is wrapped (Mainsail and the screen switch the pin with it):
#     after the upstream command ran, a change of THIS pin is saved;
#   - at klippy:ready a saved "on" is applied again with SET_PIN.
#
# Same rules as nebulaos_z_adjust: the file is written atomically (tmp in the
# same directory + fsync + rename + directory fsync), never with Klipper's
# save_variables (which rewrites in place and refuses to start Klipper on a
# half-written file); an unreadable file is "off" plus a console warning,
# never a config error - Klipper must always start. A failed save leaves the
# switch set live and says so; it never fails the SET_PIN.
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import json
import logging
import os

from . import nebulaos_calibration_journal as journal

KEY = 'enabled'
DEFAULT_PIN = 'Bed_Warp_Stabilisation'
DEFAULT_PATH = journal.DEFAULT_JOURNAL_DIR + "/bed_soak.json"


def read_enabled(path):
    """(enabled, warning). A missing file is off without a warning; anything
    unreadable or malformed is off WITH a warning. Never raises."""
    try:
        with open(path) as handle:
            data = json.load(handle)
    except (IOError, OSError) as e:
        if not os.path.exists(path):
            return False, ''
        return False, "cannot read %s (%s)" % (path, e)
    except Exception as e:
        return False, "%s is not valid JSON (%s)" % (path, e.__class__.__name__)
    value = data.get(KEY) if isinstance(data, dict) else None
    if not isinstance(value, bool):
        return False, "%s has no true/false '%s'" % (path, KEY)
    return value, ''


def write_enabled(path, enabled):
    """Atomic: tmp in the same directory + fsync + os.replace + dir fsync."""
    directory = os.path.dirname(path)
    if directory:
        os.makedirs(directory, exist_ok=True)
    journal.atomic_write_json(path + ".tmp", path, {KEY: bool(enabled)})


class NebulaOSBedSoak:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.pin = config.get('pin', DEFAULT_PIN)
        self.path = config.get('filename', DEFAULT_PATH)
        self.enabled, self.error = read_enabled(self.path)
        if self.error:
            logging.warning("nebulaos_bed_soak: %s - using off", self.error)
        self._prev_set_pin = None
        self.printer.register_event_handler("klippy:ready", self._handle_ready)

    def get_status(self, eventtime):
        return {'enabled': self.enabled, 'error': self.error}

    def _handle_ready(self):
        self._wrap_set_pin()
        self.reactor.register_callback(self._apply_at_ready)

    def _wrap_set_pin(self):
        if self._prev_set_pin is not None:
            return
        try:
            prev = self.gcode.register_command('SET_PIN', None)
        except Exception:
            logging.exception("nebulaos_bed_soak: SET_PIN lookup")
            return
        if prev is None:
            return
        self._prev_set_pin = prev
        self.gcode.register_command(
            'SET_PIN', self._cmd_set_pin,
            desc="Set the value of an output pin (the bed warp soak switch "
                 "is remembered across Klipper restarts)")

    def _cmd_set_pin(self, gcmd):
        self._prev_set_pin(gcmd)
        if gcmd.get('PIN', '') != self.pin:
            return
        enabled = gcmd.get_float('VALUE', 0.) >= 0.5
        if enabled == self.enabled and not self.error:
            return
        try:
            write_enabled(self.path, enabled)
        except (IOError, OSError) as e:
            self.error = "could not save %s: %s" % (self.path, e)
            logging.exception("nebulaos_bed_soak: save failed")
            gcmd.respond_info("!! Bed warp soak switched %s but NOT saved - %s"
                              % ("on" if enabled else "off", self.error))
            return
        self.enabled = enabled
        self.error = ''
        logging.info("nebulaos_bed_soak: switch %s (saved)",
                     "on" if enabled else "off")

    def _apply_at_ready(self, eventtime):
        if self.error:
            self.gcode.respond_info(
                "!! Bed warp soak: %s - switch is off" % (self.error,))
        if not self.enabled:
            return
        try:
            self.gcode.run_script("SET_PIN PIN=%s VALUE=1" % (self.pin,))
        except Exception as e:
            self.error = "could not switch it on again: %s" % (e,)
            logging.exception("nebulaos_bed_soak: apply at ready failed")
            self.gcode.respond_info("!! Bed warp soak: %s" % (self.error,))


def load_config(config):
    return NebulaOSBedSoak(config)
