# NebulaOS TMC autotune selection (decision D8, 2026-10-10)
#
# GuppyScreen's TMC Autotune panel used _GUPPY_SAVE_CONFIG / _GUPPY_DELETE_CONFIG
# (a shell helper that edited printer.cfg), which NebulaOS does not ship. This
# module stages the same [autotune_tmc <stepper>] sections through Klipper's own
# configfile instead; the caller follows with SAVE_CONFIG, which writes them into
# printer.cfg's autosave block and restarts Klipper. The autotune itself is the
# vendored upstream autotune_tmc.py, loaded by Klipper only once such a section
# exists - with none (the default) nothing about the drivers changes.
#
#   NEBULAOS_TMC_AUTOTUNE STEPPER=stepper_x MOTOR=creality-42-40 GOAL=performance
#   NEBULAOS_TMC_AUTOTUNE STEPPER=stepper_x REMOVE=1
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import os
import re

TRINAMIC_DRIVERS = ("tmc2130", "tmc2208", "tmc2209", "tmc2240", "tmc2660",
                    "tmc5160")
GOALS = ("auto", "silent", "performance", "autoswitch")
STEPPER_RE = re.compile(r"^[a-z0-9_]+$")
MOTOR_DATABASE = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                              "motor_database.cfg")


def motor_names(path=MOTOR_DATABASE):
    """[motor_constants NAME] and [motor_alias NAME] names in the database."""
    names = set()
    try:
        with open(path, "r") as f:
            for line in f:
                line = line.strip()
                for prefix in ("[motor_constants ", "[motor_alias "):
                    if line.startswith(prefix) and line.endswith("]"):
                        names.add(line[len(prefix):-1].strip())
    except (IOError, OSError):
        pass
    return names


def validate_selection(stepper, motor, goal, tmc_sections, motors):
    """Returns an error message, or "" when the selection is valid. Pure."""
    if not STEPPER_RE.match(stepper or ""):
        return "STEPPER must name one stepper (e.g. stepper_x)"
    if not any(("%s %s" % (d, stepper)) in tmc_sections for d in TRINAMIC_DRIVERS):
        return "%s has no TMC driver section" % stepper
    if motor not in motors:
        return "unknown motor '%s' (not in motor_database.cfg)" % motor
    if goal not in GOALS:
        return "GOAL must be one of %s" % ", ".join(GOALS)
    return ""


class NebulaOSTMCAutotune:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object("gcode")
        self.gcode.register_command(
            "NEBULAOS_TMC_AUTOTUNE", self.cmd_NEBULAOS_TMC_AUTOTUNE,
            desc="Stage a TMC autotune motor/goal for a stepper (then SAVE_CONFIG)")

    def _tmc_sections(self):
        configfile = self.printer.lookup_object("configfile")
        status = configfile.get_status(None)
        return set((status.get("settings") or {}).keys()) | set(
            (status.get("config") or {}).keys())

    def cmd_NEBULAOS_TMC_AUTOTUNE(self, gcmd):
        stepper = gcmd.get("STEPPER", "").strip().lower()
        configfile = self.printer.lookup_object("configfile")
        section = "autotune_tmc %s" % stepper
        if gcmd.get_int("REMOVE", 0, minval=0, maxval=1):
            if not STEPPER_RE.match(stepper):
                raise gcmd.error("STEPPER must name one stepper")
            configfile.remove_section(section)
            gcmd.respond_info("TMC autotune for %s removed - run SAVE_CONFIG "
                              "to apply" % stepper)
            return
        motor = gcmd.get("MOTOR", "").strip()
        goal = gcmd.get("GOAL", "auto").strip().lower()
        err = validate_selection(stepper, motor, goal, self._tmc_sections(),
                                 motor_names())
        if err:
            raise gcmd.error("NEBULAOS_TMC_AUTOTUNE: %s" % err)
        configfile.set(section, "motor", motor)
        configfile.set(section, "tuning_goal", goal)
        gcmd.respond_info("TMC autotune for %s: motor %s, goal %s - run "
                          "SAVE_CONFIG to apply" % (stepper, motor, goal))


def load_config(config):
    return NebulaOSTMCAutotune(config)
