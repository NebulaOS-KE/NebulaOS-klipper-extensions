# NebulaOS calibration coordinator (Phase 2 calibration-framework mission;
# contact-safety stabilization rewrite, corrected).
#
# The single [nebulaos_calibration] object that owns the canonical
# NEBULAOS_* public calibration API. Implemented: standalone
# NEBULAOS_Z_OFFSET_CALIBRATE (LOAD_CELL only, bounded-descent envelope +
# measurement-quality/repeatability gating, plus its SIMULATE_BOOTSTRAP=1
# qualification aid - mission §8); NEBULAOS_AUTO_CALIBRATE, the
# geometry-only preflight->home->nozzle-clean->Z-offset->bed-mesh->
# one-SAVE_CONFIG-restart-and-verify sequence (mission §11), backed by
# nebulaos_calibration_journal.py's persistent transaction journal (mission
# §12) and NEBULAOS_CALIBRATION_CANCEL. Automatic Axis Twist is a final
# product decision to NOT support (see "Axis Twist" section below) -
# NEBULAOS_AXIS_TWIST_CALIBRATE has been removed entirely, not merely
# blocked. The guided Input Shaper/E-Steps workflows (separate commands,
# not part of NEBULAOS_AUTO_CALIBRATE - see the Phase 2 mission
# document's own §13/§14) are NOT part of this slice.
#
# ---------------------------------------------------------------------
# Upstream-first cleanup (Overnight Contact-Safety Stabilization mission)
# ---------------------------------------------------------------------
# Manual Axis Twist and manual Z-offset are NO LONGER wrapped here at all.
# There is exactly one way to run each: call pristine upstream's own
# AXIS_TWIST_COMPENSATION_CALIBRATE AXIS=X|Y or PROBE_CALIBRATE directly
# (both their own interactive TESTZ/ACCEPT/ABORT workflows) - this module
# used to offer thin METHOD=MANUAL passthroughs for both, which added a
# second name for the exact same upstream command with no product value,
# and both have been removed. klippy/extras/axis_twist_compensation.py and
# probe.py (58bd67db..., NOT modified, NOT shadowed, NOT monkeypatched -
# see docs/NEBULAOS_PRISTINE_KLIPPER.md) remain fully authoritative,
# exactly as before.
#
# The PID-default-target and bed-mesh-named-profile Python wrappers
# (NEBULAOS_PID_CALIBRATE_BED/_HOTEND, NEBULAOS_BED_MESH_CALIBRATE) are
# also removed. The bed-mesh one was a real duplication, not just a
# convenience: pinned upstream bed_mesh.py's own cmd_BED_MESH_CALIBRATE
# already reads `gcmd.get('PROFILE', "default")` and saves under it
# directly (confirmed against the pinned source, not assumed) - so
# `BED_MESH_CALIBRATE PROFILE=<name>` alone is the exact upstream
# equivalent of what this module's own wrapper did in two commands. PID's
# default-target convenience has no upstream equivalent (TARGET is
# required, no default), but that is a firmware-config concern, not a
# reason to keep Python in this repository - see NebulaOS-firmware's own
# gcode_macro convenience wrappers instead.
#
# ---------------------------------------------------------------------
# Axis Twist: automatic (LOAD_CELL) path REMOVED - product decision
# ---------------------------------------------------------------------
# Phase 2 mission, final product decision (2026-09): automatic (remote
# load-cell contact) Axis Twist is UNSUPPORTED on the Ender-3 V3 KE and
# will not ship. NEBULAOS_AXIS_TWIST_CALIBRATE, axis_twist_bed_points(),
# axis_twist_geometry_preflight(), and every axis_twist_* status field
# that existed only to support that abandoned command have been removed
# entirely (they previously existed hard-blocked, then dead/unwired,
# pending a hardware qualification that was cancelled rather than
# completed - see the real safety incident that originally motivated the
# hard block, _evidence/overnight-hx711-investigation-20260831-233518/
# REPORT.md, and the Aug 31 qualification evidence this decision
# supersedes, _evidence/phase2-axis-twist-qualification-20260831-203913/).
#
# Manual Axis Twist is NOT affected and needs no wrapper here: call
# pristine upstream AXIS_TWIST_COMPENSATION_CALIBRATE (AXIS=X or AXIS=Y)
# directly - see klippy/extras/axis_twist_compensation.py (58bd67db...,
# NOT modified, NOT shadowed). Its own [axis_twist_compensation] config
# section and calibrate_start/end_x/y geometry remain untouched.
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import math
import os
import re
import shutil
import subprocess
import sys
import time

from . import nebulaos_probe_pair
from . import nebulaos_raw_contact as raw_contact
from . import nebulaos_calibration_journal as calibration_journal
from . import nebulaos_calibration_history as calibration_history

_MAX_ERROR_LEN = 200


# G91: states in which a guided flow is running, committing (restart
# pending) or waiting for the user - get_status()['busy'].
BUSY_STATES = frozenset((
    'running', 'commit_requested', 'restart_pending', 'verification_pending',
    'awaiting_continue', 'awaiting_measurement', 'awaiting_confirmation',
    'awaiting_x_mount', 'awaiting_y_mount'))

# G104: the user-facing steps of each guided flow - get_status()['progress']
# = {"flow", "step", "total", "key", "label", "detail"} (PLAN Run 19, fixed
# interface). (key, label, advice when this step fails).
_LOAD_CELL_ADVICE = ("Remove everything from the bed and try again, or use "
                     "Calibration > Load Cell > Manual.")
AUTO_CALIBRATE_STEPS = (
    ('preflight', "Load cell check", _LOAD_CELL_ADVICE),
    ('home', "Homing",
     "Check that nothing blocks the axes, then try again."),
    ('nozzle_clean', "Nozzle clean",
     "Check the wipe pad and that the nozzle is free of filament, then try "
     "again."),
    ('establish_thermal_state', "Heating up",
     "Check the heaters (Calibration > Heaters (PID)), then try again."),
    ('stabilize', "Settling", "Try again."),
    ('hot_load_cell_check', "Load cell check (hot)", _LOAD_CELL_ADVICE),
    ('localized_z_offset', "Measuring the Z offset",
     "Clean the nozzle and the bed at the front-left corner, then try "
     "again."),
    ('bed_mesh', "Bed mesh",
     "Check that the bed is clean and the probe (BLTouch) deploys, then try "
     "again."),
    ('commit', "Saving (Klipper restarts)",
     "Restart the printer and run Auto Calibration again."),
    ('post_restart_verification', "Verifying",
     "The saved values do not match what was measured - run Auto "
     "Calibration again."),
)
_AUTO_STAGE_STEP = {
    'preflight': 1, 'home': 2, 'nozzle_clean': 3,
    'establish_thermal_state': 4, 'stabilize': 5, 'localized_z_offset': 7,
    'bed_mesh': 8, 'final_validation': 9, 'commit': 9, 'restart': 9,
    'post_restart_verification': 10}
_SHAPER_ADVICE = ("Check that the accelerometer is mounted firmly and its "
                  "cable is connected, then try again.")
INPUT_SHAPER_STEPS = (
    ('home', "Homing", "Check that nothing blocks the axes, then try again."),
    ('mount_x', "Mount the sensor on the toolhead", ""),
    ('measure_x', "Measuring X", _SHAPER_ADVICE),
    ('mount_y', "Move the sensor to the bed", ""),
    ('measure_y', "Measuring Y", _SHAPER_ADVICE),
    ('commit', "Saving (Klipper restarts)",
     "Restart the printer and run Input Shaper again."),
    ('post_restart_verification', "Verifying",
     "The saved values do not match what was measured - run Input Shaper "
     "again."),
)
ESTEPS_STEPS = (
    ('heating', "Heating up",
     "Check the hotend heater, then try again."),
    ('mark_filament', "Mark the filament", ""),
    ('measure', "Measure and enter the length",
     "Check the measurement (in mm) and try again."),
    ('confirm', "Confirm the new value", ""),
    ('commit', "Saving (Klipper restarts)", ""),
)
_MAX_LINE = calibration_history.MAX_SUMMARY_LEN

# G109 (Run 20): what upstream SHAPER_CALIBRATE prints (resonance_tester.py /
# shaper_calibrate.py, pinned 58bd67db) - read through a gcode output
# handler, so the screen can say what is happening without any change to
# the upstream command.
_RE_TEST_FREQ = re.compile(r"Testing frequency ([0-9.]+) Hz")
_RE_CALCULATING = re.compile(
    r"Calculating the best input shaper parameters for (\w+) axis")
_RE_FITTED = re.compile(
    r"Fitted shaper '(\w+)' frequency = ([0-9.]+) Hz \(vibrations = "
    r"([0-9.]+)%, smoothing ~= ([0-9.]+)\)")
_RE_MAX_ACCEL = re.compile(
    r"To avoid too much smoothing with '(\w+)', suggested max_accel <= "
    r"([0-9.]+) mm/sec\^2")
_RE_CSV = re.compile(r"Shaper calibration data written to (\S+\.csv) file")
_GRAPH_SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             'nebulaos_shaper_graph.py')


def _sanitize_error(exc):
    """Same convention as z_compensate.py's own _sanitize_calibration_error
    - str() on a Klipper error never includes a traceback/repr, so this
    just collapses whitespace and bounds length for a status field."""
    text = str(exc).strip() or exc.__class__.__name__
    text = ' '.join(text.split())
    if len(text) > _MAX_ERROR_LEN:
        text = text[:_MAX_ERROR_LEN - 3].rstrip() + '...'
    return text


def mesh_summary(mesh_status):
    """'Bed mesh 9x9, range 0.16 mm' from the probed matrix (columns x
    rows, max - min of every probed point), or None."""
    matrix = (mesh_status or {}).get('probed_matrix') or []
    values = [v for row in matrix for v in (row or [])
              if isinstance(v, (int, float)) and math.isfinite(v)]
    if not values:
        return None
    return "Bed mesh %dx%d, range %.2f mm" % (
        len(matrix[0]), len(matrix), max(values) - min(values))


def auto_calibrate_summary(new_z, old_z, mesh_status, old_z_adjust=None):
    lines = []
    if old_z is not None and math.isfinite(old_z):
        lines.append("Z offset %.3f mm (was %.3f)" % (new_z, old_z))
    else:
        lines.append("Z offset %.3f mm" % (new_z,))
    mesh = mesh_summary(mesh_status)
    if mesh:
        lines.append(mesh)
    if old_z_adjust:
        lines.append("Z adjustment reset to 0 (was %+.3f mm)"
                     % (old_z_adjust,))
    return lines


def auto_calibrate_history_summary(new_z, mesh_status):
    """The one-line status of the Calibration menu row (G105)."""
    text = "Z offset %.3f mm" % (new_z,)
    matrix = (mesh_status or {}).get('probed_matrix') or []
    values = [v for row in matrix for v in (row or [])
              if isinstance(v, (int, float)) and math.isfinite(v)]
    if values:
        text += ", mesh range %.2f mm" % (max(values) - min(values),)
    return text


class NebulaOSCalibration:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.reactor = self.printer.get_reactor()

        self.down_min_z = config.getfloat('down_min_z', default=10., minval=1., maxval=50.)
        self.pro_cnt = config.getint('pro_cnt', default=3, minval=1)
        self.horizontal_move_z = config.getfloat(
            'horizontal_move_z', default=5., minval=1., maxval=50.)
        self.travel_speed = config.getfloat('travel_speed', default=200., above=0.)
        self.probe_lift_speed = config.getfloat('probe_lift_speed', default=20., above=0.)
        # Same conservative default and rationale as z_compensate.py's own
        # max_offset_correction_mm - a sanity ceiling, not a physical bound
        # (that is [heater_bed]/[extruder] max_temp's job for temperatures,
        # and here simply "this is a fine-tune, not a re-leveling").
        self.max_offset_correction_mm = config.getfloat(
            'max_offset_correction_mm', default=2., minval=0.1, maxval=10.)
        # reference_x/reference_y (Phase 2 contact-safety mission, §11):
        # the canonical physical bed point NEBULAOS_Z_OFFSET_CALIBRATE
        # LOAD_CELL measures at, absent an explicit X=/Y= override -
        # firmware config's job, not a hardcoded Python constant. Factory
        # candidate is the stock KE's own validated touch-reference region
        # (noz_pos_center: 20,25 in z_compensate.py/[z_compensate], the
        # same real, stock-precedented point the overnight investigation
        # confirmed is stock's sole validated single-strain-gauge contact
        # location - see _project/missions/
        # nebulaos-klipper-loadcell-architecture-history.md). Left unset
        # (None) here on purpose; _resolve_reference_xy() falls back to
        # [z_compensate]'s own home_x/home_y when neither this nor an
        # explicit X=/Y= is available, so an older printer.cfg without
        # these two options still works exactly as before.
        self.reference_x = config.getfloat('reference_x', default=None)
        self.reference_y = config.getfloat('reference_y', default=None)
        # Nozzle-contact safety envelope constants (corrected) - see
        # nebulaos_probe_pair.py's own header for the exact two formulas.
        # Deliberately no invented production defaults for either; exactly
        # one is required per run, depending on whether the CURRENT live
        # probe z_offset is a credible physical prior (ESTABLISHED) or not
        # (BOOTSTRAP/virgin, e.g. the real factory default of 0.000).
        #
        # established_contact_margin_mm: how much FURTHER than the
        # predicted nozzle-contact plane (raw_probe_trigger_z -
        # CURRENT probe z_offset) the nozzle may still be commanded to
        # travel. Deliberately does NOT also carry the probe's own Z
        # offset the way an earlier, incorrect version of this envelope
        # (anchored to the raw probe trigger Z directly) accidentally did
        # - on this printer's real captured hardware state that offset is
        # ~1.795mm, far larger than any sane margin, which is exactly the
        # bug this rename and re-derivation fixes.
        #
        # Phase 2 mission §7 qualification (2026-09-02): 1.0mm, qualified
        # from CR-Touch repeatability at a nearby point (~0.01mm, two
        # orders of magnitude tighter than this margin - see
        # _evidence/phase2-contact-safety-hwqual-20260901-182810/
        # 07-contact-parameter-proposal/07-proposal.txt for the original
        # derivation) and confirmed with zero established-envelope
        # incidents across 22 real ESTABLISHED touch_probe runs (66
        # individual contacts) spanning cold (20C ambient) and hot (bed
        # 65C/hotend 230C) sessions on 2026-09-01 and 2026-09-02 - see
        # _evidence/phase2-live-full-stack-closure-20260902-180602/
        # REPORT.md. No longer requires a qualification-only printer.cfg
        # override.
        self.established_contact_margin_mm = config.getfloat(
            'established_contact_margin_mm', default=1.0,
            minval=0.01, maxval=5.)
        # bootstrap_contact_envelope_mm: how far below the nozzle's own
        # actual, known, currently-measured starting Z (horizontal_move_z,
        # default 5.0mm) a BOOTSTRAP/virgin calibration run (no credible
        # prior) may blindly search. A SEPARATE value from down_min_z -
        # never silently reused as one.
        #
        # Phase 2 mission §8 qualification (2026-09-02): 8.0mm, derived
        # from this unit's own measured z_offset history (1.6-2.3mm across
        # cold/hot sessions this mission) plus real margin - a genuinely
        # virgin unit (probe_z_offset=0.000 at homing) needs to search
        # from horizontal_move_z=5.0mm down to roughly -(1.6 to 2.3)mm in
        # that frame, i.e. 6.6-7.3mm of real depth; 8.0mm leaves ~0.7-1.4mm
        # of margin beyond this unit's own worst observed case for
        # ordinary unit-to-unit assembly variance, while the resulting
        # commanded_floor_z (5.0-8.0=-3.0mm) still leaves 2mm of
        # independent margin before the absolute stepper floor
        # (position_min=-5.0mm on this printer).
        #
        # Physically qualified via NEBULAOS_Z_OFFSET_CALIBRATE
        # SIMULATE_BOOTSTRAP=1 (see that command's own docstring): 6 real
        # BOOTSTRAP-mode nozzle contacts on hardware that already has a
        # trusted ESTABLISHED calibration, WITHOUT ever reading, mutating,
        # or risking that real calibration - 5 at ENVELOPE=7.0mm, all
        # succeeding with results (1.014-1.161mm) closely agreeing with an
        # ESTABLISHED-mode measurement taken moments earlier in the same
        # session (1.014-2.06mm range that same session) - cross-
        # validating that the BOOTSTRAP formula/motion path measures the
        # same real physical quantity correctly. A 6th run at a
        # deliberately excessive ENVELOPE=12.0mm (commanded_floor_z=-7.0mm,
        # beyond position_min) completed identically and safely: real
        # contact triggered at ~1.16mm long before any floor was
        # approached, and touch_probe()'s own contact_floor=max(z_floor,
        # minimum_allowed_z) clamp means the ACTUAL floor a probing move
        # could ever reach is structurally bounded by the independent
        # absolute z_floor regardless of how generous
        # bootstrap_contact_envelope_mm is - confirming this constant
        # cannot itself cause a deeper-than-intended search even if
        # mis-qualified. Full evidence:
        # _evidence/phase2-live-full-stack-closure-20260902-180602/
        # 03-bootstrap-simulation-qualification/. Not yet tested with an
        # actual, deeper real virgin-unit contact point (this unit's own
        # current ESTABLISHED calibration only lets the simulation reach
        # its OWN, comparatively shallow, contact point) - the mechanism,
        # formula, and safety clamping are proven; the specific 8.0mm
        # number is a geometry-and-margin-based estimate for a genuinely
        # virgin unit, not itself hardware-proven at full depth.
        self.bootstrap_contact_envelope_mm = config.getfloat(
            'bootstrap_contact_envelope_mm', default=8.0,
            minval=0.5, maxval=30.)
        # Measurement-quality and repeatability acceptance bounds (§7/§9/
        # §10).
        #
        # Phase 2 mission §7 qualification (2026-09-02): all three
        # defaults below are qualified from the same 22-run/66-touch
        # cold+hot dataset referenced above. Observed maxima against each
        # bound (all comfortably passing, no violations in any run):
        #   |fit_delta|:            0.2586mm observed vs 0.3mm bound
        #                           (~14% headroom - the tightest of the
        #                           three; matches the original proposal's
        #                           intent of rejecting the real recorded
        #                           ~1.106mm historical incident with wide
        #                           margin while still accepting healthy
        #                           trigger-latency corrections)
        #   repeatability range:    0.0407mm observed vs 0.15mm bound
        #                           (~3.7x headroom)
        #   repeatability stddev:   0.01973mm observed vs 0.06mm bound
        #                           (~3x headroom)
        # min_accepted_samples=2 (of pro_cnt=3): every one of the 22 runs
        # in this dataset accepted all 3 samples; no evidence to justify
        # a different value. No longer requires a qualification-only
        # printer.cfg override for any of the four.
        self.max_abs_fit_delta = config.getfloat(
            'max_abs_fit_delta', default=0.3, minval=0.001, maxval=10.)
        self.min_accepted_samples = config.getint(
            'min_accepted_samples', default=2, minval=1)
        self.max_repeatability_range = config.getfloat(
            'max_repeatability_range', default=0.15, minval=0.001, maxval=10.)
        self.max_repeatability_stddev = config.getfloat(
            'max_repeatability_stddev', default=0.06, minval=0.001, maxval=10.)

        # NEBULAOS_AUTO_CALIBRATE (mission §11). pid_bed_target/
        # pid_hotend_target are the PID-tune targets only (PID_CALIBRATE's
        # own TARGET=, run early in the sequence, well before nozzle
        # clean) - defaults are the qualification thermal state this
        # mission's hot-reliability evidence was gathered at (bed 65C /
        # hotend 230C).
        #
        # z_offset_reference_temp (root-caused 2026-09-02/03, live A/B
        # test - see _evidence/phase2-live-full-stack-closure-
        # 20260902-180602/07-nozzle-contamination-test/): the LOCALIZED_
        # Z_OFFSET stage deliberately does NOT reheat to pid_hotend_target
        # (230C) - it measures at this lower, separate temperature
        # instead. Root cause: residual nozzle ooze/contamination
        # accumulating during a long hot dwell (PID-hotend tune + a hot
        # nozzle-clean cycle) was found to bias the load-cell contact
        # measurement low by ~0.6mm (a reproducible ~1.01-1.15mm cluster,
        # against a normal ~1.73-1.80mm cluster) - proven by measuring
        # immediately after a fresh nozzle clean, at 150C instead of
        # 230C: 3/3 clean runs landed back in the normal cluster
        # (1.733-1.742mm, range 0.0095mm) on the SAME unit, SAME
        # persisted prior, SAME contact algorithm - nothing else changed.
        # 150C default matches nozzle_clean's own hot_start_temp resting
        # temperature ([z_compensate] config), so this is normally a
        # no-op confirmation of a state nozzle_clean already reached, not
        # a second real heat cycle. The contact algorithm and safety
        # envelope (established_contact_margin_mm etc.) are UNCHANGED -
        # this fix is sequencing/thermal-reference only.
        #
        # thermal_soak_seconds is an EXPLICIT additional dwell after
        # M190/M109 already report the target reached (their own
        # convergence tolerance is looser than "fully thermally
        # settled") - a plain upstream G4 dwell, not a reimplemented wait
        # loop. bed_mesh_profile matches BED_MESH_CALIBRATE's own
        # upstream default (gcmd.get('PROFILE', "default")) so a plain
        # `BED_MESH_CALIBRATE` invoked by a user later, with no PROFILE=,
        # loads exactly what this orchestrator saved.
        self.pid_bed_target = config.getfloat(
            'pid_bed_target', default=65., minval=0., maxval=120.)
        self.pid_hotend_target = config.getfloat(
            'pid_hotend_target', default=230., minval=0., maxval=320.)
        self.z_offset_reference_temp = config.getfloat(
            'z_offset_reference_temp', default=150., minval=0., maxval=320.)
        self.thermal_soak_seconds = config.getfloat(
            'thermal_soak_seconds', default=15., minval=0., maxval=300.)
        self.bed_mesh_profile = config.get('bed_mesh_profile', default='default')
        # Overridable only for unit tests (a real printer always uses the
        # real default path under /usr/data) - see
        # nebulaos_calibration_journal.py's own header for why this journal
        # exists at all (crash-safety around the one SAVE_CONFIG+restart
        # boundary, not routine status - NEBULAOS_CALIBRATION_STATUS's
        # auto_calibrate_* fields are the normal way to poll progress).
        self.journal_path = config.get(
            'journal_path', default=calibration_journal.DEFAULT_JOURNAL_PATH)
        # G104/G105: history + last result, next to the journal (same
        # directory; a test journal_path keeps its history beside it).
        self.history_path = config.get(
            'history_path', default=os.path.join(
                os.path.dirname(self.journal_path) or '.',
                calibration_history.FILE_NAME))

        # NEBULAOS_ESTEPS_CALIBRATE (mission §14) - a separate, standalone
        # guided workflow (not part of NEBULAOS_AUTO_CALIBRATE). Formula
        # (upstream's own documented measure-and-trim relationship):
        # new_rotation_distance = old_rotation_distance * actual_extruded
        #                          / commanded_extrusion
        # esteps_max_correction_ratio bounds |actual/commanded - 1| - a
        # sanity ceiling against a mistyped measurement (e.g. cm instead
        # of mm), same rationale as max_offset_correction_mm elsewhere in
        # this module: a real e-steps correction is normally a few
        # percent, not a doubling.
        self.esteps_temp = config.getfloat(
            'esteps_temp', default=200., minval=0., maxval=320.)
        self.esteps_commanded_length_mm = config.getfloat(
            'esteps_commanded_length_mm', default=100., above=0.)
        self.esteps_extrude_speed_mm_s = config.getfloat(
            'esteps_extrude_speed_mm_s', default=5., above=0.)
        self.esteps_max_correction_ratio = config.getfloat(
            'esteps_max_correction_ratio', default=0.3, above=0.)

        # NEBULAOS_INPUT_SHAPER_CALIBRATE (mission §13) - a separate,
        # standalone guided workflow (not part of NEBULAOS_AUTO_CALIBRATE:
        # resonance testing is done cold and has nothing to do with the
        # nozzle/bed thermal state that workflow coordinates). A genuine
        # 3-call guided sequence (see cmd_input_shaper_calibrate's own
        # header comment) wrapping upstream G28 + `SHAPER_CALIBRATE
        # AXIS=X` + `SHAPER_CALIBRATE AXIS=Y` + one SAVE_CONFIG+restart +
        # post-restart verification, the same journal pattern as
        # cmd_auto_calibrate's own localized_z_offset/bed_mesh stages,
        # using its own separate journal file so an in-flight
        # NEBULAOS_AUTO_CALIBRATE journal is never at risk of being
        # overwritten by this unrelated workflow (or vice versa).
        #
        # Platform feasibility (2026-09-02/03 overnight source-prep): the
        # underlying platform - [mcu rpi]/[adxl345]/[resonance_tester]/
        # [input_shaper] in machine.cfg, klipper_mcu served by
        # S54nebulaos-host-mcu - is Phase 1.9A work already merged into
        # this branch's own history (confirmed: `git merge-base
        # --is-ancestor c86dc54 HEAD` on phase2/calibration-framework),
        # not a separate unmerged-branch prerequisite as an earlier
        # research pass in this mission incorrectly assumed. Live-
        # reconfirmed 2026-09-03 on the actual candidate: ACCELEROMETER_
        # QUERY and MEASURE_AXES_NOISE both return real, plausible values
        # (see this session's own evidence), and the operator confirmed
        # live that the accelerometer is a genuinely movable clip-on
        # module (not fixed, despite machine.cfg's own wiring comment
        # describing the electrical path, not the physical mounting) -
        # which is exactly why this is a real 3-call relocate-between-
        # axes guided workflow rather than one combined pass.
        self.input_shaper_journal_path = config.get(
            'input_shaper_journal_path', default=(
                calibration_journal.DEFAULT_JOURNAL_DIR
                + "/input_shaper_journal.json"))
        # shaper_freq_end / shaper_accel_per_hz (mission Phase 2, overnight
        # 2026-09-03/04 prep, NOT hardware-qualified yet - deliberately left
        # unset/None below): this guided workflow previously called plain
        # `SHAPER_CALIBRATE AXIS=X`/`AXIS=Y` with no parameters of its own,
        # meaning its actual test envelope was WHATEVER [resonance_tester]'s
        # own max_freq/accel_per_hz happened to be configured to at the
        # time - correct behavior when this command was first built (it
        # matched the same values a plain manual SHAPER_CALIBRATE would
        # use), but an accidental coupling: a future change to
        # [resonance_tester]'s own defaults (for a completely unrelated
        # reason) would silently change this guided workflow's own test
        # envelope too. These two options let this workflow own its own,
        # deliberately-chosen values instead, independent of whatever
        # [resonance_tester] itself defaults to. Left unset (None) by
        # default so the exact current, live-proven-safe behavior (plain
        # `SHAPER_CALIBRATE AXIS=<x>`, using [resonance_tester]'s own
        # max_freq=60/accel_per_hz=70) is completely unchanged until real
        # hardware qualification of a specific wider envelope (see
        # _evidence/ for the 2026-09-03 math verification of an 80Hz/
        # accel_per_hz=50 candidate) promotes real numbers here.
        self.shaper_freq_end = config.getfloat(
            'shaper_freq_end', default=None, above=0.)
        self.shaper_accel_per_hz = config.getfloat(
            'shaper_accel_per_hz', default=None, above=0.)
        # G109 (Run 20): per-axis graphs, drawn by nebulaos_shaper_graph.py
        # in a nice'd child process after each axis. The screen PNG lives
        # next to the journal, the large one in the config folder for
        # Mainsail (latest per axis). A graph that fails or times out never
        # fails the calibration.
        self.shaper_graph_timeout = config.getfloat(
            'shaper_graph_timeout', default=240., above=0.)
        self.shaper_graph_dir = config.get(
            'shaper_graph_dir',
            default=os.path.dirname(self.input_shaper_journal_path))
        self.shaper_graph_config_dir = config.get(
            'shaper_graph_config_dir', default=self._default_graph_config_dir())

        self.z_offset_state = 'idle'
        self.z_offset_id = 0
        self.z_offset_result = None
        self.z_offset_error = None
        # Structured diagnostics (§8/§22) for the most recent LOAD_CELL
        # attempt - the small, Moonraker-status-safe summary fields, NOT
        # the full per-sample ContactSample list (see get_status()'s own
        # comment on why that stays out of normal status).
        self.z_offset_physical_x = None
        self.z_offset_physical_y = None
        self.z_offset_contact_mode = None
        self.z_offset_predicted_nozzle_contact_z = None
        self.z_offset_commanded_floor_z = None
        self.z_offset_raw_probe_trigger_z = None
        self.z_offset_sample_count = None
        self.z_offset_accepted_count = None
        self.z_offset_range = None
        self.z_offset_stddev = None

        # SIMULATE_BOOTSTRAP (mission §8) - a completely separate status
        # namespace from the real z_offset_* fields above, on purpose: a
        # simulation result must never be mistakable for (or accidentally
        # overwrite the display of) a real applied/staged calibration.
        self.bootstrap_sim_id = 0
        # NEBULAOS_RAW_CONTACT_COMPARE (G83b-auto stage C, supervised)
        self.raw_compare_id = 0
        self.raw_compare_state = 'idle'
        self.raw_compare_error = None
        self.raw_compare_result = None
        self.bootstrap_sim_state = 'idle'
        self.bootstrap_sim_result = None
        self.bootstrap_sim_error = None
        self.bootstrap_sim_envelope_mm = None
        self.bootstrap_sim_raw_probe_trigger_z = None
        self.bootstrap_sim_commanded_floor_z = None

        # NEBULAOS_AUTO_CALIBRATE / journal state (mission §11/§12).
        # auto_calibrate_stage mirrors calibration_journal.STAGES; the
        # journal itself is the crash-safe copy of this same information
        # on disk, written at every stage transition (see
        # _auto_calibrate_advance() below) - these in-memory fields are
        # what NEBULAOS_CALIBRATION_STATUS actually reports during a
        # normal (non-crashed) run, cheaper than re-reading the journal
        # file on every status poll.
        self.auto_calibrate_id = 0
        self.auto_calibrate_state = 'idle'
        self.auto_calibrate_stage = None
        self.auto_calibrate_error = None
        self.auto_calibrate_result = None
        self._auto_calibrate_cancel_requested = False

        # NEBULAOS_ESTEPS_CALIBRATE state (mission §14) - a 3-call guided
        # workflow (start/heat -> CONTINUE=1/extrude -> MEASURED=<mm>/
        # apply), completely separate status namespace from everything
        # above for the same reason bootstrap_sim_* is separate.
        self.esteps_id = 0
        self.esteps_state = 'idle'
        self.esteps_error = None
        self.esteps_extruder_name = None
        self.esteps_commanded_length = None
        self.esteps_measured_length = None
        self.esteps_old_rotation_distance = None
        self.esteps_new_rotation_distance = None

        # NEBULAOS_INPUT_SHAPER_CALIBRATE state (mission §13) - its own
        # status namespace, same reasoning as bootstrap_sim_*/esteps_*
        # above.
        self.input_shaper_id = 0
        self.input_shaper_state = 'idle'
        self.input_shaper_stage = None
        self.input_shaper_error = None
        self.input_shaper_result = None
        # Private (not part of get_status()'s public contract) - carry the
        # X measurement and the in-flight journal across the 3 separate
        # gcode calls of the guided workflow, same reasoning as esteps_*'s
        # own cross-call fields above but kept private since these are
        # mid-flight internals, not a result anyone should poll.
        self._input_shaper_type_x = None
        self._input_shaper_freq_x = None
        self._input_shaper_active_journal = None
        # G109 (Run 20): live phase of the axis being measured and the
        # result of each measured axis (see PLAN Run 20 fixed interface).
        self.input_shaper_phase = ''
        self.input_shaper_axis = ''
        self.input_shaper_test_freq = None
        self.input_shaper_graphs_enabled = True
        self.input_shaper_axes = {}
        self._shaper_fits = {}
        self._shaper_csv = None
        register_output = getattr(self.gcode, 'register_output_handler', None)
        if register_output is not None:
            register_output(self._handle_gcode_output)

        # G104/G105: progress, last result and calibration history.
        self._auto_step = None          # 1..10 while auto-calibrate runs
        self._touch = None              # (n, m) during the Z offset touches
        self._mesh_probe_events = 0     # probe results during the bed mesh
        self._esteps_saving = False
        self._history_warning = None
        try:
            self._history, self._history_warning = calibration_history.load(
                self.history_path)
        except Exception:
            logging.exception("nebulaos_calibration: history not loaded")
            self._history = calibration_history.empty_state()
        if self._history_warning:
            logging.warning("nebulaos_calibration: %s", self._history_warning)
        self.printer.register_event_handler(
            "nebulaos:contact_touch", self._handle_contact_touch)
        self.printer.register_event_handler(
            "probe:update_results", self._handle_probe_result)

        self.gcode.register_command(
            'NEBULAOS_CALIBRATION_RECORD', self.cmd_calibration_record,
            desc=self.cmd_calibration_record_help)
        self.gcode.register_command(
            '_NEBULAOS_Z_OFFSET_CALIBRATE', self.cmd_z_offset_calibrate,
            desc=self.cmd_z_offset_calibrate_help)
        # Console-only on purpose: no macro, no UI (architecture review Run
        # 14). SUPERVISED=1 is only a typed flag - Klipper cannot enforce who
        # sends it, so the command logs loudly and never saves anything.
        self.gcode.register_command(
            'NEBULAOS_RAW_CONTACT_COMPARE', self.cmd_raw_contact_compare,
            desc=self.cmd_raw_contact_compare_help)
        self.gcode.register_command(
            'NEBULAOS_CALIBRATION_STATUS', self.cmd_calibration_status,
            desc=self.cmd_calibration_status_help)
        self.gcode.register_command(
            '_NEBULAOS_AUTO_CALIBRATE', self.cmd_auto_calibrate,
            desc=self.cmd_auto_calibrate_help)
        self.gcode.register_command(
            '_NEBULAOS_CALIBRATION_CANCEL', self.cmd_calibration_cancel,
            desc=self.cmd_calibration_cancel_help)
        self.gcode.register_command(
            '_NEBULAOS_ESTEPS_CALIBRATE', self.cmd_esteps_calibrate,
            desc=self.cmd_esteps_calibrate_help)
        self.gcode.register_command(
            '_NEBULAOS_INPUT_SHAPER_CALIBRATE',
            self.cmd_input_shaper_calibrate,
            desc=self.cmd_input_shaper_calibrate_help)
        self.gcode.register_command(
            '_NEBULAOS_INPUT_SHAPER_CANCEL',
            self.cmd_input_shaper_cancel,
            desc="Cancel an in-progress Input Shaper calibration")
        self.gcode.register_command(
            '_NEBULAOS_ESTEPS_CANCEL',
            self.cmd_esteps_cancel,
            desc="Cancel an in-progress E-Steps calibration")
        self.gcode.register_command(
            '_NEBULAOS_ESTEPS_CONFIRM',
            self.cmd_esteps_confirm,
            desc="Confirm and persist a pending E-Steps result")
        self.printer.register_event_handler(
            "klippy:ready", self._handle_ready)

    # ------------------------------------------------------------------
    # Reference XY point
    # ------------------------------------------------------------------
    def _resolve_reference_xy(self, gcmd):
        """The bed point NEBULAOS_Z_OFFSET_CALIBRATE measures at, absent an
        explicit X=/Y= override.

        Phase 2 contact-safety mission (§11): the canonical reference is
        now this section's OWN configured reference_x/reference_y (the
        local sensor area, e.g. the stock-precedented 20,25 region) -
        firmware config's job, not a hardcoded Python constant and not
        borrowed from a different section's concern.

        Falls back to [z_compensate]'s own already-correct, already-
        hardware-qualified home_x/home_y resolution (z_compensate.py's
        _resolve_z_home_xy(), which fixed a real 1.5mm Y-axis bug in an
        earlier mission by preferring [gcode_macro _HOMING_PARAMS]'s real
        homing target over a [bed_mesh]-center approximation) ONLY when
        reference_x/reference_y are both unset - keeps an older
        printer.cfg without those two new options working exactly as
        before, without silently overriding an operator who has not yet
        adopted the new config keys."""
        x = gcmd.get_float('X', None)
        y = gcmd.get_float('Y', None)
        if x is not None and y is not None:
            return x, y
        if self.reference_x is not None and self.reference_y is not None:
            return self.reference_x, self.reference_y
        zc = self.printer.lookup_object('z_compensate', None)
        if zc is None or zc.home_x is None or zc.home_y is None:
            raise self.printer.command_error(
                "NEBULAOS_Z_OFFSET_CALIBRATE: no X=/Y= given, no "
                "reference_x/reference_y configured on [nebulaos_calibration], "
                "and no [z_compensate] reference point is available - "
                "specify X=<pos> Y=<pos> explicitly")
        return zc.home_x, zc.home_y

    def busy(self):
        return any(state in BUSY_STATES for state in (
            self.z_offset_state, self.raw_compare_state,
            self.bootstrap_sim_state, self.auto_calibrate_state,
            self.esteps_state, self.input_shaper_state))

    def get_status(self, eventtime):
        history, last_result = calibration_history.snapshot(self._history)
        return {
            'busy': self.busy(),
            'z_offset_state': self.z_offset_state,
            'z_offset_id': self.z_offset_id,
            'z_offset_result': self.z_offset_result,
            'z_offset_error': self.z_offset_error,
            'z_offset_physical_x': self.z_offset_physical_x,
            'z_offset_physical_y': self.z_offset_physical_y,
            'z_offset_contact_mode': self.z_offset_contact_mode,
            'z_offset_predicted_nozzle_contact_z': self.z_offset_predicted_nozzle_contact_z,
            'z_offset_commanded_floor_z': self.z_offset_commanded_floor_z,
            'z_offset_raw_probe_trigger_z': self.z_offset_raw_probe_trigger_z,
            'z_offset_sample_count': self.z_offset_sample_count,
            'z_offset_accepted_count': self.z_offset_accepted_count,
            'z_offset_range': self.z_offset_range,
            'z_offset_stddev': self.z_offset_stddev,
            'bootstrap_sim_id': self.bootstrap_sim_id,
            'raw_compare_id': self.raw_compare_id,
            'raw_compare_state': self.raw_compare_state,
            'raw_compare_error': self.raw_compare_error,
            'raw_compare_result': self.raw_compare_result,
            'bootstrap_sim_state': self.bootstrap_sim_state,
            'bootstrap_sim_result': self.bootstrap_sim_result,
            'bootstrap_sim_error': self.bootstrap_sim_error,
            'bootstrap_sim_envelope_mm': self.bootstrap_sim_envelope_mm,
            'bootstrap_sim_raw_probe_trigger_z': self.bootstrap_sim_raw_probe_trigger_z,
            'bootstrap_sim_commanded_floor_z': self.bootstrap_sim_commanded_floor_z,
            'auto_calibrate_id': self.auto_calibrate_id,
            'auto_calibrate_state': self.auto_calibrate_state,
            'auto_calibrate_stage': self.auto_calibrate_stage,
            'auto_calibrate_error': self.auto_calibrate_error,
            'auto_calibrate_result': self.auto_calibrate_result,
            'esteps_id': self.esteps_id,
            'esteps_state': self.esteps_state,
            'esteps_error': self.esteps_error,
            'esteps_extruder_name': self.esteps_extruder_name,
            'esteps_commanded_length': self.esteps_commanded_length,
            'esteps_measured_length': self.esteps_measured_length,
            'esteps_old_rotation_distance': self.esteps_old_rotation_distance,
            'esteps_new_rotation_distance': self.esteps_new_rotation_distance,
            'load_cell_calibrated': self._load_cell_calibrated(),
            'input_shaper_id': self.input_shaper_id,
            'input_shaper_state': self.input_shaper_state,
            'input_shaper_stage': self.input_shaper_stage,
            'input_shaper_error': self.input_shaper_error,
            'input_shaper_result': self.input_shaper_result,
            # G109 (PLAN Run 20 fixed interface)
            'input_shaper_phase': self.input_shaper_phase,
            'input_shaper_axis': self.input_shaper_axis,
            'input_shaper_test_freq': self.input_shaper_test_freq,
            'input_shaper_freq_range': self._shaper_freq_range(),
            'input_shaper_graphs_enabled': self.input_shaper_graphs_enabled,
            'input_shaper_axes': self.input_shaper_axes,
            # G104/G105 (PLAN Run 19 fixed interface)
            'progress': self._progress(eventtime),
            'last_result': last_result,
            'history': history,
        }

    # ------------------------------------------------------------------
    # G104: progress of the running guided flow
    # ------------------------------------------------------------------
    @staticmethod
    def _step(flow, steps, n, detail=''):
        key, label, _advice = steps[n - 1]
        return {'flow': flow, 'step': n, 'total': len(steps), 'key': key,
                'label': label, 'detail': detail}

    def _temps(self, eventtime, bed=True):
        """'bed 61/65 C, nozzle 148/150 C' from the live heater status, or
        '' when the heaters are not available."""
        parts = []
        names = (('heater_bed', 'bed'), ('extruder', 'nozzle'))
        for obj_name, label in names if bed else names[1:]:
            heater = self.printer.lookup_object(obj_name, None)
            if heater is None:
                continue
            try:
                st = heater.get_status(eventtime)
                parts.append("%s %.0f/%.0f C" % (
                    label, st['temperature'], st['target']))
            except Exception:
                continue
        return ", ".join(parts)

    def _mesh_point_detail(self):
        """'point n of 81' from upstream's own per-sample probe event
        (probe:update_results) and the configured grid; '' when either is
        not observable."""
        try:
            bed_mesh = self.printer.lookup_object('bed_mesh', None)
            cfg = bed_mesh.bmc.mesh_config
            total = int(cfg['x_count']) * int(cfg['y_count'])
            probe = self.printer.lookup_object('probe')
            samples = max(1, int(probe.get_probe_params()['samples']))
        except Exception:
            return ''
        if total <= 0:
            return ''
        point = min(total, self._mesh_probe_events // samples + 1)
        return "point %d of %d" % (point, total)

    def _auto_progress(self, eventtime):
        n = self._auto_step
        detail = ''
        if n == 4:
            detail = self._temps(eventtime)
        elif n == 7 and self._touch is not None:
            detail = "touch %d of %d" % self._touch
        elif n == 8:
            detail = self._mesh_point_detail()
        return self._step('auto_calibrate', AUTO_CALIBRATE_STEPS, n, detail)

    def _input_shaper_step(self):
        state, stage = self.input_shaper_state, self.input_shaper_stage
        if state not in ('running', 'awaiting_x_mount', 'awaiting_y_mount'):
            return None
        if stage in ('commit', 'restart'):
            return 6
        if stage == 'final_validation':
            return 5
        if state == 'awaiting_y_mount':
            return 5 if stage == 'measure_y' else 4
        if state == 'awaiting_x_mount':
            return 3 if stage == 'measure_x' else 2
        return 1

    def _esteps_step(self):
        if self._esteps_saving:
            return 5
        return {'running': 1, 'awaiting_continue': 2,
                'awaiting_measurement': 3,
                'awaiting_confirmation': 4}.get(self.esteps_state)

    # ------------------------------------------------------------------
    # G109 (Run 20): Input Shaper live feedback and graphs
    # ------------------------------------------------------------------
    def _default_graph_config_dir(self):
        try:
            cfg = self.printer.get_start_args().get('config_file')
        except Exception:
            cfg = None
        if not cfg:
            return ''
        return os.path.join(os.path.dirname(os.path.abspath(cfg)), 'input_shaper')

    def _shaper_freq_range(self):
        try:
            settings = self.printer.lookup_object('configfile').get_status(
                None)['settings'].get('resonance_tester', {})
            lo = float(settings.get('min_freq', 5.))
            hi = float(self.shaper_freq_end or settings.get('max_freq', 135.))
        except Exception:
            return None
        return [lo, hi]

    def _input_shaper_detail(self):
        axis = self.input_shaper_axis.upper()
        phase = self.input_shaper_phase
        if phase == 'measuring':
            if self.input_shaper_test_freq is None:
                return "Shaking %s (about 1-2 minutes)" % (axis,)
            rng = self._shaper_freq_range()
            if rng:
                return "Shaking %s: %.0f Hz (%.0f-%.0f Hz)" % (
                    axis, self.input_shaper_test_freq, rng[0], rng[1])
            return "Shaking %s: %.0f Hz" % (axis, self.input_shaper_test_freq)
        if phase == 'calculating':
            return "Calculating the best shaper for %s..." % (axis,)
        if phase == 'graph':
            return "Drawing the %s graph..." % (axis,)
        return ''

    def _handle_gcode_output(self, msg):
        """Every console line; only read while an axis is measured."""
        if self.input_shaper_phase not in ('measuring', 'calculating'):
            return
        m = _RE_TEST_FREQ.search(msg)
        if m:
            self.input_shaper_test_freq = float(m.group(1))
            return
        if _RE_CALCULATING.search(msg):
            self.input_shaper_phase = 'calculating'
            return
        m = _RE_FITTED.search(msg)
        if m:
            self._shaper_fits[m.group(1)] = {
                'freq': float(m.group(2)), 'vibrations': float(m.group(3)),
                'smoothing': float(m.group(4)), 'max_accel': None}
            return
        m = _RE_MAX_ACCEL.search(msg)
        if m and m.group(1) in self._shaper_fits:
            self._shaper_fits[m.group(1)]['max_accel'] = float(m.group(2))
            return
        m = _RE_CSV.search(msg)
        if m:
            self._shaper_csv = m.group(1)

    def _shaper_axis_info(self, axis, shaper_type, shaper_freq):
        fit = self._shaper_fits.get(shaper_type) or {}
        return {'type': shaper_type, 'freq': round(float(shaper_freq), 1),
                'vibrations': fit.get('vibrations'),
                'smoothing': fit.get('smoothing'),
                'max_accel': fit.get('max_accel'),
                'graph': '', 'graph_error': ''}

    def _wait_child(self, proc, timeout):
        """Waits for the graph process without blocking Klippy (the
        reactor keeps running, like upstream's background_process_exec).
        Returns (returncode, stderr text); kills it after the timeout."""
        reactor = self.printer.get_reactor()
        eventtime = reactor.monotonic()
        deadline = eventtime + timeout
        while proc.poll() is None:
            if eventtime > deadline:
                proc.kill()
                proc.wait()
                return None, "took longer than %.0f s" % (timeout,)
            eventtime = reactor.pause(eventtime + .2)
        err = proc.stderr.read() if proc.stderr is not None else b''
        if isinstance(err, bytes):
            err = err.decode('utf-8', 'replace')
        return proc.returncode, err.strip()

    def _start_graph_process(self, args):
        env = dict(os.environ)
        env['MPLCONFIGDIR'] = '/tmp/nebulaos-matplotlib'
        cmd = [sys.executable, _GRAPH_SCRIPT] + list(args)
        nice = shutil.which('nice')
        if nice:
            cmd = [nice, '-n', '19'] + cmd
        return subprocess.Popen(cmd, stdin=subprocess.DEVNULL,
                                stdout=subprocess.DEVNULL,
                                stderr=subprocess.PIPE, close_fds=True, env=env)

    def _draw_shaper_graph(self, axis, info):
        """Fills info['graph'] or info['graph_error']; never raises."""
        csv = self._shaper_csv
        if not csv or not os.path.isfile(csv):
            info['graph_error'] = "no measurement file from SHAPER_CALIBRATE"
            return
        screen = os.path.join(self.shaper_graph_dir,
                              'input_shaper_%s.png' % (axis,))
        args = [csv, axis, info['type'], '%.1f' % (info['freq'],), screen]
        if self.shaper_graph_config_dir:
            args.append(os.path.join(self.shaper_graph_config_dir,
                                     'shaper_%s.png' % (axis,)))
        self.input_shaper_phase = 'graph'
        try:
            proc = self._start_graph_process(args)
            rc, err = self._wait_child(proc, self.shaper_graph_timeout)
        except Exception as e:
            rc, err = None, _sanitize_error(e)
        if rc == 0 and os.path.isfile(screen):
            info['graph'] = screen
        else:
            info['graph_error'] = (err or "graph not drawn")[:_MAX_ERROR_LEN]
            logging.warning("nebulaos_calibration: input shaper %s graph: %s",
                            axis, info['graph_error'])

    def _reset_shaper_phase(self):
        self.input_shaper_phase = ''
        self.input_shaper_axis = ''
        self.input_shaper_test_freq = None

    def _progress(self, eventtime):
        if self.auto_calibrate_state == 'running' and self._auto_step:
            return self._auto_progress(eventtime)
        n = self._input_shaper_step()
        if n is not None:
            return self._step('input_shaper', INPUT_SHAPER_STEPS, n,
                              self._input_shaper_detail())
        n = self._esteps_step()
        if n is not None:
            return self._step('esteps', ESTEPS_STEPS, n,
                              self._temps(eventtime, bed=False)
                              if n == 1 else '')
        return None

    def _set_auto_step(self, n):
        self._auto_step = n
        if n == 7:
            self._touch = None
        elif n == 8:
            self._mesh_probe_events = 0

    def _handle_contact_touch(self, n, m):
        # nebulaos_probe_pair: nozzle touch n of m is about to start.
        self._touch = (n, m)

    def _handle_probe_result(self, results):
        if self.auto_calibrate_state == 'running' and self._auto_step == 8:
            self._mesh_probe_events += 1

    # ------------------------------------------------------------------
    # G104/G105: last result + calibration history (persisted next to the
    # journal; see nebulaos_calibration_history.py)
    # ------------------------------------------------------------------
    def _save_history(self):
        try:
            calibration_history.save(self.history_path, self._history)
        except Exception:
            # Never fails a calibration: the status keeps the in-memory copy.
            logging.exception("nebulaos_calibration: could not write the "
                              "calibration history %s", self.history_path)

    def record_flow(self, key, summary, save=True):
        """A flow finished successfully (also called by nebulaos_bed_tram
        and nebulaos_load_cell_calibration)."""
        calibration_history.record(self._history, key, summary, time.time())
        if save:
            self._save_history()

    def mark_redo(self, keys, reason):
        calibration_history.mark_redo(self._history, keys, reason)
        self._save_history()

    def _set_last_result(self, flow, ok, summary=(), steps=None, step=None,
                         error='', advice=None, save=True):
        error_step = step_advice = ''
        if not ok and steps is not None and step:
            _key, error_step, step_advice = steps[step - 1]
        if advice is None:
            advice = step_advice
        self._history['last_result'] = calibration_history.make_result(
            flow, ok, time.time(), summary, error_step, error, advice)
        if save:
            self._save_history()

    def _clear_last_result(self):
        if self._history.get('last_result') is not None:
            self._history['last_result'] = None
            self._save_history()

    cmd_calibration_record_help = (
        "Record a finished calibration that NebulaOS does not run itself: "
        "FLOW=pid|axis_twist|skew|tmc_autotune|bed_mesh SUMMARY=\"<text>\" "
        "(shown on the Calibration menu)")

    def cmd_calibration_record(self, gcmd):
        flow = gcmd.get('FLOW', '').strip().lower()
        if flow not in calibration_history.EXTERNAL_FLOWS:
            if flow in calibration_history.FLOWS:
                raise gcmd.error(
                    "NEBULAOS_CALIBRATION_RECORD: FLOW=%s is recorded "
                    "automatically when it finishes" % (flow,))
            raise gcmd.error(
                "NEBULAOS_CALIBRATION_RECORD: unknown FLOW - use one of %s"
                % (", ".join(calibration_history.EXTERNAL_FLOWS),))
        summary = calibration_history.sanitize_text(gcmd.get('SUMMARY', ''))
        self.record_flow(flow, summary)
        gcmd.respond_info("NEBULAOS_CALIBRATION_RECORD: %s recorded%s"
                          % (flow, (" - " + summary) if summary else ""))

    # ------------------------------------------------------------------
    # NEBULAOS_Z_OFFSET_CALIBRATE
    # ------------------------------------------------------------------
    cmd_z_offset_calibrate_help = (
        "Calibrate the BLTouch Z-offset via the local secondary load cell "
        "- for a manual calibration, use pristine upstream PROBE_CALIBRATE "
        "directly. SIMULATE_BOOTSTRAP=1 ENVELOPE=<mm> qualifies the "
        "BOOTSTRAP contact path on an already-calibrated unit without "
        "applying or staging anything - see NEBULAOS_CALIBRATION_STATUS's "
        "bootstrap_sim_* fields")

    def cmd_z_offset_calibrate(self, gcmd):
        """LOAD_CELL only - there is no METHOD=MANUAL passthrough any more
        (see this module's own header comment: call pristine upstream
        PROBE_CALIBRATE directly for a manual run, it needs no wrapper)."""
        z_offset_probe = self.printer.lookup_object('nebulaos_z_offset_probe', None)
        if z_offset_probe is None:
            raise self.printer.command_error(
                "NEBULAOS_Z_OFFSET_CALIBRATE: no [nebulaos_z_offset_probe] "
                "is configured on this printer - use stock PROBE_CALIBRATE "
                "instead, or add [nebulaos_z_offset_probe] to printer.cfg")
        # G99: automatic mode runs the no-motion automatic check here
        # (inside NEBULAOS_AUTO_CALIBRATE: the cold preflight check, then
        # the hot refresh right before this step - _refresh_hot_reference).
        self._require_contact(
            z_offset_probe, 'NEBULAOS_Z_OFFSET_CALIBRATE',
            "run NEBULAOS_LOAD_CELL_CALIBRATE first, or use stock "
            "PROBE_CALIBRATE instead",
            run_check=self.auto_calibrate_state != 'running')

        x, y = self._resolve_reference_xy(gcmd)
        probe_obj = self.printer.lookup_object('probe')
        probe_x_offset, probe_y_offset, probe_z_offset = probe_obj.get_offsets()

        if gcmd.get_int('SIMULATE_BOOTSTRAP', 0):
            self._cmd_z_offset_calibrate_simulate_bootstrap(
                gcmd, x, y, probe_x_offset, probe_y_offset, z_offset_probe)
            return

        self.z_offset_id += 1
        self.z_offset_state = 'running'
        self.z_offset_result = None
        self.z_offset_error = None
        self.z_offset_physical_x = x
        self.z_offset_physical_y = y

        try:
            # Bounded-descent envelope (ESTABLISHED vs BOOTSTRAP, derived
            # from the CURRENT live probe_z_offset), measurement-quality
            # gates, and repeatability aggregation all live inside this
            # call now (§5-§10, corrected) - measure_probe_nozzle_pair()
            # fails closed with CONTACT_SAFETY_LIMIT_UNQUALIFIED before any
            # nozzle contact motion if the envelope constant this run
            # needs (established_contact_margin_mm or bootstrap_contact_
            # envelope_mm), max_abs_fit_delta, or the repeatability bounds
            # are not configured (not yet hardware-qualified). See
            # nebulaos_probe_pair.py's own header for the exact formulas.
            measurement = nebulaos_probe_pair.measure_probe_nozzle_pair(
                self.printer, x, y, probe_x_offset, probe_y_offset,
                probe_z_offset,
                self.horizontal_move_z, z_offset_probe, self.down_min_z,
                pro_cnt=self.pro_cnt, travel_speed=self.travel_speed,
                probe_lift_speed=self.probe_lift_speed,
                established_contact_margin_mm=self.established_contact_margin_mm,
                bootstrap_contact_envelope_mm=self.bootstrap_contact_envelope_mm,
                max_abs_fit_delta=self.max_abs_fit_delta,
                min_accepted_samples=self.min_accepted_samples,
                max_repeatability_range=self.max_repeatability_range,
                max_repeatability_stddev=self.max_repeatability_stddev,
                **self._z_offset_arming_kwargs(z_offset_probe))

            self.z_offset_contact_mode = measurement.contact_mode
            self.z_offset_predicted_nozzle_contact_z = measurement.predicted_nozzle_contact_z
            self.z_offset_commanded_floor_z = measurement.commanded_floor_z
            self.z_offset_raw_probe_trigger_z = measurement.raw_probe_trigger_z
            self.z_offset_sample_count = measurement.repeatability.sample_count
            self.z_offset_accepted_count = measurement.repeatability.accepted_count
            self.z_offset_range = measurement.repeatability.range
            self.z_offset_stddev = measurement.repeatability.stddev

            # §9: a rejected result (bad fit, out-of-envelope, excessive
            # fit delta, or failed repeatability) must NEVER stage a
            # Z-offset, transition to COMPLETE, or call SAVE_CONFIG - the
            # measurement's own `accepted` flag is the single
            # authoritative gate. The toolhead is already lifted clear
            # (measure_probe_nozzle_pair's own final hover()) before this
            # check runs.
            if not measurement.accepted:
                raise self.printer.command_error(
                    "NEBULAOS_Z_OFFSET_CALIBRATE: measurement rejected - "
                    "%s (samples: %d taken, %d accepted)" %
                    (measurement.rejection_reason,
                     measurement.repeatability.sample_count,
                     measurement.repeatability.accepted_count))

            new_offset = measurement.probe_z_offset

            if not math.isfinite(new_offset):
                raise self.printer.command_error(
                    "NEBULAOS_Z_OFFSET_CALIBRATE: measured value %r is "
                    "not a finite number" % (new_offset,))
            # max_offset_correction_mm bounds the CORRECTION - how far this
            # measurement moves the probe's z_offset from the credible prior
            # it is refining - not the absolute z_offset value itself. A
            # thermal-expansion-driven absolute offset near or past this
            # many mm (e.g. established cold ~1.78mm drifting to ~2.1-2.3mm
            # hot) is real calibration data, not automatically implausible;
            # what would be implausible is THIS RUN disagreeing with the
            # PRIOR by more than max_offset_correction_mm. There is no
            # credible prior in BOOTSTRAP mode (see nebulaos_probe_pair.py's
            # _is_credible_probe_z_offset) - measure_probe_nozzle_pair's own
            # bootstrap_contact_envelope_mm bound already gates plausibility
            # for that case, so this check only applies when refining an
            # ESTABLISHED calibration.
            if measurement.contact_mode == 'established':
                correction = new_offset - probe_z_offset
                if abs(correction) > self.max_offset_correction_mm:
                    raise self.printer.command_error(
                        "NEBULAOS_Z_OFFSET_CALIBRATE: measured value %.5fmm "
                        "implies a %.5fmm correction from the current probe "
                        "z_offset %.5fmm, exceeding "
                        "max_offset_correction_mm=%.5fmm - refusing to "
                        "apply an implausibly large correction"
                        % (new_offset, correction, probe_z_offset,
                           self.max_offset_correction_mm))

            # Effective in the CURRENT session immediately (needed before
            # Bed Mesh runs later in a future Auto-Calibrate sequence) -
            # pinned upstream Klipper exposes no public runtime command to
            # replace the registered probe's live z_offset directly (only
            # a full config reload/restart, or Z_OFFSET_APPLY_PROBE +
            # SAVE_CONFIG, which is exactly the restart this must avoid
            # mid-sequence). This is the minimum NebulaOS-side adapter for
            # that gap: confirmed directly against the pinned source
            # (klippy/extras/bltouch.py's PrinterBLTouch.__init__ does
            # `self.probe_offsets = probe.ProbeOffsetsHelper(config)`, and
            # every get_offsets() call reads that same live instance) that
            # mutating probe_obj.probe_offsets.z_offset is the correct,
            # real seam - not an upstream Klipper patch, nothing on disk
            # is touched, and it is undone automatically by the next
            # config (re)load like any other unsaved runtime state.
            #
            # Inside NEBULAOS_AUTO_CALIBRATE the live value is applied by
            # cmd_auto_calibrate() only AFTER its bed mesh (C2): the axes
            # were homed with the OLD z_offset, and a mesh probed with the
            # new one live would be shifted by (old - new).
            if self.auto_calibrate_state != 'running':
                probe_obj.probe_offsets.z_offset = new_offset

            configfile = self.printer.lookup_object('configfile')
            configfile.set(probe_obj.cmd_helper.name, 'z_offset',
                            "%.3f" % (new_offset,))
        except Exception as e:
            # §22 status model: distinguish WHY a run failed where the
            # error message already tells us, rather than collapsing
            # everything to a generic 'error' - a caller polling status
            # can tell "not yet hardware-qualified" apart from "this
            # specific measurement failed quality" apart from "something
            # else broke" without parsing prose.
            text = str(e)
            if 'CONTACT_SAFETY_LIMIT_UNQUALIFIED' in text:
                self.z_offset_state = 'capability_unqualified'
            elif 'measurement rejected' in text:
                self.z_offset_state = 'measurement_quality_failure'
            else:
                self.z_offset_state = 'error'
            self.z_offset_result = None
            self.z_offset_error = _sanitize_error(e)
            raise

        self.z_offset_state = 'complete'
        self.z_offset_result = new_offset
        self.z_offset_error = None
        self.gcode.respond_info(
            "NEBULAOS_Z_OFFSET_CALIBRATE: measured %.5f mm (probe trigger "
            "%.5f, nozzle contact %.5f at X=%.3f Y=%.3f), %s."
            % (new_offset, measurement.raw_probe_trigger_z,
               measurement.raw_nozzle_contact_z, x, y,
               "applied after the bed mesh"
               if self.auto_calibrate_state == 'running' else "applied live"))
        if self.auto_calibrate_state != 'running':
            try:
                self._reset_z_adjust()
            except Exception as e:
                self.z_offset_state = 'error'
                self.z_offset_error = _sanitize_error(e)
                raise
            self.gcode.respond_info(
                "NEBULAOS_Z_OFFSET_CALIBRATE: committing with SAVE_CONFIG, "
                "printer will restart")
            self.gcode.run_script_from_command('SAVE_CONFIG')

    def _z_adjust(self):
        # [nebulaos_z_adjust] is optional: an older printer.cfg has none.
        return self.printer.lookup_object('nebulaos_z_adjust', None)

    def _reset_z_adjust(self):
        """A new z_offset is about to be saved: the saved Z offset
        adjustment (G92) was made against the old one -> 0 (file + live)."""
        z_adjust = self._z_adjust()
        if z_adjust is not None:
            z_adjust.reset_for_calibration()

    def _contact_caller(self):
        return (raw_contact.CALLER_AUTO_CALIBRATE
                if self.auto_calibrate_state == 'running'
                else raw_contact.CALLER_Z_OFFSET_PAIR)

    def _contact_ready(self, z_offset_probe):
        """The guard (brief E.1): the probe's contact_ready() for this
        caller; is_calibrated for a probe without it (unchanged)."""
        ready = getattr(z_offset_probe, 'contact_ready', None)
        if ready is None:
            return z_offset_probe.get_status(
                self.reactor.monotonic())['is_calibrated']
        return ready(self._contact_caller())

    def _load_cell_mode(self, z_offset_probe):
        """'auto' | 'manual', or None for a probe without modes."""
        mode = getattr(z_offset_probe, 'load_cell_mode', None)
        return mode() if mode is not None else None

    def _require_contact(self, z_offset_probe, cmd, manual_hint, run_check):
        """The guard before any motion (G99). Automatic mode: run the
        no-motion automatic check (run_check), then the plan must be raw;
        a failure stops with the reason and offers Manual. Manual mode: the
        bottle calibration must exist (popup offers it), as before."""
        if self._load_cell_mode(z_offset_probe) == raw_contact.USER_AUTO:
            if run_check:
                self.gcode.respond_info(
                    "%s: automatic load cell check (no motion - the bed "
                    "must be empty)" % (cmd,))
                ok, reason = z_offset_probe.run_auto_check()
                if not ok:
                    self._offer_manual_load_cell(reason)
                    raise self.printer.command_error(
                        "%s: the automatic load cell check failed - %s - use "
                        "Calibration > Load Cell > Manual (bottle "
                        "calibration)" % (cmd, reason))
            kind, refusal = z_offset_probe.plan_arming(self._contact_caller())
            if kind is None:
                self._offer_manual_load_cell(refusal[1])
                raise self.printer.command_error(
                    "%s: automatic load cell contact is not ready - %s"
                    % (cmd, refusal[1]))
            return
        if not self._contact_ready(z_offset_probe):
            self._offer_load_cell_calibration()
            raise self.printer.command_error(
                "%s: the load cell has not been calibrated yet - %s"
                % (cmd, manual_hint))

    def _refresh_hot_reference(self, z_offset_probe, cmd, run):
        """G99, automatic mode only: lift the nozzle to horizontal_move_z
        (clear of the bed and the wipe pad), then the no-motion automatic
        check again at the current (hot) temperature. A failure stops with
        the reason and offers Manual, like the cold check."""
        if self._load_cell_mode(z_offset_probe) != raw_contact.USER_AUTO:
            return
        run('SAVE_GCODE_STATE NAME=_nebulaos_hot_ref')
        run('G90')
        run('G1 Z%.3f F%.0f' % (self.horizontal_move_z,
                                self.probe_lift_speed * 60.))
        run('RESTORE_GCODE_STATE NAME=_nebulaos_hot_ref')
        self.gcode.respond_info(
            "%s: automatic load cell check at temperature (no motion - "
            "the bed must be empty)" % (cmd,))
        ok, reason = z_offset_probe.run_auto_check()
        if not ok:
            self._offer_manual_load_cell(reason)
            raise self.printer.command_error(
                "%s: the automatic load cell check at temperature failed - "
                "%s - use Calibration > Load Cell > Manual (bottle "
                "calibration)" % (cmd, reason))
        # G105: in automatic mode this IS the load cell calibration, the same
        # check as NEBULAOS_LOAD_CELL_AUTO_CHECK - record it (clears its redo).
        self.record_flow('load_cell', "Automatic check passed (%s)" % (cmd,))

    def _z_offset_arming_kwargs(self, z_offset_probe):
        """{'arming': plan} for the paired Z-offset path (standalone or
        inside auto-calibrate - G99: both follow the user's mode); {} for a
        probe without plan_arming (calibrated force, as before)."""
        plan = getattr(z_offset_probe, 'plan_arming', None)
        if plan is None:
            return {}
        return {'arming': plan(self._contact_caller())}

    # ------------------------------------------------------------------
    # NEBULAOS_RAW_CONTACT_COMPARE (G83b-auto stage C): supervised
    # manual-vs-automatic comparison on a CALIBRATED unit. Same paired
    # Z-offset path, same fixed reference XY, bounded floor and speeds;
    # first the calibrated touches, then the raw candidate touches. Nothing
    # is ever applied, staged or saved.
    # ------------------------------------------------------------------
    cmd_raw_contact_compare_help = (
        "SUPERVISED stage-C comparison: Z-offset touches with the calibrated "
        "load cell, then with a raw-count candidate - SUPERVISED=1 "
        "TRIGGER_COUNTS=<n> POLARITY=<1|-1> [SAMPLES=<1-5>]. Never saves.")

    RAW_COMPARE_MAX_SAMPLES = 5

    def cmd_raw_contact_compare(self, gcmd):
        if gcmd.get_int('SUPERVISED', 0) != 1:
            raise gcmd.error(
                "NEBULAOS_RAW_CONTACT_COMPARE moves the nozzle onto the bed "
                "with an UNQUALIFIED raw-count trigger: only with someone at "
                "the printer, and only with SUPERVISED=1")
        if gcmd.get('X', None) is not None or gcmd.get('Y', None) is not None:
            raise gcmd.error("NEBULAOS_RAW_CONTACT_COMPARE: X/Y are not "
                             "allowed - it uses the fixed reference point")
        samples = gcmd.get_int('SAMPLES', 3, minval=1,
                               maxval=self.RAW_COMPARE_MAX_SAMPLES)
        trigger_counts = gcmd.get_int('TRIGGER_COUNTS', None)
        polarity = gcmd.get_int('POLARITY', None)
        if trigger_counts is None or polarity not in (1, -1):
            raise gcmd.error("NEBULAOS_RAW_CONTACT_COMPARE: TRIGGER_COUNTS= "
                             "and POLARITY=1|-1 are required")
        print_stats = self.printer.lookup_object('print_stats', None)
        if print_stats is not None:
            state = print_stats.get_status(self.reactor.monotonic()).get('state')
            if state in ('printing', 'paused'):
                raise gcmd.error("NEBULAOS_RAW_CONTACT_COMPARE: not while a "
                                 "print is %s" % (state,))
        toolhead = self.printer.lookup_object('toolhead')
        homed = toolhead.get_status(self.reactor.monotonic()).get('homed_axes', '')
        if not all(a in homed for a in 'xyz'):
            raise gcmd.error("NEBULAOS_RAW_CONTACT_COMPARE: home all axes first")
        z_offset_probe = self.printer.lookup_object('nebulaos_z_offset_probe', None)
        if z_offset_probe is None or not hasattr(z_offset_probe, 'plan_arming'):
            raise gcmd.error("NEBULAOS_RAW_CONTACT_COMPARE: no load cell probe")
        pstat = z_offset_probe.get_status(self.reactor.monotonic())
        if not pstat['is_calibrated']:
            raise gcmd.error("NEBULAOS_RAW_CONTACT_COMPARE: calibrate the load "
                             "cell first - manual is the reference")
        lc = z_offset_probe.get_load_cell()
        candidate, problems = raw_contact.candidate_profile(
            trigger_counts, polarity, lc.get_counts_per_gram(),
            pstat['trigger_force'], pstat['force_safety_limit'],
            lc.get_sensor().get_samples_per_second())
        if candidate is None:
            raise gcmd.error("NEBULAOS_RAW_CONTACT_COMPARE: candidate refused: "
                             "%s" % ("; ".join(problems),))
        x, y = self._resolve_reference_xy(gcmd)   # X/Y refused above
        probe_obj = self.printer.lookup_object('probe')
        probe_x_offset, probe_y_offset, probe_z_offset = probe_obj.get_offsets()
        if not nebulaos_probe_pair._is_credible_probe_z_offset(probe_z_offset):
            raise gcmd.error("NEBULAOS_RAW_CONTACT_COMPARE: needs an "
                             "established probe Z offset (bounded floor)")
        self.gcode.respond_info(
            "!! NEBULAOS_RAW_CONTACT_COMPARE (SUPERVISED): %d calibrated then "
            "%d raw touches at X=%.1f Y=%.1f, candidate trigger %d counts "
            "(%.0f g on this unit), polarity %+d - nothing is saved"
            % (samples, samples, x, y, trigger_counts,
               trigger_counts / lc.get_counts_per_gram(), polarity))
        self.raw_compare_id += 1
        self.raw_compare_state = 'running'
        self.raw_compare_error = None
        self.raw_compare_result = None
        results = {}
        try:
            # The calibrated half is armed explicitly: a touch without a
            # plan follows the user's mode (G99), which may be automatic.
            for label, arming in (
                    ('calibrated', (raw_contact.ARM_CALIBRATED, None)),
                    ('raw', z_offset_probe.plan_arming(
                        raw_contact.CALLER_Z_OFFSET_PAIR, profile=candidate))):
                m = nebulaos_probe_pair.measure_probe_nozzle_pair(
                    self.printer, x, y, probe_x_offset, probe_y_offset,
                    probe_z_offset,
                    self.horizontal_move_z, z_offset_probe, self.down_min_z,
                    pro_cnt=samples, travel_speed=self.travel_speed,
                    probe_lift_speed=self.probe_lift_speed,
                    established_contact_margin_mm=self.established_contact_margin_mm,
                    bootstrap_contact_envelope_mm=self.bootstrap_contact_envelope_mm,
                    max_abs_fit_delta=self.max_abs_fit_delta,
                    min_accepted_samples=self.min_accepted_samples,
                    max_repeatability_range=self.max_repeatability_range,
                    max_repeatability_stddev=self.max_repeatability_stddev,
                    arming=arming)
                st = z_offset_probe.get_status(self.reactor.monotonic())
                results[label] = {
                    'accepted': m.accepted,
                    'rejection_reason': m.rejection_reason,
                    'probe_z_offset': m.probe_z_offset,
                    'range': m.repeatability.range,
                    'stddev': m.repeatability.stddev,
                    'effective_contact_mode': st.get('effective_contact_mode'),
                    'last_peak_force_g': st.get('last_peak_force_g'),
                    'last_force_at_trigger_g': st.get('last_force_at_trigger_g'),
                    'last_raw_peak_delta_counts':
                        st.get('last_raw_peak_delta_counts'),
                }
                self.gcode.respond_info(
                    "RAW_CONTACT_COMPARE %s: accepted=%s z_offset=%s range=%s "
                    "stddev=%s peak=%s g at_trigger=%s g"
                    % (label, m.accepted, m.probe_z_offset,
                       m.repeatability.range, m.repeatability.stddev,
                       st.get('last_peak_force_g'),
                       st.get('last_force_at_trigger_g')))
        except Exception as e:
            self.raw_compare_state = 'error'
            self.raw_compare_error = _sanitize_error(e)
            self.raw_compare_result = results
            raise
        cal, raw = results['calibrated'], results['raw']
        diff = None
        if cal['probe_z_offset'] is not None and raw['probe_z_offset'] is not None:
            diff = raw['probe_z_offset'] - cal['probe_z_offset']
        results['z_offset_difference'] = diff
        self.raw_compare_state = 'complete'
        self.raw_compare_result = results
        self.gcode.respond_info(
            "RAW_CONTACT_COMPARE: raw - calibrated Z offset = %s mm "
            "(NOT applied, NOT saved)" % ("%.4f" % diff if diff is not None
                                          else "n/a",))

    # ------------------------------------------------------------------
    # NEBULAOS_Z_OFFSET_CALIBRATE SIMULATE_BOOTSTRAP=1 (mission §8: a
    # controlled, simulated virgin state for qualifying the BOOTSTRAP
    # contact envelope without ever touching this unit's real,
    # already-calibrated probe_z_offset)
    # ------------------------------------------------------------------
    def _cmd_z_offset_calibrate_simulate_bootstrap(
            self, gcmd, x, y, probe_x_offset, probe_y_offset, z_offset_probe):
        """Forces probe_z_offset=0.0 into measure_probe_nozzle_pair() for
        THIS call only - _is_credible_probe_z_offset(0.0) is False, so the
        REAL BOOTSTRAP code path (classification, formula, motion, quality
        gates) runs for real, with real nozzle contact - without ever
        reading, mutating, or reapplying this unit's own real,
        already-calibrated probe_offsets.z_offset (never looked up in this
        method at all, unlike the real ESTABLISHED path above). The result
        is reported in its own separate bootstrap_sim_* status namespace
        and is NEVER applied live or staged/SAVE_CONFIG'd - this exists
        purely to physically qualify the bootstrap envelope and exercise
        the bootstrap motion/measurement path on hardware that already has
        a trusted calibration. This is exactly the "controlled temporary/
        simulated virgin state" the mission asks for, with an "exact
        backup/restore" of zero work needed: nothing about the real probe
        is ever touched to begin with, so there is nothing to restore.

        ENVELOPE= is required and always explicit here (never falls back
        to self.bootstrap_contact_envelope_mm) - this command's entire
        purpose is qualifying that value BEFORE it becomes trusted
        production config in the first place; a caller who has already
        set a production default can still pass ENVELOPE=<that value>
        explicitly to re-verify it on a specific unit.
        """
        envelope = gcmd.get_float('ENVELOPE', None)
        if envelope is None:
            raise self.printer.command_error(
                "NEBULAOS_Z_OFFSET_CALIBRATE SIMULATE_BOOTSTRAP=1 requires "
                "ENVELOPE=<mm> - this is a qualification aid for "
                "determining bootstrap_contact_envelope_mm itself, so it "
                "never falls back to a config default")

        self.bootstrap_sim_id += 1
        self.bootstrap_sim_state = 'running'
        self.bootstrap_sim_result = None
        self.bootstrap_sim_error = None
        self.bootstrap_sim_envelope_mm = envelope
        self.bootstrap_sim_raw_probe_trigger_z = None
        self.bootstrap_sim_commanded_floor_z = None

        try:
            measurement = nebulaos_probe_pair.measure_probe_nozzle_pair(
                self.printer, x, y, probe_x_offset, probe_y_offset,
                0.0,  # forced - see this method's own docstring
                self.horizontal_move_z, z_offset_probe, self.down_min_z,
                pro_cnt=self.pro_cnt, travel_speed=self.travel_speed,
                probe_lift_speed=self.probe_lift_speed,
                bootstrap_contact_envelope_mm=envelope,
                max_abs_fit_delta=self.max_abs_fit_delta,
                min_accepted_samples=self.min_accepted_samples,
                max_repeatability_range=self.max_repeatability_range,
                max_repeatability_stddev=self.max_repeatability_stddev)

            self.bootstrap_sim_raw_probe_trigger_z = measurement.raw_probe_trigger_z
            self.bootstrap_sim_commanded_floor_z = measurement.commanded_floor_z

            if measurement.contact_mode != 'bootstrap':
                raise self.printer.command_error(
                    "SIMULATE_BOOTSTRAP: internal error - contact_mode=%r, "
                    "expected 'bootstrap' (probe_z_offset=0.0 should "
                    "always classify as BOOTSTRAP)" % (measurement.contact_mode,))

            if not measurement.accepted:
                raise self.printer.command_error(
                    "SIMULATE_BOOTSTRAP: measurement rejected - %s "
                    "(samples: %d taken, %d accepted)" %
                    (measurement.rejection_reason,
                     measurement.repeatability.sample_count,
                     measurement.repeatability.accepted_count))

            result = measurement.probe_z_offset
            if not math.isfinite(result):
                raise self.printer.command_error(
                    "SIMULATE_BOOTSTRAP: measured value %r is not a "
                    "finite number" % (result,))
        except Exception as e:
            self.bootstrap_sim_state = 'error'
            self.bootstrap_sim_error = _sanitize_error(e)
            raise

        self.bootstrap_sim_state = 'complete'
        self.bootstrap_sim_result = result
        self.gcode.respond_info(
            "SIMULATE_BOOTSTRAP: measured %.5f mm at X=%.3f Y=%.3f with "
            "ENVELOPE=%.3fmm (commanded_floor_z=%.5f) - NOT applied or "
            "staged, this is a simulation only"
            % (result, x, y, envelope, measurement.commanded_floor_z))

    # ------------------------------------------------------------------
    # NEBULAOS_AUTO_CALIBRATE / NEBULAOS_CALIBRATION_CANCEL: geometry-
    # only calibration sequence - preflight, home, nozzle clean, establish
    # the calibration thermal state, stabilize, localized Z-offset,
    # upstream bed mesh, final validation, ONE persistence transaction
    # (SAVE_CONFIG), restart, post-restart verification. PID tuning is
    # deliberately excluded (separate PID_BED/PID_HOTEND macros). Every
    # real algorithm (BED_MESH_CALIBRATE, G28, SAVE_CONFIG) stays pristine
    # upstream, dispatched via gcode.run_script_from_command() - this
    # module owns only the sequencing, the journal, and reading back each
    # stage's own already-existing status fields to build the one
    # expected_values dict the post-restart verification checks. Axis
    # Twist is deliberately NOT part of this sequence.
    # ------------------------------------------------------------------
    def _auto_calibrate_advance(self, journal, stage, now):
        """Advance both the in-memory status (what NEBULAOS_CALIBRATION_
        STATUS reports right now) and the on-disk journal (what a future
        boot reads back after a crash) together, in that order - the
        in-memory field is genuinely for THIS process only; a torn write
        to the journal file would still leave this process's own status
        correct for as long as it keeps running."""
        self.auto_calibrate_stage = stage
        self._set_auto_step(_AUTO_STAGE_STEP[stage])
        calibration_journal.advance_stage(journal, stage, now)
        calibration_journal.write_journal(journal, path=self.journal_path)

    def _auto_calibrate_check_cancel(self, journal, now):
        if not self._auto_calibrate_cancel_requested:
            return
        self._auto_calibrate_cancel_requested = False
        calibration_journal.mark_cancelled(journal, now)
        calibration_journal.write_journal(journal, path=self.journal_path)
        raise self.printer.command_error(
            "NEBULAOS_AUTO_CALIBRATE: cancelled by NEBULAOS_CALIBRATION_CANCEL")

    cmd_auto_calibrate_help = (
        "Guided geometry calibration: nozzle clean, localized Z-offset, "
        "upstream bed mesh, one SAVE_CONFIG+restart. PID tuning is "
        "separate (PID_BED/PID_HOTEND). Axis Twist is NOT included.")

    def cmd_auto_calibrate(self, gcmd):
        if self.auto_calibrate_state == 'running':
            raise self.printer.command_error(
                "NEBULAOS_AUTO_CALIBRATE: a calibration is already in progress")

        now = time.time()
        self.auto_calibrate_id += 1
        self.auto_calibrate_state = 'running'
        self.auto_calibrate_stage = None
        self.auto_calibrate_error = None
        self.auto_calibrate_result = None
        self._auto_calibrate_cancel_requested = False
        self._set_auto_step(None)
        self._clear_last_result()
        # G104 end screen: the values before this run.
        old_z_offset = None
        probe_before = self.printer.lookup_object('probe', None)
        if probe_before is not None:
            try:
                old_z_offset = probe_before.get_offsets()[2]
            except Exception:
                old_z_offset = None
        old_z_adjust = getattr(self._z_adjust(), 'value', None)
        if not isinstance(old_z_adjust, (int, float)):
            old_z_adjust = None
        journal = calibration_journal.new_journal(
            self.auto_calibrate_id, 'auto_calibrate', now)
        calibration_journal.write_journal(journal, path=self.journal_path)

        def run(script):
            self.gcode.run_script_from_command(script)

        heated = [False]
        try:
            # preflight: the same load-cell/reference-point checks
            # cmd_z_offset_calibrate() already performs, run up front so a
            # missing prerequisite is caught before any heating/motion at
            # all rather than partway through the sequence.
            self._auto_calibrate_advance(journal, 'preflight', time.time())
            z_offset_probe = self.printer.lookup_object(
                'nebulaos_z_offset_probe', None)
            if z_offset_probe is None:
                raise self.printer.command_error(
                    "NEBULAOS_AUTO_CALIBRATE: no [nebulaos_z_offset_probe] "
                    "is configured on this printer")
            # G99: in automatic mode the no-motion automatic load cell
            # check runs here, first, on the empty bed (a precondition of
            # this sequence anyway) - a failure stops before any motion.
            self._require_contact(
                z_offset_probe, 'NEBULAOS_AUTO_CALIBRATE',
                "run NEBULAOS_LOAD_CELL_CALIBRATE first", run_check=True)
            self._resolve_reference_xy(gcmd)
            self._auto_calibrate_check_cancel(journal, time.time())

            self._auto_calibrate_advance(journal, 'home', time.time())
            run('G28')
            self._auto_calibrate_check_cancel(journal, time.time())

            # Start bed heating early so it soaks during nozzle_clean.
            heated[0] = True
            run('M140 S%.1f' % (self.pid_bed_target,))

            self._auto_calibrate_advance(journal, 'nozzle_clean', time.time())
            # C5: the wipe/press depth follows the live gcode Z offset - a
            # saved Z offset adjustment (or any other) must not shift it.
            # Live only; the saved adjustment is left alone here.
            run('SET_GCODE_OFFSET Z=0 MOVE=0')
            run('_NEBULAOS_NOZZLE_CLEAN')
            self._auto_calibrate_check_cancel(journal, time.time())

            # Confirm exact calibration temperatures. The nozzle is set
            # to z_offset_reference_temp (150C) — nozzle_clean's own
            # hot_start_temp rests at the same value, so this is normally
            # a confirmation, not a second heat cycle. The bed waits for
            # pid_bed_target (65C), started before nozzle_clean above.
            self._auto_calibrate_advance(
                journal, 'establish_thermal_state', time.time())
            run('M104 S%.1f' % (self.z_offset_reference_temp,))
            run('M190 S%.1f' % (self.pid_bed_target,))
            run('M109 S%.1f' % (self.z_offset_reference_temp,))
            self._auto_calibrate_check_cancel(journal, time.time())

            self._auto_calibrate_advance(journal, 'stabilize', time.time())
            if self.thermal_soak_seconds > 0:
                run('G4 P%d' % (int(self.thermal_soak_seconds * 1000),))
            self._auto_calibrate_check_cancel(journal, time.time())

            self._auto_calibrate_advance(
                journal, 'localized_z_offset', time.time())
            # G99: the preflight check was cold; bed and nozzle are hot
            # now. Refresh the empty-bed reference right before the touches
            # (nozzle lifted, no contact; the check waits for the moves and
            # settles) so the drift check compares hot with hot.
            self._set_auto_step(6)
            self._refresh_hot_reference(z_offset_probe,
                                        'NEBULAOS_AUTO_CALIBRATE', run)
            self._set_auto_step(7)
            run('_NEBULAOS_Z_OFFSET_CALIBRATE')
            if self.z_offset_state != 'complete' or self.z_offset_result is None:
                raise self.printer.command_error(
                    "NEBULAOS_AUTO_CALIBRATE: localized Z-offset did not "
                    "complete (state=%r)" % (self.z_offset_state,))
            z_offset_result = self.z_offset_result
            self._auto_calibrate_check_cancel(journal, time.time())

            self._auto_calibrate_advance(journal, 'bed_mesh', time.time())
            run('BED_MESH_CALIBRATE PROFILE=%s' % (self.bed_mesh_profile,))
            # C2: the new z_offset goes live only now, after the mesh was
            # probed with the same z_offset the axes were homed with (the
            # value is already staged in configfile; after the SAVE_CONFIG
            # restart homing uses it).
            self.printer.lookup_object('probe').probe_offsets.z_offset = \
                z_offset_result
            self._auto_calibrate_check_cancel(journal, time.time())

            self._auto_calibrate_advance(
                journal, 'final_validation', time.time())
            bed_mesh = self.printer.lookup_object('bed_mesh', None)
            mesh_status = (bed_mesh.get_status(self.reactor.monotonic())
                            if bed_mesh is not None else {})
            if not mesh_status.get('profile_name'):
                raise self.printer.command_error(
                    "NEBULAOS_AUTO_CALIBRATE: final validation failed - "
                    "no bed_mesh profile is active after BED_MESH_CALIBRATE")
            expected_values = {
                'bltouch.z_offset': round(z_offset_result, 3),
                'bed_mesh.profile': mesh_status['profile_name'],
            }
            self._auto_calibrate_check_cancel(journal, time.time())

            self._reset_z_adjust()
            # G104: the end-screen lines travel in the journal across the
            # SAVE_CONFIG restart; _handle_ready() publishes them once the
            # saved values are verified.
            journal['summary'] = auto_calibrate_summary(
                z_offset_result, old_z_offset, mesh_status, old_z_adjust)
            journal['history_summary'] = auto_calibrate_history_summary(
                z_offset_result, mesh_status)
            calibration_journal.mark_commit_requested(
                journal, expected_values, time.time())
            calibration_journal.write_journal(journal, path=self.journal_path)
            self.auto_calibrate_stage = 'commit'
            self._set_auto_step(9)
            self.gcode.respond_info(
                "NEBULAOS_AUTO_CALIBRATE: all stages complete (z_offset=%.5f, "
                "bed_mesh profile=%r) - committing with SAVE_CONFIG, "
                "printer will restart"
                % (z_offset_result, mesh_status['profile_name']))
            run('SAVE_CONFIG')
            # On upstream Klipper, SAVE_CONFIG's own restart never returns
            # control here at all. Root-caused live 2026-09-03: on this
            # platform's own reactor, the restart is queued and this
            # gcode handler DOES return normally, moments before the
            # actual restart happens - confirmed by the Moonraker HTTP
            # caller seeing a clean response here immediately followed by
            # a real klippy restart. Either way, returning normally here
            # is correct: the journal's commit_requested/restart_pending/
            # verification_pending were already written to disk above,
            # before SAVE_CONFIG ran, so _handle_ready() on the next boot
            # (whichever process reaches it) is what actually verifies
            # and finalizes auto_calibrate_state - this function's own
            # in-memory state staying at 'running' in the old process
            # until it's replaced is expected, not a bug.
            return
        except Exception as e:
            if journal.get('commit_requested'):
                # mark_commit_requested() already ran and was written to
                # disk BEFORE the SAVE_CONFIG call above - in production
                # that call never returns at all (klippy restarts); any
                # exception surfacing from it here (or from anything
                # after it, though nothing runs after it on the success
                # path) must NOT downgrade the already-correct
                # commit_requested/restart_pending/verification_pending
                # journal state back to an error - that would be
                # actively wrong, since the commit truly happened. Only
                # _handle_ready()'s own post-restart verification may
                # ever change this journal's state from here.
                raise
            self.auto_calibrate_state = (
                'cancelled' if 'cancelled by NEBULAOS_CALIBRATION_CANCEL'
                in str(e) else 'error')
            self.auto_calibrate_error = _sanitize_error(e)
            failed_step = self._auto_step or 1
            self._set_auto_step(None)
            if self.auto_calibrate_state == 'cancelled':
                self._set_last_result(
                    'auto_calibrate', False,
                    steps=AUTO_CALIBRATE_STEPS, step=failed_step,
                    error="Cancelled", advice='')
            else:
                self._set_last_result(
                    'auto_calibrate', False, steps=AUTO_CALIBRATE_STEPS,
                    step=failed_step, error=self.auto_calibrate_error)
            if journal['state'] not in (calibration_journal.STATE_CANCELLED,):
                calibration_journal.mark_error(
                    journal, self.auto_calibrate_error, time.time())
                calibration_journal.write_journal(journal, path=self.journal_path)
            # Run 18 architect note: a failed or cancelled run must not leave
            # the bed/nozzle heating until idle_timeout (wasted heat, and the
            # NebulaOS busy check reports ACTIVE, blocking restarts/updates).
            try:
                if heated[0]:
                    self.gcode.run_script_from_command("TURN_OFF_HEATERS")
            except Exception:
                logging.exception("nebulaos_calibration: could not turn "
                                  "the heaters off after a failed run")
            # The live Z offset was zeroed for the nozzle clean (C5): give
            # the user's saved adjustment back.
            z_adjust = self._z_adjust()
            if z_adjust is not None:
                try:
                    z_adjust.apply_saved()
                except Exception:
                    logging.exception("nebulaos_calibration: could not "
                                      "re-apply the Z offset adjustment")
            raise

    cmd_calibration_cancel_help = (
        "Request cancellation of an in-progress NEBULAOS_AUTO_CALIBRATE - "
        "takes effect at the next stage boundary, never mid-stage")

    def cmd_calibration_cancel(self, gcmd):
        if self.auto_calibrate_state != 'running':
            raise self.printer.command_error(
                "NEBULAOS_CALIBRATION_CANCEL: no NEBULAOS_AUTO_CALIBRATE "
                "is currently running")
        self._auto_calibrate_cancel_requested = True
        self.gcode.respond_info(
            "NEBULAOS_CALIBRATION_CANCEL: requested - will take effect at "
            "the next stage boundary")

    # ------------------------------------------------------------------
    # NEBULAOS_ESTEPS_CALIBRATE (mission §14): a standalone guided
    # workflow, NOT part of NEBULAOS_AUTO_CALIBRATE. Internally just
    # upstream `rotation_distance` - "E-Steps" is product-facing naming
    # only, matching the mission's own wording. Real motion/heating stays
    # upstream (M104/M109, G92/G1, SET_EXTRUDER_ROTATION_DISTANCE) -
    # NebulaOS owns only the guided sequencing, the formula, and staging.
    #
    # Three calls, one human interaction, matching the REAL physical
    # procedure (the filament must be marked BEFORE the guided extrusion
    # happens, not after - there is no way to ask a human to mark a
    # moving filament mid-extrusion):
    #   1. NEBULAOS_ESTEPS_CALIBRATE                    - preflight, heat,
    #      wait for temp. Stops here and asks the human to mark the
    #      filament at the extruder's entry point now.
    #   2. NEBULAOS_ESTEPS_CALIBRATE CONTINUE=1         - extrudes exactly
    #      esteps_commanded_length_mm. Stops here and asks for the ONE
    #      real measurement: how far the mark actually moved.
    #   3. NEBULAOS_ESTEPS_CALIBRATE MEASURED=<mm>      - computes, sanity
    #      -checks, and applies/stages the new rotation_distance.
    # ------------------------------------------------------------------
    cmd_esteps_calibrate_help = (
        "Guided E-Steps (rotation_distance) calibration - call with no "
        "params to start (heats and waits for you to mark the filament), "
        "CONTINUE=1 to extrude the commanded length, MEASURED=<mm> to "
        "apply the result. EXTRUDER=<name> to target a non-default "
        "extruder (default 'extruder').")

    def cmd_esteps_calibrate(self, gcmd):
        measured = gcmd.get_float('MEASURED', None)
        do_continue = gcmd.get_int('CONTINUE', 0)
        extruder_name = gcmd.get('EXTRUDER', 'extruder')

        if measured is not None:
            self._esteps_apply(gcmd, measured)
        elif do_continue:
            self._esteps_extrude(gcmd)
        else:
            self._esteps_start(gcmd, extruder_name)

    def _esteps_failed(self, exc):
        # G109: no prompt stays open after a failure
        self._dismiss_action_prompt()
        failed_step = self._esteps_step() or 1
        self.esteps_state = 'error'
        self.esteps_error = _sanitize_error(exc)
        self._esteps_saving = False
        self._set_last_result('esteps', False, steps=ESTEPS_STEPS,
                              step=failed_step, error=self.esteps_error)

    def _esteps_lookup_extruder(self, name):
        extruder_obj = self.printer.lookup_object(name, None)
        if extruder_obj is None or getattr(
                extruder_obj, 'extruder_stepper', None) is None:
            raise self.printer.command_error(
                "NEBULAOS_ESTEPS_CALIBRATE: '%s' is not a configured "
                "extruder with a rotation_distance stepper" % (name,))
        return extruder_obj

    def _esteps_start(self, gcmd, extruder_name):
        if self.esteps_state in ('awaiting_continue', 'awaiting_measurement'):
            raise self.printer.command_error(
                "NEBULAOS_ESTEPS_CALIBRATE: a guided run is already in "
                "progress (state=%s) - finish it with CONTINUE=1/MEASURED=, "
                "or power-cycle to abandon it" % (self.esteps_state,))
        extruder_obj = self._esteps_lookup_extruder(extruder_name)

        self.esteps_id += 1
        self.esteps_state = 'running'
        self.esteps_error = None
        self.esteps_extruder_name = extruder_name
        self.esteps_commanded_length = None
        self.esteps_measured_length = None
        self.esteps_old_rotation_distance = None
        self.esteps_new_rotation_distance = None
        self._esteps_saving = False
        self._clear_last_result()

        try:
            self.gcode.run_script_from_command('M104 S%.1f' % (self.esteps_temp,))
            self.gcode.run_script_from_command('M109 S%.1f' % (self.esteps_temp,))
        except Exception as e:
            self._esteps_failed(e)
            raise

        self.esteps_state = 'awaiting_continue'
        self._send_action_prompt(
            "E-Steps Calibration",
            ["Hotend at %.0fC." % (self.esteps_temp,),
             "Mark the filament at the extruder entry point now.",
             "When marked, click Start Extrusion to extrude %.0fmm."
             % (self.esteps_commanded_length_mm,)],
            [("Start Extrusion",
              "_NEBULAOS_ESTEPS_CALIBRATE CONTINUE=1"),
             ("Cancel",
              "_NEBULAOS_ESTEPS_CANCEL")])

    def _esteps_extrude(self, gcmd):
        if self.esteps_state != 'awaiting_continue':
            raise self.printer.command_error(
                "NEBULAOS_ESTEPS_CALIBRATE CONTINUE=1: no guided run is "
                "waiting to extrude (state=%s) - start one first with "
                "NEBULAOS_ESTEPS_CALIBRATE" % (self.esteps_state,))
        extruder_obj = self._esteps_lookup_extruder(self.esteps_extruder_name)
        # G109: the "mark the filament" prompt closes when the extrusion starts
        self._dismiss_action_prompt()

        try:
            old_rotation_distance = (
                extruder_obj.extruder_stepper.stepper.get_rotation_distance()[0])
            self.esteps_old_rotation_distance = old_rotation_distance
            # M82 (absolute extrusion) + G92 E0 makes this extrusion
            # self-contained regardless of whatever extrusion mode/offset
            # a prior print or macro left active.
            self.gcode.run_script_from_command('M82')
            self.gcode.run_script_from_command('G92 E0')
            self.gcode.run_script_from_command(
                'G1 E%.4f F%d' % (self.esteps_commanded_length_mm,
                                  int(self.esteps_extrude_speed_mm_s * 60)))
        except Exception as e:
            self._esteps_failed(e)
            raise

        self.esteps_commanded_length = self.esteps_commanded_length_mm
        self.esteps_state = 'awaiting_measurement'
        self._send_action_prompt(
            "E-Steps Calibration",
            ["Extruded %.0fmm (commanded)." % (self.esteps_commanded_length,),
             "Measure how far the mark actually moved.",
             "Enter it on the printer screen (E-Steps panel) or in the console:",
             "NEBULAOS_ESTEPS_CALIBRATE MEASURED=<mm>"],
            # Closes only the dialog (standard Klipper RESPOND), so a screen's
            # own entry field is reachable; the run keeps waiting (D3).
            [("Enter measurement",
              "RESPOND TYPE=command MSG=action:prompt_end"),
             ("Cancel",
              "_NEBULAOS_ESTEPS_CANCEL")])

    def _esteps_apply(self, gcmd, measured):
        if self.esteps_state != 'awaiting_measurement':
            raise self.printer.command_error(
                "NEBULAOS_ESTEPS_CALIBRATE MEASURED=: no guided run is "
                "waiting for a measurement (state=%s) - start one first "
                "with NEBULAOS_ESTEPS_CALIBRATE" % (self.esteps_state,))
        try:
            if not math.isfinite(measured) or measured <= 0.:
                raise self.printer.command_error(
                    "NEBULAOS_ESTEPS_CALIBRATE MEASURED=%r must be a "
                    "finite, positive length in mm" % (measured,))
            commanded = self.esteps_commanded_length
            old_rd = self.esteps_old_rotation_distance
            ratio = measured / commanded
            new_rd = old_rd * ratio
            if abs(ratio - 1.0) > self.esteps_max_correction_ratio:
                raise self.printer.command_error(
                    "NEBULAOS_ESTEPS_CALIBRATE: measured %.3fmm vs "
                    "commanded %.3fmm implies a %.1f%% correction, "
                    "exceeding esteps_max_correction_ratio=%.1f%% - "
                    "refusing an implausible result (check for a "
                    "units/typo mistake in MEASURED=)"
                    % (measured, commanded, (ratio - 1.0) * 100.,
                       self.esteps_max_correction_ratio * 100.))
        except Exception as e:
            self._esteps_failed(e)
            raise

        self.esteps_measured_length = measured
        self.esteps_new_rotation_distance = new_rd
        self.esteps_state = 'awaiting_confirmation'
        self._send_action_prompt(
            "E-Steps Calibration — Confirm",
            ["Old rotation_distance: %.6f" % (old_rd,),
             "Commanded extrusion: %.1fmm" % (commanded,),
             "Measured extrusion: %.1fmm" % (measured,),
             "Proposed rotation_distance: %.6f" % (new_rd,)],
            [("Apply & Save",
              "_NEBULAOS_ESTEPS_CONFIRM"),
             ("Cancel",
              "_NEBULAOS_ESTEPS_CANCEL")])

    # ------------------------------------------------------------------
    # NEBULAOS_INPUT_SHAPER_CALIBRATE (mission §13). Corrected live
    # 2026-09-03: an earlier draft of this command assumed the ADXL345 was
    # fixed-mounted (matching this unit's own [adxl345] SPI wiring
    # comment in machine.cfg) and ran a single combined `SHAPER_CALIBRATE`
    # pass for both axes - the operator confirmed live that the real
    # accelerometer on this printer is in fact a separate, movable
    # clip-on module, not the fixed one machine.cfg's own comment
    # describes (that comment is accurate about the WIRING/electrical
    # path, just not about whether the sensor itself is rigidly fixed in
    # one physical spot). A movable sensor needs a genuine relocate-
    # between-axes guided workflow, matching this mission's own §7/§8
    # requirement and mirroring NEBULAOS_ESTEPS_CALIBRATE's own 3-call
    # guided pattern (the human step must happen BETWEEN two real
    # measurements, not before or after both):
    #   1. NEBULAOS_INPUT_SHAPER_CALIBRATE            - preflight, home.
    #      Stops here and asks the human to mount the accelerometer
    #      rigidly on the TOOLHEAD now.
    #   2. NEBULAOS_INPUT_SHAPER_CALIBRATE CONTINUE=1 - runs upstream
    #      `SHAPER_CALIBRATE AXIS=X`. Stops here and asks the human to
    #      move the SAME accelerometer rigidly to the BED now.
    #   3. NEBULAOS_INPUT_SHAPER_CALIBRATE CONTINUE=1 - runs upstream
    #      `SHAPER_CALIBRATE AXIS=Y`, then commits both results with one
    #      SAVE_CONFIG+restart+post-restart verification, same journal
    #      pattern as NEBULAOS_AUTO_CALIBRATE.
    # ------------------------------------------------------------------
    def _send_action_prompt(self, title, text_lines, buttons=None):
        self.gcode.respond_raw("// action:prompt_end")
        self.gcode.respond_raw("// action:prompt_begin %s" % (title,))
        for line in text_lines:
            self.gcode.respond_raw("// action:prompt_text %s" % (line,))
        if buttons:
            for label, gcode_cmd in buttons:
                self.gcode.respond_raw(
                    "// action:prompt_button %s|%s" % (label, gcode_cmd))
        self.gcode.respond_raw("// action:prompt_show")

    def _dismiss_action_prompt(self):
        self.gcode.respond_raw("// action:prompt_end")

    cmd_input_shaper_calibrate_help = (
        "Guided Input Shaper calibration for a MOVABLE clip-on "
        "accelerometer - call with no params to home and preflight; call "
        "again with CONTINUE=1 once it's mounted on the TOOLHEAD (measures "
        "X); call a third time with CONTINUE=1 once it's been moved to the "
        "BED (measures Y, then commits with SAVE_CONFIG+restart). GRAPHS=0 "
        "skips the per-axis graphs (default GRAPHS=1)")

    def cmd_input_shaper_calibrate(self, gcmd):
        if gcmd.get_int('CONTINUE', 0):
            if self.input_shaper_state == 'awaiting_x_mount':
                return self._input_shaper_measure_axis(gcmd, 'x')
            if self.input_shaper_state == 'awaiting_y_mount':
                return self._input_shaper_measure_axis(gcmd, 'y')
            raise self.printer.command_error(
                "NEBULAOS_INPUT_SHAPER_CALIBRATE CONTINUE=1: no guided run "
                "is waiting for a sensor move (state=%s) - start one first "
                "with a plain NEBULAOS_INPUT_SHAPER_CALIBRATE"
                % (self.input_shaper_state,))
        return self._input_shaper_start(gcmd)

    def _input_shaper_start(self, gcmd):
        if self.input_shaper_state in ('awaiting_x_mount', 'awaiting_y_mount'):
            raise self.printer.command_error(
                "NEBULAOS_INPUT_SHAPER_CALIBRATE: a guided run is already "
                "in progress (state=%s) - finish it with CONTINUE=1, or "
                "power-cycle to abandon it" % (self.input_shaper_state,))

        now = time.time()
        self.input_shaper_id += 1
        self.input_shaper_state = 'running'
        self.input_shaper_stage = None
        self.input_shaper_error = None
        self.input_shaper_result = None
        self._input_shaper_type_x = None
        self._input_shaper_freq_x = None
        self.input_shaper_graphs_enabled = bool(gcmd.get_int('GRAPHS', 1))
        self.input_shaper_axes = {}
        self._reset_shaper_phase()
        self._clear_last_result()
        journal = calibration_journal.new_journal(
            self.input_shaper_id, 'input_shaper_calibrate', now)
        calibration_journal.write_journal(
            journal, path=self.input_shaper_journal_path)
        self._input_shaper_active_journal = journal

        def advance(stage, t):
            self.input_shaper_stage = stage
            calibration_journal.advance_stage(
                journal, stage, t,
                stages=calibration_journal.INPUT_SHAPER_STAGES)
            calibration_journal.write_journal(
                journal, path=self.input_shaper_journal_path)

        try:
            advance('preflight', time.time())
            if self.printer.lookup_object('resonance_tester', None) is None:
                raise self.printer.command_error(
                    "NEBULAOS_INPUT_SHAPER_CALIBRATE: no [resonance_tester] "
                    "is configured on this printer")
            if self.printer.lookup_object('input_shaper', None) is None:
                raise self.printer.command_error(
                    "NEBULAOS_INPUT_SHAPER_CALIBRATE: no [input_shaper] is "
                    "configured on this printer")

            advance('home', time.time())
            self.gcode.run_script_from_command('G28')
        except Exception as e:
            self._dismiss_action_prompt()
            self.input_shaper_state = 'error'
            self.input_shaper_error = _sanitize_error(e)
            self._set_last_result('input_shaper', False,
                                  steps=INPUT_SHAPER_STEPS, step=1,
                                  error=self.input_shaper_error)
            calibration_journal.mark_error(
                journal, self.input_shaper_error, time.time())
            calibration_journal.write_journal(
                journal, path=self.input_shaper_journal_path)
            raise

        self.input_shaper_state = 'awaiting_x_mount'
        self._send_action_prompt(
            "Input Shaper Calibration",
            ["Homed successfully.",
             "Mount the accelerometer RIGIDLY on the TOOLHEAD now."],
            [("Continue",
              "_NEBULAOS_INPUT_SHAPER_CALIBRATE CONTINUE=1"),
             ("Cancel",
              "_NEBULAOS_INPUT_SHAPER_CANCEL")])

    def _build_shaper_calibrate_script(self, axis):
        """The exact upstream SHAPER_CALIBRATE invocation this workflow
        runs for one axis. Deliberately just a plain string builder - no
        resonance/shaper analysis of any kind lives here, that stays
        entirely upstream in resonance_tester.py/shaper_calibrate.py.
        FREQ_END=/ACCEL_PER_HZ= are appended only when this section's own
        shaper_freq_end/shaper_accel_per_hz are configured; left unset,
        the command is bare `SHAPER_CALIBRATE AXIS=<x>`, which upstream
        itself then resolves against [resonance_tester]'s own max_freq/
        accel_per_hz - byte-for-byte the same command this workflow has
        always sent, so the unset case is a pure no-op change from the
        live-proven-safe 2026-09-03 behavior."""
        parts = ['SHAPER_CALIBRATE', 'AXIS=%s' % (axis.upper(),)]
        if self.shaper_freq_end is not None:
            parts.append('FREQ_END=%.1f' % (self.shaper_freq_end,))
        if self.shaper_accel_per_hz is not None:
            parts.append('ACCEL_PER_HZ=%.1f' % (self.shaper_accel_per_hz,))
        return ' '.join(parts)

    def _input_shaper_measure_axis(self, gcmd, axis):
        journal = self._input_shaper_active_journal
        stage = 'measure_%s' % (axis,)

        def advance(s, t):
            self.input_shaper_stage = s
            calibration_journal.advance_stage(
                journal, s, t, stages=calibration_journal.INPUT_SHAPER_STAGES)
            calibration_journal.write_journal(
                journal, path=self.input_shaper_journal_path)

        # G109: Continue was pressed - its prompt closes now, not when the
        # next one opens (the screen shows the live phase instead)
        self._dismiss_action_prompt()
        try:
            advance(stage, time.time())
            self.input_shaper_axis = axis
            self.input_shaper_phase = 'measuring'
            self.input_shaper_test_freq = None
            self._shaper_fits = {}
            self._shaper_csv = None
            self.gcode.run_script_from_command(
                self._build_shaper_calibrate_script(axis))
            input_shaper_obj = self.printer.lookup_object('input_shaper')
            shapers_by_axis = {
                s.axis: s for s in input_shaper_obj.get_shapers()}
            shaper = shapers_by_axis.get(axis)
            if shaper is None or not shaper.params.shaper_freq:
                raise self.printer.command_error(
                    "NEBULAOS_INPUT_SHAPER_CALIBRATE: SHAPER_CALIBRATE "
                    "AXIS=%s did not produce a usable shaper_freq"
                    % (axis.upper(),))
            info = self._shaper_axis_info(axis, shaper.params.shaper_type,
                                          shaper.params.shaper_freq)
            if self.input_shaper_graphs_enabled:
                self._draw_shaper_graph(axis, info)
            axes = dict(self.input_shaper_axes)
            axes[axis] = info
            self.input_shaper_axes = axes
            journal['axes'] = axes
            calibration_journal.write_journal(
                journal, path=self.input_shaper_journal_path)
            self._reset_shaper_phase()

            if axis == 'x':
                self._input_shaper_type_x = shaper.params.shaper_type
                self._input_shaper_freq_x = round(shaper.params.shaper_freq, 3)
                self.input_shaper_state = 'awaiting_y_mount'
                lines = ["X axis measured: %s @ %.1fHz."
                         % (shaper.params.shaper_type,
                            shaper.params.shaper_freq)]
                if info['graph'] and self.shaper_graph_config_dir:
                    lines.append("Graph: config folder, input_shaper/shaper_x.png")
                lines.append("Move the SAME accelerometer RIGIDLY to the BED now.")
                self._send_action_prompt(
                    "Input Shaper Calibration", lines,
                    [("Continue",
                      "_NEBULAOS_INPUT_SHAPER_CALIBRATE CONTINUE=1"),
                     ("Cancel",
                      "_NEBULAOS_INPUT_SHAPER_CANCEL")])
                return

            advance('final_validation', time.time())
            expected_values = {
                'input_shaper.shaper_type_x': self._input_shaper_type_x,
                'input_shaper.shaper_freq_x': self._input_shaper_freq_x,
                'input_shaper.shaper_type_y': shaper.params.shaper_type,
                'input_shaper.shaper_freq_y':
                    round(shaper.params.shaper_freq, 3),
            }
            journal['summary'] = [
                "X: %s %.1f Hz" % (self._input_shaper_type_x,
                                   self._input_shaper_freq_x),
                "Y: %s %.1f Hz" % (shaper.params.shaper_type,
                                   shaper.params.shaper_freq)]
            journal['history_summary'] = "; ".join(journal['summary'])
            calibration_journal.mark_commit_requested(
                journal, expected_values, time.time())
            calibration_journal.write_journal(
                journal, path=self.input_shaper_journal_path)
            self.input_shaper_stage = 'commit'
            self.gcode.respond_info(
                "NEBULAOS_INPUT_SHAPER_CALIBRATE: shaper_x=%s@%.1fHz "
                "shaper_y=%s@%.1fHz - committing with SAVE_CONFIG, printer "
                "will restart"
                % (self._input_shaper_type_x, self._input_shaper_freq_x,
                   shaper.params.shaper_type, shaper.params.shaper_freq))
            self._dismiss_action_prompt()
            self.gcode.run_script_from_command('SAVE_CONFIG')
            # See cmd_auto_calibrate's own identical comment/root-cause -
            # returning normally here is correct on this platform.
            return
        except Exception as e:
            self._reset_shaper_phase()
            self._dismiss_action_prompt()
            if journal.get('commit_requested'):
                raise
            failed_step = self._input_shaper_step() or 1
            self.input_shaper_state = 'error'
            self.input_shaper_error = _sanitize_error(e)
            self._set_last_result('input_shaper', False,
                                  steps=INPUT_SHAPER_STEPS, step=failed_step,
                                  error=self.input_shaper_error)
            if journal['state'] != calibration_journal.STATE_CANCELLED:
                calibration_journal.mark_error(
                    journal, self.input_shaper_error, time.time())
                calibration_journal.write_journal(
                    journal, path=self.input_shaper_journal_path)
            raise

    def cmd_input_shaper_cancel(self, gcmd):
        if self.input_shaper_state not in ('awaiting_x_mount', 'awaiting_y_mount'):
            raise self.printer.command_error(
                "No Input Shaper calibration is in progress to cancel")
        journal = self._input_shaper_active_journal
        self.input_shaper_state = 'cancelled'
        self.input_shaper_error = None
        self._input_shaper_type_x = None
        self._input_shaper_freq_x = None
        self._reset_shaper_phase()
        self._dismiss_action_prompt()
        if journal is not None:
            calibration_journal.mark_cancelled(journal, time.time())
            calibration_journal.write_journal(
                journal, path=self.input_shaper_journal_path)
        self._input_shaper_active_journal = None
        self.gcode.respond_info(
            "Input Shaper calibration cancelled.")

    def cmd_esteps_cancel(self, gcmd):
        if self.esteps_state not in ('awaiting_continue', 'awaiting_measurement',
                                     'awaiting_confirmation'):
            raise self.printer.command_error(
                "No E-Steps calibration is in progress to cancel")
        self.esteps_state = 'cancelled'
        self.esteps_error = None
        self.esteps_new_rotation_distance = None
        self._dismiss_action_prompt()
        self.gcode.respond_info(
            "E-Steps calibration cancelled. Original rotation_distance preserved.")

    def cmd_esteps_confirm(self, gcmd):
        if self.esteps_state != 'awaiting_confirmation':
            raise self.printer.command_error(
                "No E-Steps result is waiting for confirmation")
        try:
            new_rd = self.esteps_new_rotation_distance
            extruder_obj = self._esteps_lookup_extruder(self.esteps_extruder_name)
            self.gcode.run_script_from_command(
                'SET_EXTRUDER_ROTATION_DISTANCE EXTRUDER=%s DISTANCE=%.6f'
                % (self.esteps_extruder_name, new_rd))
            configfile = self.printer.lookup_object('configfile')
            configfile.set(self.esteps_extruder_name, 'rotation_distance',
                            "%.6f" % (new_rd,))
        except Exception as e:
            self._esteps_failed(e)
            raise

        self.esteps_state = 'complete'
        self.esteps_error = None
        # G104/G105: no post-restart verification for E-Steps - the
        # result and history are written before the restart.
        summary = ["Rotation distance %.4f (was %.4f)"
                   % (new_rd, self.esteps_old_rotation_distance)]
        self.record_flow('esteps', summary[0], save=False)
        self._set_last_result('esteps', True, summary)
        self._esteps_saving = True
        self._dismiss_action_prompt()
        self.gcode.respond_info(
            "NEBULAOS_ESTEPS_CALIBRATE: rotation_distance %.6f applied "
            "and staged - committing with SAVE_CONFIG, printer will restart"
            % (new_rd,))
        self.gcode.run_script_from_command('SAVE_CONFIG')

    # ------------------------------------------------------------------
    # Load cell guard (G83a). The guided calibration itself lives in
    # nebulaos_load_cell_calibration.py (G86).
    # ------------------------------------------------------------------
    def _load_cell_calibrated(self):
        probe = self.printer.lookup_object('nebulaos_z_offset_probe', None)
        if probe is None or not hasattr(probe, 'get_load_cell'):
            return None
        try:
            return bool(probe.get_load_cell().is_calibrated())
        except Exception:
            return None

    def _offer_load_cell_calibration(self):
        """Guard popup: the screen and Mainsail show it as a prompt with a
        button that starts the guided bottle calibration."""
        self._send_action_prompt(
            "Load cell not calibrated",
            ["The nozzle touch sensor (load cell) has not been calibrated,",
             "so the Z offset cannot be measured safely yet.",
             "Calibrate it now? It takes about 2 minutes. You need a 1.5 L",
             "bottle of water and a kitchen scale. Nothing moves or heats."],
            [("Calibrate load cell", "NEBULAOS_LOAD_CELL_CALIBRATE"),
             ("Use automatic", "NEBULAOS_LOAD_CELL_MODE MODE=auto"),
             ("Close", "RESPOND TYPE=command MSG=action:prompt_end")])

    def _offer_manual_load_cell(self, reason):
        """G99: the automatic load cell check failed (or automatic contact
        is not ready): say why and offer Manual (the bottle calibration)."""
        self._send_action_prompt(
            "Automatic load cell check failed",
            ["The automatic nozzle touch sensor check did not pass:",
             reason,
             "Remove everything from the bed and try again, or use",
             "Calibration > Load Cell > Manual (a 1.5 L bottle of water",
             "and a kitchen scale, about 2 minutes)."],
            [("Manual calibration", "NEBULAOS_LOAD_CELL_CALIBRATE"),
             ("Close", "RESPOND TYPE=command MSG=action:prompt_end")])

    def _verify_expected_values(self, expected):
        """Checks a journal's expected_values dict against the real,
        just-reloaded printer config/state - shared by every workflow's
        post-restart verification in _handle_ready() below, since each
        workflow only ever populates the small subset of keys it knows
        about and this method simply ignores keys that aren't present."""
        errors = []
        expected_z_offset = expected.get('bltouch.z_offset')
        if expected_z_offset is not None:
            probe_obj = self.printer.lookup_object('probe', None)
            if probe_obj is None:
                errors.append('probe object is not configured')
            else:
                actual = probe_obj.get_offsets()[2]
                if abs(actual - expected_z_offset) > 0.0005:
                    errors.append(
                        'bltouch.z_offset: expected %.5f, found %.5f'
                        % (expected_z_offset, actual))
        expected_profile = expected.get('bed_mesh.profile')
        if expected_profile is not None:
            # Checks that the profile was SAVED (present in configfile's
            # own per-profile section, reflected in bed_mesh's own
            # in-memory `profiles` dict on every boot regardless of
            # whether it's active), not that it's the currently ACTIVE
            # mesh - upstream bed_mesh does not auto-load any saved
            # profile as active at klippy startup on its own (that is a
            # print-time step, e.g. a START_PRINT macro's own explicit
            # BED_MESH_PROFILE LOAD=<name>). Root-caused live
            # 2026-09-03: a real AUTO_CALIBRATE run saved a genuine 9x9
            # mesh under 'default' (confirmed present in both
            # configfile.settings.bed_mesh and bed_mesh's own `profiles`
            # status field after restart) but bed_mesh's own
            # profile_name/mesh_matrix stayed empty since nothing had
            # loaded it yet - the original check here compared against
            # profile_name and produced a false-negative "verification
            # failed" for an actually-correct save.
            bed_mesh = self.printer.lookup_object('bed_mesh', None)
            profiles = (
                bed_mesh.get_status(self.reactor.monotonic()).get('profiles')
                if bed_mesh is not None else None) or {}
            if expected_profile not in profiles:
                errors.append(
                    'bed_mesh.profile: expected saved profile %r, found '
                    'saved profiles %r'
                    % (expected_profile, sorted(profiles.keys())))
        input_shaper_keys = (
            'input_shaper.shaper_type_x', 'input_shaper.shaper_freq_x',
            'input_shaper.shaper_type_y', 'input_shaper.shaper_freq_y')
        if any(k in expected for k in input_shaper_keys):
            input_shaper_obj = self.printer.lookup_object(
                'input_shaper', None)
            shapers_by_axis = (
                {s.axis: s for s in input_shaper_obj.get_shapers()}
                if input_shaper_obj is not None else {})
            for axis in ('x', 'y'):
                expected_type = expected.get(
                    'input_shaper.shaper_type_%s' % (axis,))
                expected_freq = expected.get(
                    'input_shaper.shaper_freq_%s' % (axis,))
                if expected_type is None and expected_freq is None:
                    continue
                shaper = shapers_by_axis.get(axis)
                if shaper is None:
                    errors.append(
                        'input_shaper axis %s is not configured' % (axis,))
                    continue
                if (expected_type is not None
                        and shaper.params.shaper_type != expected_type):
                    errors.append(
                        'input_shaper.shaper_type_%s: expected %r, found %r'
                        % (axis, expected_type, shaper.params.shaper_type))
                if (expected_freq is not None
                        and abs(shaper.params.shaper_freq - expected_freq)
                        > 0.05):
                    errors.append(
                        'input_shaper.shaper_freq_%s: expected %.3f, '
                        'found %.3f'
                        % (axis, expected_freq, shaper.params.shaper_freq))
        return errors

    def _verify_journal_at_path(self, path):
        """Reads one on-disk journal and, if it shows a commit that was
        requested but never verified, checks it and writes the result
        back - returns the (possibly updated) journal dict, or None if
        there was nothing to verify. Shared body of _handle_ready() below,
        called once per known journal file (auto_calibrate's own
        self.journal_path, input_shaper's own self.input_shaper_journal_
        path, ...) since each guided workflow keeps a separate journal
        file on purpose (see input_shaper_journal_path's own __init__
        comment)."""
        journal = calibration_journal.read_journal(path=path)
        if journal is None or not journal.get('verification_pending'):
            return None
        expected = journal.get('expected_values') or {}
        errors = self._verify_expected_values(expected)
        now = time.time()
        if errors:
            calibration_journal.mark_verification_result(
                journal, success=False, result=None,
                error='; '.join(errors), now=now)
        else:
            calibration_journal.mark_verification_result(
                journal, success=True, result=dict(expected), error=None,
                now=now)
        calibration_journal.write_journal(journal, path=path)
        return journal

    def _handle_ready(self, *args):
        """klippy:ready - runs on EVERY klippy startup, including the one
        immediately after NEBULAOS_AUTO_CALIBRATE's (or NEBULAOS_INPUT_
        SHAPER_CALIBRATE's) own SAVE_CONFIG restart. Reads each guided
        workflow's own on-disk journal (never the in-memory status, which
        is always fresh/empty in a new process) and, for any journal that
        shows a commit that was requested but never verified, checks the
        real, just-reloaded config against what was expected and records
        the result - the only code path that ever clears
        verification_pending. A journal showing anything else (no file, or
        an already-verified/errored/cancelled state) is left untouched -
        this only ever acts on a genuine "restart just happened,
        verification still owed" case."""
        journal = self._verify_journal_at_path(self.journal_path)
        if journal is not None and (
                journal['calibration_id'] == self.auto_calibrate_id
                or self.auto_calibrate_id == 0):
            self.auto_calibrate_id = journal['calibration_id']
            self.auto_calibrate_state = journal['state']
            self.auto_calibrate_stage = journal['stage']
            self.auto_calibrate_error = journal['error']
            self.auto_calibrate_result = journal['result']
        if journal is not None:
            self._publish_verified('auto_calibrate', journal,
                                   AUTO_CALIBRATE_STEPS)

        input_shaper_journal = self._verify_journal_at_path(
            self.input_shaper_journal_path)
        if input_shaper_journal is not None and (
                input_shaper_journal['calibration_id'] == self.input_shaper_id
                or self.input_shaper_id == 0):
            self.input_shaper_id = input_shaper_journal['calibration_id']
            self.input_shaper_state = input_shaper_journal['state']
            self.input_shaper_stage = input_shaper_journal['stage']
            self.input_shaper_error = input_shaper_journal['error']
            self.input_shaper_result = input_shaper_journal['result']
        if input_shaper_journal is not None:
            self._publish_verified('input_shaper', input_shaper_journal,
                                   INPUT_SHAPER_STEPS)
        # G109: the last run's per-axis results + graphs (end screen, and the
        # idle page later) survive the SAVE_CONFIG restart in the journal
        if not self.input_shaper_axes:
            last = input_shaper_journal or calibration_journal.read_journal(
                path=self.input_shaper_journal_path)
            axes = (last.get('axes') if isinstance(last, dict) and last.get(
                'state') == calibration_journal.STATE_COMPLETE else None)
            if isinstance(axes, dict):
                self.input_shaper_axes = dict(
                    (a, i) for a, i in axes.items()
                    if a in ('x', 'y') and isinstance(i, dict))
        if self._history_warning:
            self.gcode.respond_info("nebulaos_calibration: %s"
                                    % (self._history_warning,))
            self._history_warning = None

    def _publish_verified(self, flow, journal, steps):
        """G104/G105: a commit was just verified after its restart - the
        end screen (last_result) and the history; a mismatch is a failure
        at the last step (Verifying)."""
        summary = [s for s in (journal.get('summary') or [])
                   if isinstance(s, str)]
        if journal.get('state') == calibration_journal.STATE_COMPLETE:
            history_summary = journal.get('history_summary')
            if not isinstance(history_summary, str):
                history_summary = "; ".join(summary)
            self.record_flow(flow, history_summary, save=False)
            self._set_last_result(flow, True, summary)
        else:
            self._set_last_result(flow, False, summary, steps=steps,
                                  step=len(steps),
                                  error=journal.get('error') or '')

    # ------------------------------------------------------------------
    # NEBULAOS_CALIBRATION_STATUS
    # ------------------------------------------------------------------
    cmd_calibration_status_help = "Report the current NebulaOS calibration status"

    def cmd_calibration_status(self, gcmd):
        status = self.get_status(self.reactor.monotonic())
        gcmd.respond_info(
            "NEBULAOS_CALIBRATION_STATUS: z_offset_state=%s z_offset_id=%s "
            "z_offset_result=%s z_offset_error=%s"
            % (status['z_offset_state'], status['z_offset_id'],
               status['z_offset_result'], status['z_offset_error']))
        gcmd.respond_info(
            "NEBULAOS_CALIBRATION_STATUS: auto_calibrate_state=%s "
            "auto_calibrate_stage=%s auto_calibrate_error=%s"
            % (status['auto_calibrate_state'], status['auto_calibrate_stage'],
               status['auto_calibrate_error']))
        gcmd.respond_info(
            "NEBULAOS_CALIBRATION_STATUS: input_shaper_state=%s "
            "input_shaper_stage=%s input_shaper_error=%s"
            % (status['input_shaper_state'], status['input_shaper_stage'],
               status['input_shaper_error']))


def load_config(config):
    return NebulaOSCalibration(config)
