# nebulaos_power_loss_recovery.py - NebulaOS's own power-loss recovery
# extra (Phase 1.9B), built entirely on public/mainline Klipper interfaces.
#
# Persistence has two tiers:
#   - A tiny (16-byte) durable record on the physical BL24C16F EEPROM, owned
#     in production by the Linux 6.6 in-tree at24/nvmem driver (see
#     nebulaos_plr_journal.py's own header for the full physical layout and
#     why [bl24c16f]/i2c-chardev are NOT used here) - generation counter +
#     file_position + a CRC, durable across genuine power loss because it is
#     the only thing this module trusts as ground truth for "did a
#     checkpoint really land".
#   - A larger, human-readable JSON "sidecar" file on /usr/data (dual A/B
#     files, selected by the EEPROM generation's parity) holding the full
#     recoverable gcode/motion/thermal/etc state. The EEPROM record is
#     written LAST, only after the sidecar write is fully durable (fsync'd
#     file, atomically renamed, fsync'd parent directory) - see
#     _perform_checkpoint_blocking() for the exact ordering this enforces.
#
# No Klipper core patches. Every field this module reads comes from a public
# get_status() dict and every field it restores is set back through a
# standard gcode command - with ONE exception, the bed mesh (C3): the print's
# adaptive mesh is never saved as a profile, so BED_MESH_PROFILE LOAD cannot
# bring it back. Its probed points and parameters are recorded
# (BedMesh.get_mesh().get_mesh_params(), not in get_status) and rebuilt on
# resume exactly as upstream's own profile load does (ZMesh + build_mesh +
# BedMesh.set_mesh). These symbols are in nebulaos-extensions.json's
# required_klipper_symbols, so API drift refuses to start instead of
# breaking a resume. See docs/ (added alongside this mission) for the full
# design rationale.
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from __future__ import annotations

import hashlib
import json
import logging
import os

from . import nebulaos_plr_journal as journal

DEFAULT_EEPROM_PATH = "/sys/bus/i2c/devices/2-0050/eeprom"
DEFAULT_SIDECAR_DIR = "/usr/data/nebulaos/printer_data/plr"
DEFAULT_CHECKPOINT_INTERVAL = 5.0
DEFAULT_POLL_INTERVAL = 0.75
DEFAULT_MIN_Z_FOR_START = 0.6

# Schema 2 (2026-10-09): adds "identity" (printer configuration hash) and
# "toolhead" (kinematic position) - both needed by the physical resume. A
# schema-1 record from an older build is reported incompatible, never
# resumed.
# Schema 3 (2026-10-10, decision D6): adds identity.firmware_sha (the NebulaOS
# image the record was written under, from [nebulaos_version]). A record is
# never resumed under different firmware; there is no age limit.
SCHEMA_VERSION = 3

DEFAULT_RESUME_Z_LIFT = 5.0
DEFAULT_PRIME_LENGTH = 8.0

SIDECAR_FILENAMES = ("state-a.json", "state-b.json")

_HASH_CHUNK_SIZE = 1024 * 1024

# Structured status for front-ends (get_status()["state"]). A front-end
# presents these; it never reads the EEPROM or the sidecar files itself.
STATE_NONE = "NONE"
STATE_RECORDING = "RECORDING"
STATE_RECOVERY_AVAILABLE = "RECOVERY_AVAILABLE"
STATE_RECOVERY_INCOMPATIBLE = "RECOVERY_INCOMPATIBLE"
STATE_RESUME_PREPARED = "RESUME_PREPARED"
STATE_ERROR = "ERROR"

# print_stats states in which NEBULAOS_PLR_RESUME may run: nothing is
# printing or paused, i.e. the state Klipper is in after a power loss.
RESUME_ALLOWED_PRINT_STATES = ("standby", "complete", "cancelled", "error")


# ---------------------------------------------------------------------------
# Pure logic - no Klipper objects, no I/O side effects beyond what's passed
# in explicitly. Every function here is exercised directly by
# test_nebulaos_power_loss_recovery.py with plain dicts/tmp files, with zero
# printer/reactor/hardware involved.
# ---------------------------------------------------------------------------

def meaningful_progress(print_stats_status, virtual_sdcard_status,
                         toolhead_status, min_z):
    """Gate for starting persistent checkpointing at all - deliberately NOT
    Creality's layer-comment/G1-count approach, just simple, public,
    already-available status fields."""
    if print_stats_status.get('state') != 'printing':
        return False
    if not virtual_sdcard_status.get('is_active'):
        return False
    if virtual_sdcard_status.get('file_position', 0) <= 0:
        return False
    if print_stats_status.get('print_duration', 0) <= 0:
        return False
    position = toolhead_status.get('position')
    z = position.z if position is not None else 0.0
    if z <= min_z:
        return False
    return True


def sidecar_path_for_generation(sidecar_dir, generation):
    index = journal.sidecar_parity(generation)
    return os.path.join(sidecar_dir, SIDECAR_FILENAMES[index])


def coord_to_list(coord):
    """Converts a Klipper gcode_move `Coord` (a namedtuple-like object with
    .x/.y/.z/.e) into a plain JSON-serializable [x, y, z, e] list."""
    return [coord.x, coord.y, coord.z, coord.e]


def config_identity(raw_config):
    """SHA-256 over Klipper's parsed raw configuration (configfile status
    'config': every section and option as loaded, includes and the
    SAVE_CONFIG block included). Any change to printer.cfg that Klipper
    actually reads - Z offset, mesh, rotation distance, kinematics - changes
    it."""
    payload = json.dumps(raw_config or {}, sort_keys=True, default=str)
    return hashlib.sha256(payload.encode('utf-8')).hexdigest()


def build_sidecar_state(generation, file_info, file_position,
                         gcode_move_status, toolhead_status,
                         extruder_status, bed_mesh_status,
                         exclude_object_status, fan_status,
                         firmware_retraction_status=None,
                         config_sha256=None, firmware_sha=None):
    """Pure assembly of the "Sidecar V1 State" schema from already-fetched
    get_status() dicts (or plain equivalents in tests). Never touches
    gcode_move.base_position or any other private attribute - everything
    here is a documented public get_status() key."""
    state = {
        "schema_version": SCHEMA_VERSION,
        "generation": generation,
        "file": {
            "path": file_info["path"],
            "size": file_info["size"],
            "sha256": file_info["sha256"],
        },
        "file_position": file_position,
        "gcode": {
            "gcode_position": coord_to_list(gcode_move_status["gcode_position"]),
            "absolute_coordinates": gcode_move_status["absolute_coordinates"],
            "absolute_extrude": gcode_move_status["absolute_extrude"],
            "speed_factor": gcode_move_status["speed_factor"],
            "extrude_factor": gcode_move_status["extrude_factor"],
            "homing_origin": coord_to_list(gcode_move_status["homing_origin"]),
        },
        "toolhead": {
            "position": (coord_to_list(toolhead_status["position"])
                         if toolhead_status.get("position") is not None else None),
        },
        "identity": {
            "config_sha256": config_sha256,
            "firmware_sha": firmware_sha,
        },
        "motion": {
            "max_velocity": toolhead_status["max_velocity"],
            "max_accel": toolhead_status["max_accel"],
            "minimum_cruise_ratio": toolhead_status["minimum_cruise_ratio"],
            "square_corner_velocity": toolhead_status["square_corner_velocity"],
        },
        "thermal": {
            "extruder_target": extruder_status.get("target", 0.0),
            "bed_target": extruder_status.get("bed_target", 0.0),
        },
        "fan": {
            "speed": fan_status.get("speed", 0.0),
        },
        "extruder": {
            "pressure_advance": extruder_status.get("pressure_advance", 0.0),
            "smooth_time": extruder_status.get("smooth_time", 0.0),
        },
        "bed_mesh": mesh_state(bed_mesh_status),
        "exclude_object": {
            "objects": exclude_object_status.get("objects", []),
            "excluded_objects": exclude_object_status.get("excluded_objects", []),
        },
    }
    if firmware_retraction_status is not None:
        state["firmware_retraction"] = {
            "retract_length": firmware_retraction_status.get("retract_length", 0.0),
            "retract_speed": firmware_retraction_status.get("retract_speed", 0.0),
            "unretract_extra_length":
                firmware_retraction_status.get("unretract_extra_length", 0.0),
            "unretract_speed": firmware_retraction_status.get("unretract_speed", 0.0),
        }
    return state


def mesh_state(bed_mesh_status):
    """The active mesh: its profile name and (when the caller added
    "mesh_params", see _take_candidate) the mesh itself - probed points +
    parameters, exactly what a saved profile holds. C3: the print's adaptive
    mesh ("adaptive") is never saved, and a saved profile of the same name
    may differ from the one in use (unsaved BED_MESH_CALIBRATE), so the
    recorded mesh, not the name, is what a resume restores."""
    state = {"profile_name": bed_mesh_status.get("profile_name", "")}
    points = bed_mesh_status.get("probed_matrix")
    params = bed_mesh_status.get("mesh_params")
    if (state["profile_name"] and params and points
            and any(len(row) for row in points)):
        state["points"] = [list(row) for row in points]
        state["mesh_params"] = dict(params)
    return state


class MeshNotRestorable(Exception):
    def __init__(self, reason):
        super(MeshNotRestorable, self).__init__(reason)
        self.reason = reason


def mesh_restore_line(bed_mesh_state, saved_mesh_profiles=None):
    """The one G-code line that brings the checkpoint's mesh back:
    no mesh -> BED_MESH_CLEAR; recorded mesh -> _NEBULAOS_PLR_LOAD_MESH (the
    exact mesh in use); otherwise a saved profile of that name (an older
    sidecar without the recorded mesh). Never a different mesh in place of
    the one the print used: raises MeshNotRestorable instead.
    saved_mesh_profiles None = not checked (pure callers/tests)."""
    bed_mesh_state = bed_mesh_state or {}
    name = bed_mesh_state.get("profile_name")
    if not name:
        return "BED_MESH_CLEAR"
    if bed_mesh_state.get("points") and bed_mesh_state.get("mesh_params"):
        return "_NEBULAOS_PLR_LOAD_MESH"
    if saved_mesh_profiles is not None and name not in saved_mesh_profiles:
        raise MeshNotRestorable(
            "bed mesh '%s' was not saved and was not recorded; it cannot be "
            "reloaded" % (name,))
    return "BED_MESH_PROFILE LOAD=%s" % (name,)


def hash_file(path, chunk_size=_HASH_CHUNK_SIZE):
    digest = hashlib.sha256()
    with open(path, 'rb') as handle:
        while True:
            chunk = handle.read(chunk_size)
            if not chunk:
                break
            digest.update(chunk)
    return digest.hexdigest()


def atomic_write_json(tmp_path, final_path, data):
    """Checkpoint transaction steps 2-5, exactly as specified:
    write .tmp, flush + fsync(file), os.replace (atomic rename),
    fsync(parent directory). Runs on a background thread via aio_executor -
    this function itself is plain blocking I/O, deliberately."""
    payload = json.dumps(data, sort_keys=True).encode('utf-8')
    with open(tmp_path, 'wb') as handle:
        handle.write(payload)
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(tmp_path, final_path)
    parent = os.path.dirname(final_path) or "."
    parent_fd = os.open(parent, os.O_RDONLY)
    try:
        os.fsync(parent_fd)
    finally:
        os.close(parent_fd)


def read_sidecar(path):
    """Returns the parsed sidecar dict, or None if the file is missing or
    torn (unparseable) - both are treated identically by callers (no
    recovery available from this slot), never raised as an error."""
    try:
        with open(path, 'rb') as handle:
            payload = handle.read()
    except (IOError, OSError):
        return None
    try:
        return json.loads(payload.decode('utf-8'))
    except (ValueError, UnicodeDecodeError):
        return None


class IntegrityFailure(Exception):
    """Raised by validate_recovery() for any integrity failure. ALLOW_UNSAFE
    may NEVER suppress this - it only ever exists to report a reason
    string, and validate_recovery() never even looks at ALLOW_UNSAFE except
    for the one dedicated position-safety check."""
    def __init__(self, reason):
        super(IntegrityFailure, self).__init__(reason)
        self.reason = reason


def validate_recovery(eeprom_record, sidecar_state, current_file_path,
                       current_file_size, current_file_sha256,
                       allow_unsafe, current_config_sha256=None,
                       current_firmware_sha=None):
    """Runs the full validation chain from the mission's File Identity and
    Physical Position Policy sections, in order, and returns the validated
    sidecar_state dict on success.

    Raises IntegrityFailure for any of: missing/tombstoned EEPROM state,
    missing/corrupt sidecar, schema mismatch, stale sidecar generation,
    missing gcode file path, file size/sha256/path mismatch. These can
    NEVER be bypassed by allow_unsafe.

    Raises PositionUnsafe (a distinct exception) ONLY for the physical
    Z-position-uncertainty gate, which allow_unsafe=True is permitted to
    override - and only that gate.
    """
    if eeprom_record is None:
        raise IntegrityFailure("no EEPROM checkpoint available")
    if sidecar_state is None:
        raise IntegrityFailure("sidecar missing or unreadable (torn/absent)")
    if sidecar_state.get("schema_version") != SCHEMA_VERSION:
        raise IntegrityFailure("sidecar schema_version mismatch")
    if sidecar_state.get("generation") != eeprom_record.generation:
        raise IntegrityFailure(
            "stale sidecar generation (sidecar=%r, eeprom=%r)"
            % (sidecar_state.get("generation"), eeprom_record.generation))

    file_info = sidecar_state.get("file") or {}
    saved_path = file_info.get("path")
    saved_size = file_info.get("size")
    saved_sha256 = file_info.get("sha256")
    if not saved_path:
        raise IntegrityFailure("sidecar has no recorded gcode file path")
    if (saved_path != current_file_path or saved_size != current_file_size
            or saved_sha256 != current_file_sha256):
        raise IntegrityFailure(
            "file identity mismatch (path/size/sha256 changed since checkpoint)")
    recorded_config = (sidecar_state.get("identity") or {}).get("config_sha256")
    if recorded_config != current_config_sha256:
        raise IntegrityFailure(
            "printer configuration changed since the checkpoint (Z offset, "
            "mesh, kinematics or other printer.cfg settings may differ)")
    recorded_firmware = (sidecar_state.get("identity") or {}).get("firmware_sha")
    if recorded_firmware != current_firmware_sha:
        raise IntegrityFailure(
            "NebulaOS firmware changed since the checkpoint (recorded %s, "
            "running %s)" % (recorded_firmware, current_firmware_sha))

    # Every integrity gate above is unconditional. Only the physical
    # position-safety gate below is subject to allow_unsafe, and it is
    # checked LAST, deliberately, so a caller can never short-circuit past
    # an integrity failure by also passing allow_unsafe=True.
    if not allow_unsafe:
        raise PositionUnsafe(
            "physical Z position cannot be proven safe after power loss; "
            "resume refused (NEBULAOS_PLR_RESUME ALLOW_UNSAFE=1 to override "
            "this specific check only)")

    return sidecar_state


class PositionUnsafe(Exception):
    def __init__(self, reason):
        super(PositionUnsafe, self).__init__(reason)
        self.reason = reason


def light_recovery_check(eeprom_record, sidecar_state, resolved_file_path,
                         current_firmware_sha=None):
    """The cheap, boot-time half of validate_recovery(): everything except
    hashing the gcode file, which can take seconds on this SoC and is
    therefore left to NEBULAOS_PLR_RESUME. Returns (state, reason) where
    state is STATE_NONE, STATE_RECOVERY_AVAILABLE or
    STATE_RECOVERY_INCOMPATIBLE. Pure: the caller does all I/O."""
    if eeprom_record is None:
        return STATE_NONE, ""
    if sidecar_state is None:
        return STATE_RECOVERY_INCOMPATIBLE, "sidecar missing or unreadable"
    if sidecar_state.get("schema_version") != SCHEMA_VERSION:
        return STATE_RECOVERY_INCOMPATIBLE, "sidecar schema_version mismatch"
    if sidecar_state.get("generation") != eeprom_record.generation:
        return STATE_RECOVERY_INCOMPATIBLE, "stale sidecar generation"
    file_info = sidecar_state.get("file") or {}
    if not file_info.get("path"):
        return STATE_RECOVERY_INCOMPATIBLE, "no recorded gcode file path"
    if not resolved_file_path or not os.path.isfile(resolved_file_path):
        return STATE_RECOVERY_INCOMPATIBLE, "gcode file missing"
    if os.path.getsize(resolved_file_path) != file_info.get("size"):
        return STATE_RECOVERY_INCOMPATIBLE, "gcode file size changed"
    if ((sidecar_state.get("identity") or {}).get("firmware_sha")
            != current_firmware_sha):
        return STATE_RECOVERY_INCOMPATIBLE, "NebulaOS firmware changed since the checkpoint"
    return STATE_RECOVERY_AVAILABLE, ""


def build_resume_gcode_lines(state, saved_mesh_profiles=None):
    """Builds the standard-command restoration sequence from the mission's
    "Public Klipper Resume Path" section. Pure string assembly - no gcode
    is actually run here, the caller feeds this list to
    run_script_from_command().

    Deliberately excludes M24 and any G28/motion command: this mission
    does not implement or hardware-qualify physical position recovery, so
    the toolhead is never moved and printing is never resumed
    automatically by this function - see the module docstring and the
    mission's own "Physical Position Policy" section. The caller is
    responsible for M24 as a clearly separate, later, manual step once a
    real physical-position-recovery design exists.
    """
    gcode_state = state["gcode"]
    motion = state["motion"]
    thermal = state["thermal"]
    extruder = state["extruder"]
    fan = state["fan"]
    bed_mesh = state["bed_mesh"]
    file_info = state["file"]

    lines = []
    lines.append("M23 %s" % file_info["path"])

    for name in state["exclude_object"].get("objects", []):
        lines.append("EXCLUDE_OBJECT_DEFINE NAME=%s" % name)
    for name in state["exclude_object"].get("excluded_objects", []):
        lines.append("EXCLUDE_OBJECT NAME=%s" % name)

    lines.append("M26 S%d" % state["file_position"])

    lines.append("G90" if gcode_state["absolute_coordinates"] else "G91")
    lines.append("M82" if gcode_state["absolute_extrude"] else "M83")
    lines.append("M220 S%g" % (gcode_state["speed_factor"] * 100.0))
    lines.append("M221 S%g" % (gcode_state["extrude_factor"] * 100.0))

    homing_origin = gcode_state["homing_origin"]
    lines.append("SET_GCODE_OFFSET Z=%g MOVE=0" % homing_origin[2])
    lines.append("G92 E%g" % gcode_state["gcode_position"][3])

    lines.append(
        "SET_VELOCITY_LIMIT VELOCITY=%g ACCEL=%g "
        "MINIMUM_CRUISE_RATIO=%g SQUARE_CORNER_VELOCITY=%g"
        % (motion["max_velocity"], motion["max_accel"],
           motion["minimum_cruise_ratio"], motion["square_corner_velocity"]))
    lines.append("SET_PRESSURE_ADVANCE ADVANCE=%g SMOOTH_TIME=%g"
                  % (extruder["pressure_advance"], extruder["smooth_time"]))

    fan_speed = fan.get("speed", 0.0)
    if fan_speed > 0:
        lines.append("M106 S%d" % round(fan_speed * 255))
    else:
        lines.append("M107")

    lines.append(mesh_restore_line(bed_mesh, saved_mesh_profiles))

    retraction = state.get("firmware_retraction")
    if retraction:
        lines.append(
            "SET_RETRACTION RETRACT_LENGTH=%g RETRACT_SPEED=%g "
            "UNRETRACT_EXTRA_LENGTH=%g UNRETRACT_SPEED=%g"
            % (retraction["retract_length"], retraction["retract_speed"],
               retraction["unretract_extra_length"],
               retraction["unretract_speed"]))

    # Restoring heater targets is the one thermal action this mission
    # performs automatically, and only here - after an explicit,
    # human-issued NEBULAOS_PLR_RESUME command (never at boot, never
    # automatically). It is not a physical-position action.
    if thermal.get("extruder_target", 0.0) > 0:
        lines.append("M104 S%g" % thermal["extruder_target"])
    if thermal.get("bed_target", 0.0) > 0:
        lines.append("M140 S%g" % thermal["bed_target"])

    return lines


class PhysicalResumeRefused(Exception):
    def __init__(self, reason):
        super(PhysicalResumeRefused, self).__init__(reason)
        self.reason = reason


def build_physical_resume_lines(state, z_lift, prime_length, z_max,
                                saved_mesh_profiles):
    """The physical resume (opt-in, [nebulaos_power_loss_recovery]
    physical_resume: True; NOT hardware-qualified until the supervised
    power-cut test). Same idea as Creality's stock resume - home X/Y only,
    trust that Z did not move while unpowered - with these differences:
    the bed is heated first (keeps the part attached), Z is declared at its
    recorded kinematic height and LIFTED before any XY move (stock moves XY
    at print height), the nozzle is heated and primed at the X/Y endstop
    park position off the bed, and the configuration identity was already
    proven equal by validate_recovery().

    Ordering constraint from upstream safe_z_home: G28 with Z unhomed resets
    Z to 0 and clears its homing - so Z is declared homed at its recorded
    height BEFORE G28 X Y; with Z homed above z_hop, safe_z_home leaves Z
    alone. Z is declared through this extra's internal _NEBULAOS_PLR_SET_Z
    (toolhead.set_position(..., homing_axes="z"), what force_move's
    SET_KINEMATIC_POSITION does): upstream registers SET_KINEMATIC_POSITION
    only with [force_move] enable_force_move, which NebulaOS does not enable
    for everyone.

    Returns the full G-code list, ending with M24. Raises
    PhysicalResumeRefused when it cannot be done safely."""
    toolhead_pos = (state.get("toolhead") or {}).get("position")
    if not toolhead_pos:
        raise PhysicalResumeRefused("no recorded toolhead position")
    kin_z = float(toolhead_pos[2])
    if kin_z <= 0:
        raise PhysicalResumeRefused("recorded Z height is not above the bed")
    lift = min(float(z_lift), float(z_max) - kin_z - 1.0)
    if lift < 1.0:
        raise PhysicalResumeRefused(
            "no room to lift above the part (Z %.2f of %.2f)" % (kin_z, z_max))
    try:
        mesh_restore_line(state.get("bed_mesh"), saved_mesh_profiles or ())
    except MeshNotRestorable as exc:
        raise PhysicalResumeRefused(exc.reason)
    thermal = state["thermal"]
    bed = thermal.get("bed_target", 0.0)
    hotend = thermal.get("extruder_target", 0.0)
    if hotend <= 0:
        raise PhysicalResumeRefused("no recorded nozzle temperature")
    gcode_state = state["gcode"]
    gx, gy, gz, ge = gcode_state["gcode_position"]

    lines = []
    if bed > 0:
        lines.append("M140 S%g" % bed)
    lines.append("M104 S%g" % hotend)
    if bed > 0:
        lines.append("M190 S%g" % bed)
    lines.append("_NEBULAOS_PLR_SET_Z Z=%.4f" % kin_z)
    lines.append("G91")
    lines.append("G1 Z%.3f F600" % lift)
    lines.append("G90")
    lines.append("G28 X Y")
    lines.append("M109 S%g" % hotend)
    # State restore without the parts the procedure sets itself (heaters
    # done above; coordinate modes and E are set after the moves).
    skip = ("G90", "G91", "M82", "M83", "M104 ", "M140 ", "G92 E")
    for line in build_resume_gcode_lines(state, saved_mesh_profiles or ()):
        if line in skip or line.startswith(skip):
            continue
        lines.append(line)
    lines.append("M83")
    lines.append("G1 E%.3f F300" % float(prime_length))
    lines.append("G90")
    lines.append("G1 X%.4f Y%.4f F3000" % (gx, gy))
    lines.append("G1 Z%.4f F600" % gz)
    lines.append("G92 E%g" % ge)
    lines.append("G90" if gcode_state["absolute_coordinates"] else "G91")
    lines.append("M82" if gcode_state["absolute_extrude"] else "M83")
    lines.append("M24")
    return lines


# ---------------------------------------------------------------------------
# Klipper glue.
# ---------------------------------------------------------------------------

class Candidate(object):
    """A snapshot of PLR state taken at a specific point in the gcode
    stream, not yet known to be durable. See the module-level "Checkpoint
    execution semantics" note above _tick_printing() for why this exists -
    in short, virtual_sdcard.file_position only proves motion has been
    QUEUED, not physically executed, so every checkpoint must pass through
    this pending stage before it is trusted."""
    __slots__ = ('file_position', 'file_info', 'gm_status', 'th_status',
                 'extruder_status', 'bed_mesh_status',
                 'exclude_object_status', 'fan_status', 'fr_status',
                 'taken_at', 'target_print_time')

    def __init__(self, file_position, file_info, gm_status, th_status,
                 extruder_status, bed_mesh_status, exclude_object_status,
                 fan_status, fr_status, taken_at):
        self.file_position = file_position
        self.file_info = file_info
        self.gm_status = gm_status
        self.th_status = th_status
        self.extruder_status = extruder_status
        self.bed_mesh_status = bed_mesh_status
        self.exclude_object_status = exclude_object_status
        self.fan_status = fan_status
        self.fr_status = fr_status
        self.taken_at = taken_at
        self.target_print_time = None  # set once toolhead confirms timing


class NebulaOSPowerLossRecovery:
    def __init__(self, config):
        self.config = config
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')

        self.eeprom_path = config.get('eeprom_path', DEFAULT_EEPROM_PATH)
        self.sidecar_dir = config.get('sidecar_dir', DEFAULT_SIDECAR_DIR)
        self.checkpoint_interval = config.getfloat(
            'checkpoint_interval', DEFAULT_CHECKPOINT_INTERVAL, above=0.)
        self.poll_interval = config.getfloat(
            'poll_interval', DEFAULT_POLL_INTERVAL, above=0.)
        self.min_z_for_start = config.getfloat(
            'min_z_for_start', DEFAULT_MIN_Z_FOR_START, minval=0.)
        # Physical resume (home X/Y, trust Z, lift, reheat, prime, return,
        # M24) is OFF until it has passed a supervised power-cut test print
        # on this machine. Off: NEBULAOS_PLR_RESUME restores state only.
        self.physical_resume = config.getboolean('physical_resume', False)
        self.resume_z_lift = config.getfloat(
            'resume_z_lift', DEFAULT_RESUME_Z_LIFT, minval=1., maxval=20.)
        self.prime_length = config.getfloat(
            'prime_length', DEFAULT_PRIME_LENGTH, minval=0., maxval=50.)

        self.executor = None
        self.timer_handler = None

        self._session_file_info = None  # {"path","size","sha256"} once known
        self._session_config_sha256 = None
        self._session_firmware_sha = None
        self._last_checkpoint_time = 0.0
        self._checkpoint_in_flight = False
        self._resume_in_progress = False
        self._resume_mesh = {}
        self._have_active_session = False
        self._pending_candidate = None  # a Candidate not yet proven durable
        # Set on the first 'printing' tick of a print (see _check_new_print);
        # cleared once the print is over.
        self._print_start_checked = False
        # (generation, relative gcode path) restored by NEBULAOS_PLR_RESUME
        # and waiting for the operator's M24 - the one record a starting
        # print must not tombstone.
        self._resume_armed = None

        # Structured status, cached: get_status() is polled by Moonraker
        # subscribers and must never do I/O.
        self._status_state = STATE_NONE
        self._recovery_info = {}
        self._last_action = ""
        self._last_error = ""

        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        self.printer.register_event_handler("virtual_sdcard:reset_file",
                                             self._handle_reset_file)

        self.gcode.register_command(
            "NEBULAOS_PLR_STATUS", self.cmd_NEBULAOS_PLR_STATUS,
            desc="Report NebulaOS power-loss-recovery status")
        self.gcode.register_command(
            "NEBULAOS_PLR_RESUME", self.cmd_NEBULAOS_PLR_RESUME,
            desc="Resume a NebulaOS power-loss-recovery checkpoint")
        self.gcode.register_command(
            "NEBULAOS_PLR_DISCARD", self.cmd_NEBULAOS_PLR_DISCARD,
            desc="Discard any NebulaOS power-loss-recovery checkpoint")
        self.gcode.register_command(
            "_NEBULAOS_PLR_SET_Z", self.cmd_NEBULAOS_PLR_SET_Z,
            desc="Internal to NEBULAOS_PLR_RESUME PHYSICAL=1")
        self.gcode.register_command(
            "_NEBULAOS_PLR_LOAD_MESH", self.cmd_NEBULAOS_PLR_LOAD_MESH,
            desc="Internal to NEBULAOS_PLR_RESUME")

    # -- setup -------------------------------------------------------------

    def _handle_ready(self):
        self.gcode_move = self.printer.lookup_object('gcode_move')
        self.toolhead = self.printer.lookup_object('toolhead')
        self.mcu = self.printer.lookup_object('mcu')
        self.print_stats = self.printer.lookup_object('print_stats')
        self.virtual_sdcard = self.printer.lookup_object('virtual_sdcard')
        self.exclude_object = self.printer.lookup_object('exclude_object', None)
        self.bed_mesh = self.printer.lookup_object('bed_mesh', None)
        self.extruder = self.printer.lookup_object('extruder', None)
        self.fan = self.printer.lookup_object('fan', None)
        self.firmware_retraction = self.printer.lookup_object(
            'firmware_retraction', None)

        # Same pattern virtual_sdcard.py itself uses for background file
        # I/O - load_object() force-loads the [aio_executor] singleton if
        # no other extra has already, then hands out a private Executor.
        aio = self.printer.load_object(self.config, 'aio_executor')
        self.executor = aio.allocate_executor("nebulaos_plr")

        if not os.path.isdir(self.sidecar_dir):
            try:
                os.makedirs(self.sidecar_dir)
            except OSError as exc:
                raise self.printer.config_error(
                    "nebulaos_power_loss_recovery: could not create "
                    "sidecar_dir %r: %s" % (self.sidecar_dir, exc))

        self._ensure_eeprom_accessible()
        # Startup, nothing is moving yet: one direct EEPROM scan is fine
        # here. Every later EEPROM/sidecar access goes through the executor.
        try:
            self._refresh_recovery_status_blocking()
        except Exception as exc:
            logging.exception("nebulaos_power_loss_recovery: status read failed")
            self._set_error("status read failed: %s" % exc)
        self._report_newer_schema()
        self._offer_resume_prompt()

        self.timer_handler = self.reactor.register_timer(
            self._timer_event, self.reactor.NOW)

    def _offer_resume_prompt(self):
        """G80 (Run 22): after a power loss, ask in Mainsail (Klipper's
        action:prompt dialog) whether to resume - once, at the start. The
        screen asks on its own (PowerPanel). Only for a physical resume:
        the state-only restore leaves a cold, unhomed printer."""
        if self._status_state != STATE_RECOVERY_AVAILABLE or not self.physical_resume:
            return
        info = self._recovery_info
        name = os.path.basename(info.get("file") or "") or "the last print"
        gcode = self.printer.lookup_object('gcode')
        lines = ("// action:prompt_end",
                 "// action:prompt_begin Resume print after power loss?",
                 "// action:prompt_text %s was interrupted at %d %%." % (
                     name, int(100 * (info.get("progress") or 0.))),
                 "// action:prompt_text The bed and nozzle heat up, X/Y home, Z is "
                 "lifted from the height it had; keep the bed clear around the part.",
                 "// action:prompt_button Resume|NEBULAOS_RESUME_POWER_LOSS "
                 "ALLOW_UNSAFE=1 PHYSICAL=1|primary",
                 "// action:prompt_button Discard|NEBULAOS_CLEAR_POWER_LOSS_RECOVERY|error",
                 "// action:prompt_footer_button Later|RESPOND TYPE=command "
                 "MSG=action:prompt_end",
                 "// action:prompt_show")
        for line in lines:
            gcode.respond_raw(line)

    def _report_newer_schema(self):
        """G100 R1: after a rollback, an offer written by a newer NebulaOS
        is not readable here - say so once (klippy.log + console)."""
        if self._status_state != STATE_NONE:
            return
        try:
            with self._open_eeprom() as eeprom:
                newer = journal.newer_schema_present(eeprom)
        except Exception:
            return
        if newer:
            msg = ("NebulaOS PLR: recovery data from a newer NebulaOS version "
                   "was ignored (this version cannot read it)")
            logging.warning(msg)
            self.printer.lookup_object('gcode').respond_info(msg)

    def _ensure_eeprom_accessible(self):
        try:
            with open(self.eeprom_path, 'r+b'):
                pass
        except PermissionError as exc:
            # Per this mission's explicit instruction: never invent a
            # permission workaround (chmod/sudo/etc) - report the real
            # service user and stop.
            uid = os.getuid()
            raise self.printer.config_error(
                "nebulaos_power_loss_recovery: insufficient permission to "
                "open %r (running as uid=%d). This module refuses to "
                "chmod/sudo its way around this - fix the underlying "
                "device/service permissions instead. Original error: %s"
                % (self.eeprom_path, uid, exc))
        except (IOError, OSError) as exc:
            raise self.printer.config_error(
                "nebulaos_power_loss_recovery: could not open EEPROM path "
                "%r: %s" % (self.eeprom_path, exc))

    def _open_eeprom(self):
        return open(self.eeprom_path, 'r+b')

    def _config_sha256(self):
        configfile = self.printer.lookup_object('configfile', None)
        if configfile is None:
            return None
        status = configfile.get_status(self.reactor.monotonic())
        return config_identity(status.get('config'))

    def _firmware_sha(self):
        """The running NebulaOS image (firmware_sha from [nebulaos_version],
        read once at Klippy start). None without that object."""
        version = self.printer.lookup_object('nebulaos_version', None)
        if version is None:
            return None
        return version.get_status(self.reactor.monotonic()).get('firmware_sha')

    def _io(self, fn, *args):
        """Runs blocking EEPROM/sidecar/hash I/O on the aio_executor thread.
        Executor.submit() parks only the calling greenlet; the reactor keeps
        running, so an I2C scan or a SHA-256 over a large gcode file can never
        starve the move queue (a stall longer than Klipper's look-ahead ends
        in "Timer too close")."""
        return self.executor.submit(fn, *args)

    # -- recovery status ----------------------------------------------------

    def _read_record_blocking(self):
        with self._open_eeprom() as eeprom:
            return journal.read_recovery_state(eeprom)

    def _refresh_recovery_status_blocking(self):
        """Re-derives the cached front-end status from the EEPROM and the
        sidecar (light check, no hashing). Blocking: call through _io()
        except at klippy:ready."""
        record = self._read_record_blocking()
        if self._have_active_session:
            self._status_state = STATE_RECORDING
            return
        sidecar = None
        resolved = None
        if record is not None:
            sidecar = read_sidecar(sidecar_path_for_generation(
                self.sidecar_dir, record.generation))
            path = ((sidecar or {}).get("file") or {}).get("path")
            if path:
                resolved = self._resolve_sdcard_path(path)
        state, reason = light_recovery_check(record, sidecar, resolved,
                                             self._firmware_sha())
        if state == STATE_NONE:
            self._recovery_info = {}
        else:
            file_info = (sidecar or {}).get("file") or {}
            size = file_info.get("size") or 0
            self._recovery_info = {
                "generation": record.generation,
                "file": file_info.get("path", ""),
                "file_size": size,
                "file_position": record.file_position,
                "progress": (float(record.file_position) / size
                             if size else 0.0),
                "checked": "light",
                "reason": reason,
            }
        if (state == STATE_RECOVERY_AVAILABLE and self._resume_armed is not None
                and self._resume_armed[0] == record.generation):
            state = STATE_RESUME_PREPARED
        self._status_state = state

    def _refresh_recovery_status(self):
        try:
            self._io(self._refresh_recovery_status_blocking)
        except Exception as exc:
            logging.exception("nebulaos_power_loss_recovery: status refresh failed")
            self._set_error("status refresh failed: %s" % exc)

    def _set_error(self, message):
        self._last_error = message
        self._status_state = STATE_ERROR

    # -- periodic state machine --------------------------------------------

    def _timer_event(self, eventtime):
        try:
            self._tick(eventtime)
        except Exception:
            logging.exception("nebulaos_power_loss_recovery: timer tick failed")
        return eventtime + self.poll_interval

    def _tick(self, eventtime):
        ps_status = self.print_stats.get_status(eventtime)
        state = ps_status.get('state')

        # A candidate already taken may still be promotable even outside
        # the 'printing' state (e.g. the print just paused, but motion
        # queued before the pause is still physically executing) - always
        # check promotion first, regardless of state, then decide whether
        # this state permits taking a NEW candidate or requires
        # invalidating everything instead.
        if self._pending_candidate is not None:
            self._maybe_promote_candidate(eventtime)

        if state == 'printing':
            if not self._print_start_checked:
                self._check_new_print(eventtime)
            self._tick_printing(eventtime, ps_status)
        elif state in ('complete', 'cancelled'):
            self._pending_candidate = None
            self._print_start_checked = False
            if self._have_active_session:
                self._tombstone_session("print_%s" % state)
        elif state == 'error':
            self._pending_candidate = None
            self._print_start_checked = False
        elif state == 'standby':
            self._print_start_checked = False
        # paused / standby: preserve the latest durable checkpoint and any
        # still-pending candidate, take no new candidate.

    def _check_new_print(self, eventtime):
        """First 'printing' tick of a print. A durable record that belongs to
        an EARLIER print must not survive into this one: the first checkpoint
        of the new print only lands after Z > min_z_for_start and completed
        motion, and a power loss during homing/meshing/purging/the first
        layer would otherwise leave the old print's record looking valid
        (its own file is unchanged, so every identity check passes). The one
        exception is the record NEBULAOS_PLR_RESUME just restored, when the
        operator's M24 continues exactly that file."""
        self._print_start_checked = True
        if self._have_active_session:
            return
        armed = self._resume_armed
        self._resume_armed = None
        if armed is not None:
            vsd_status = self.virtual_sdcard.get_status(eventtime)
            current = self._relative_sdcard_path(
                vsd_status.get('file_path') or "")
            if current == armed[1]:
                self._last_action = "resumed"
                return
        try:
            record = self._io(self._read_record_blocking)
        except Exception as exc:
            logging.exception("nebulaos_power_loss_recovery: EEPROM read failed")
            self._set_error("EEPROM read failed: %s" % exc)
            return
        if record is not None:
            logging.info("nebulaos_power_loss_recovery: new print started, "
                         "invalidating the previous print's record "
                         "(generation %d)", record.generation)
            self._tombstone_session("superseded_by_new_print")

    def _tick_printing(self, eventtime, ps_status):
        vsd_status = self.virtual_sdcard.get_status(eventtime)
        th_status = self.toolhead.get_status(eventtime)

        if not self._have_active_session:
            if not meaningful_progress(ps_status, vsd_status, th_status,
                                        self.min_z_for_start):
                return
            self._start_session(vsd_status, eventtime)

        if not self._have_active_session:
            return  # _start_session may decline (e.g. hash failure)

        if self._checkpoint_in_flight:
            return
        self._maybe_take_candidate(eventtime, vsd_status)

    def _maybe_take_candidate(self, eventtime, vsd_status):
        """Decides whether to snapshot a new Candidate now. A candidate
        already pending is superseded (replaced, not persisted twice) once
        it has had a full checkpoint_interval to become promotable and a
        strictly newer file_position is available - this is what keeps a
        stalled candidate (e.g. one whose associated move sits unflushed
        for an unusually long time) from blocking all future progress
        forever, per the mission's "superseded candidate" requirement."""
        if self._pending_candidate is None:
            if eventtime - self._last_checkpoint_time < self.checkpoint_interval:
                return
        else:
            if eventtime - self._pending_candidate.taken_at < self.checkpoint_interval:
                return
            if vsd_status.get('file_position', 0) <= self._pending_candidate.file_position:
                return  # no real progress since the pending candidate - not worth superseding
        self._take_candidate(eventtime, vsd_status)

    def _start_session(self, vsd_status, eventtime):
        file_path = vsd_status.get('file_path')
        if not file_path:
            return
        try:
            size = os.path.getsize(file_path)
            # Mid-print by definition (Z > min_z_for_start): hashing a large
            # file here on the reactor would stall the move queue.
            sha256 = self._io(hash_file, file_path)
        except OSError:
            logging.exception(
                "nebulaos_power_loss_recovery: could not hash %r, "
                "not starting a PLR session", file_path)
            return
        # The reactor kept running while the hash was computed: the print may
        # have ended or switched files meanwhile.
        now = self.reactor.monotonic()
        if (self.print_stats.get_status(now).get('state') != 'printing'
                or self.virtual_sdcard.get_status(now).get('file_path')
                != file_path):
            return
        self._session_file_info = {
            "path": self._relative_sdcard_path(file_path),
            "size": size,
            "sha256": sha256,
        }
        self._session_config_sha256 = self._config_sha256()
        self._session_firmware_sha = self._firmware_sha()
        self._have_active_session = True
        self._status_state = STATE_RECORDING
        self._recovery_info = {
            "file": self._session_file_info["path"],
            "file_size": size,
            "file_position": 0,
            "progress": 0.0,
            "checked": "full",
            "reason": "",
        }
        # Force the very first candidate to be taken immediately, regardless
        # of how large `eventtime` (reactor uptime, not wall-clock) already
        # is at session-start time.
        self._last_checkpoint_time = eventtime - self.checkpoint_interval

    def _relative_sdcard_path(self, file_path):
        sdcard_root = getattr(self.virtual_sdcard, 'sdcard_dirname', None)
        if sdcard_root and file_path.startswith(sdcard_root):
            rel = os.path.relpath(file_path, sdcard_root)
            return rel
        return file_path

    # -- checkpoint execution semantics --------------------------------
    #
    # virtual_sdcard.file_position only proves gcode has been fully
    # DISPATCHED (confirmed against the exact pinned Klipper source,
    # klippy/extras/virtual_sdcard.py's work_handler(): self.file_position
    # is advanced to next_file_position only AFTER self.gcode.run_script(
    # line) returns for that line - so it never runs ahead of what has been
    # queued into the toolhead). Dispatched is NOT the same as physically
    # executed: Klipper deliberately buffers commands up to BUFFER_TIME_HIGH
    # (1.0s, toolhead.py) ahead of real time, so file_position can - and
    # normally does - reflect gcode whose motion is still sitting in the
    # trapq, not yet run.
    #
    # toolhead.register_lookahead_callback(cb) (toolhead.py) attaches cb to
    # the LAST move currently in the lookahead queue; cb fires with that
    # move's print_time once _process_lookahead() converts it into
    # scheduled trapq commands - this proves "queued with known timing", not
    # "physically executed". The actual physical-completion proof, used by
    # M400's own wait_moves(), is comparing that print_time against
    # mcu.estimated_print_time(reactor.monotonic()): print_time is
    # Klipper's internal monotonically-non-decreasing scheduling clock,
    # estimated_print_time() converts a real host time into the
    # corresponding point on that same clock, and motion is physically
    # complete once estimated_print_time(now) >= print_time. wait_moves()
    # busy-waits for this via reactor.pause() - unacceptable here (would
    # stall the whole reactor once every checkpoint), so this module polls
    # the same comparison non-blockingly from its own timer instead.
    #
    # This is why every checkpoint here passes through a two-stage
    # Candidate -> durable pipeline: a Candidate's file_position/state is
    # snapshotted immediately, but only ever written to the sidecar/EEPROM
    # once _maybe_promote_candidate() proves its associated motion has
    # actually finished, not merely been scheduled.

    def _take_candidate(self, eventtime, vsd_status):
        gm_status = self.gcode_move.get_status(eventtime)
        th_status = self.toolhead.get_status(eventtime)
        # "extruder_status" doubles as the thermal snapshot: extruder's own
        # get_status() already merges the heater's 'target' key with
        # pressure_advance/smooth_time (kinematics/extruder.py), and
        # bed_target is folded in here from the separate heater_bed object
        # so build_sidecar_state() has one dict to read both from.
        extruder_status = dict(self.extruder.get_status(eventtime)) \
            if self.extruder is not None else {}
        heater_bed = self.printer.lookup_object('heater_bed', None)
        if heater_bed is not None:
            extruder_status['bed_target'] = heater_bed.get_status(
                eventtime).get('target', 0.0)
        bed_mesh_status = self.bed_mesh.get_status(eventtime) \
            if self.bed_mesh is not None else {}
        get_mesh = getattr(self.bed_mesh, 'get_mesh', None)
        mesh = get_mesh() if get_mesh is not None else None
        if mesh is not None:
            # Not in get_status; needed to rebuild an unsaved mesh (C3).
            bed_mesh_status = dict(bed_mesh_status)
            bed_mesh_status['mesh_params'] = dict(mesh.get_mesh_params())
        exclude_object_status = self.exclude_object.get_status(eventtime) \
            if self.exclude_object is not None else {}
        fan_status = self.fan.get_status(eventtime) if self.fan is not None else {}
        fr_status = self.firmware_retraction.get_status(eventtime) \
            if self.firmware_retraction is not None else None

        candidate = Candidate(
            vsd_status.get('file_position', 0), self._session_file_info,
            gm_status, th_status, extruder_status, bed_mesh_status,
            exclude_object_status, fan_status, fr_status, eventtime)
        self._pending_candidate = candidate
        # Synchronous, on the reactor thread, immediately after reading
        # file_position above - nothing else can advance the gcode stream
        # in between (Klipper's reactor only runs one callback at a time),
        # so this candidate's file_position and "the last move currently
        # queued" are guaranteed to describe the same point in the print.
        self.toolhead.register_lookahead_callback(
            lambda print_time: self._on_candidate_timing_known(
                candidate, print_time))

    def _on_candidate_timing_known(self, candidate, print_time):
        # May fire synchronously (empty queue) or later, once the
        # associated move is flushed into the trapq. If this candidate was
        # already superseded/discarded in the meantime, its target_print_
        # time is simply never consulted again - harmless.
        candidate.target_print_time = print_time

    def _maybe_promote_candidate(self, eventtime):
        candidate = self._pending_candidate
        if candidate.target_print_time is None:
            return  # toolhead hasn't finalized this candidate's timing yet
        estimated = self.mcu.estimated_print_time(eventtime)
        if estimated < candidate.target_print_time:
            return  # motion scheduled but not yet physically executed
        if self._checkpoint_in_flight:
            return
        self._checkpoint_in_flight = True
        try:
            record = self._io(self._perform_checkpoint_blocking, candidate)
            self._last_checkpoint_time = eventtime
            self._pending_candidate = None
            self._last_action = "checkpoint"
            self._last_error = ""
            if self._have_active_session and record is not None:
                size = candidate.file_info.get("size") or 0
                self._recovery_info.update({
                    "generation": record.generation,
                    "file_position": record.file_position,
                    "progress": (float(record.file_position) / size
                                 if size else 0.0),
                })
        except Exception as exc:
            logging.exception(
                "nebulaos_power_loss_recovery: checkpoint failed")
            self._last_error = "checkpoint failed: %s" % exc
        finally:
            self._checkpoint_in_flight = False

    def _perform_checkpoint_blocking(self, candidate):
        """Runs on the aio_executor background thread, only once
        _maybe_promote_candidate() has already proven candidate's motion
        physically completed. Implements the checkpoint transaction's full
        7-step ordering: build the sidecar payload, durably write+rename
        it, fsync the directory, and ONLY THEN write+verify the EEPROM
        record."""
        with self._open_eeprom() as eeprom:
            current = journal.scan_journal(eeprom)
            _, next_generation = journal.next_commit_target(current)

            state = build_sidecar_state(
                next_generation, candidate.file_info, candidate.file_position,
                candidate.gm_status, candidate.th_status,
                candidate.extruder_status, candidate.bed_mesh_status,
                candidate.exclude_object_status, candidate.fan_status,
                candidate.fr_status, config_sha256=self._session_config_sha256,
                firmware_sha=self._session_firmware_sha)

            final_path = sidecar_path_for_generation(self.sidecar_dir,
                                                      next_generation)
            tmp_path = final_path + ".tmp"
            # Sidecar write (steps 2-5) happens BEFORE the EEPROM write
            # (steps 6-7) - the file must be fully durable on disk first,
            # so a crash between the two never leaves the EEPROM pointing
            # at a generation whose sidecar was never actually written.
            atomic_write_json(tmp_path, final_path, state)
            return journal.commit_checkpoint(eeprom, candidate.file_position)

    def _tombstone_blocking(self):
        with self._open_eeprom() as eeprom:
            journal.commit_tombstone(eeprom)

    def _tombstone_session(self, reason):
        try:
            self._io(self._tombstone_blocking)
            self._last_action = "tombstone:%s" % reason
        except Exception as exc:
            logging.exception(
                "nebulaos_power_loss_recovery: tombstone (%s) failed", reason)
            self._last_error = "tombstone (%s) failed: %s" % (reason, exc)
        self._have_active_session = False
        self._session_file_info = None
        self._pending_candidate = None
        self._resume_armed = None
        self._refresh_recovery_status()

    def _handle_reset_file(self):
        if self._resume_in_progress:
            # Our own NEBULAOS_PLR_RESUME issues M23 deliberately - this is
            # expected, not a foreign file change. Never invalidate our own
            # recovery transaction because of it.
            return
        if self._resume_armed is not None:
            # A different file (or the same one, re-selected from scratch)
            # replaced the restored state: the prepared resume is void and
            # the next print start must invalidate the record.
            self._resume_armed = None
            self._refresh_recovery_status()
        if self._have_active_session:
            logging.info(
                "nebulaos_power_loss_recovery: virtual_sdcard file changed "
                "mid-session - invalidating PLR rather than silently "
                "accepting it")
            self._tombstone_session("virtual_sdcard_reset_file")

    # -- gcode commands ------------------------------------------------------

    def cmd_NEBULAOS_PLR_STATUS(self, gcmd):
        try:
            record = self._io(self._read_record_blocking)
        except (IOError, OSError) as exc:
            gcmd.respond_info("NebulaOS PLR: EEPROM read failed: %s" % exc)
            return
        if record is None:
            gcmd.respond_info("NebulaOS PLR: no recovery available")
            return
        sidecar_path = sidecar_path_for_generation(self.sidecar_dir,
                                                    record.generation)
        sidecar = self._io(read_sidecar, sidecar_path)
        if sidecar is None:
            gcmd.respond_info(
                "NebulaOS PLR: EEPROM generation %d present but sidecar %s "
                "missing/unreadable" % (record.generation, sidecar_path))
            return
        gcmd.respond_info(
            "NebulaOS PLR: recovery available (generation=%d, "
            "file_position=%d, file=%s) - status "
            "AVAILABLE_POSITION_UNSAFE, use NEBULAOS_PLR_RESUME "
            "ALLOW_UNSAFE=1 to override the position-safety gate"
            % (record.generation, record.file_position,
               sidecar.get("file", {}).get("path")))

    def _load_resume_inputs_blocking(self):
        """EEPROM record, its sidecar, and the current identity (path, size,
        sha256) of the file the sidecar names - everything validate_recovery()
        needs. Blocking (hashes the whole file): run through _io()."""
        record = self._read_record_blocking()
        sidecar = None
        current = (None, None, None)
        if record is not None:
            sidecar = read_sidecar(sidecar_path_for_generation(
                self.sidecar_dir, record.generation))
        if sidecar is not None:
            candidate_path = (sidecar.get("file") or {}).get("path")
            if candidate_path:
                resolved = self._resolve_sdcard_path(candidate_path)
                if resolved and os.path.isfile(resolved):
                    current = (candidate_path, os.path.getsize(resolved),
                               hash_file(resolved))
        return record, sidecar, current

    def cmd_NEBULAOS_PLR_RESUME(self, gcmd):
        allow_unsafe = bool(gcmd.get_int('ALLOW_UNSAFE', 0, minval=0, maxval=1))
        physical = bool(gcmd.get_int('PHYSICAL', 0, minval=0, maxval=1))

        now = self.reactor.monotonic()
        state = self.print_stats.get_status(now).get('state')
        if state not in RESUME_ALLOWED_PRINT_STATES:
            raise gcmd.error(
                "NebulaOS PLR: resume refused while a print is %s" % state)
        if physical and not self.physical_resume:
            raise gcmd.error(
                "NebulaOS PLR: physical resume is disabled ([nebulaos_power_loss_"
                "recovery] physical_resume: False) until it has passed a "
                "supervised power-cut test on this printer")

        record, sidecar, current = self._io(self._load_resume_inputs_blocking)
        try:
            validated = validate_recovery(
                record, sidecar, current[0], current[1], current[2],
                allow_unsafe, current_config_sha256=self._config_sha256(),
                current_firmware_sha=self._firmware_sha())
        except PositionUnsafe as exc:
            raise gcmd.error(str(exc))
        except IntegrityFailure as exc:
            self._refresh_recovery_status()
            raise gcmd.error("NebulaOS PLR: recovery refused: %s" % exc)

        profiles = ((self.bed_mesh.get_status(now).get('profiles') or {})
                    if self.bed_mesh is not None else {})
        if physical:
            kin = self.toolhead.get_kinematics().get_status(now)
            z_max = kin.get('axis_maximum')[2] if kin.get('axis_maximum') else 0.
            try:
                lines = build_physical_resume_lines(
                    validated, self.resume_z_lift, self.prime_length, z_max,
                    list(profiles))
            except PhysicalResumeRefused as exc:
                raise gcmd.error("NebulaOS PLR: physical resume refused: %s"
                                 % exc.reason)
        else:
            try:
                lines = build_resume_gcode_lines(validated, list(profiles))
            except MeshNotRestorable as exc:
                raise gcmd.error("NebulaOS PLR: recovery refused: %s"
                                 % exc.reason)
        self._resume_mesh = validated.get("bed_mesh") or {}

        # Armed before the lines run: a physical resume ends with M24, and the
        # print it starts must keep this record (see _check_new_print).
        self._resume_armed = (validated["generation"],
                              validated["file"]["path"])
        self._resume_in_progress = True
        try:
            for line in lines:
                self.gcode.run_script_from_command(line)
        except Exception:
            self._resume_armed = None
            raise
        finally:
            self._resume_in_progress = False

        self._last_action = "resumed_physical" if physical else "resume_prepared"
        self._refresh_recovery_status()
        self._recovery_info["checked"] = "full"
        if physical:
            gcmd.respond_info(
                "NebulaOS PLR: physical resume done (generation=%d): X/Y "
                "homed, Z taken as recorded, primed, printing resumed."
                % validated["generation"])
        else:
            gcmd.respond_info(
                "NebulaOS PLR: resume state restored (generation=%d). "
                "No motion was performed and M24 was NOT issued - verify the "
                "physical position is safe, then resume printing manually."
                % validated["generation"])

    def cmd_NEBULAOS_PLR_SET_Z(self, gcmd):
        """Declares Z homed at the recorded height. Only valid inside a
        physical resume - never a general-purpose way to fake homing."""
        if not self._resume_in_progress:
            raise gcmd.error("_NEBULAOS_PLR_SET_Z is internal to "
                             "NEBULAOS_PLR_RESUME PHYSICAL=1")
        z = gcmd.get_float('Z', above=0.)
        self.toolhead.get_last_move_time()
        pos = self.toolhead.get_position()
        pos[2] = z
        self.toolhead.set_position(pos, homing_axes="z")

    def _make_mesh(self, params, name):
        """Upstream's own ZMesh, with the parameters typed exactly as
        ProfileManager does when it loads a saved profile."""
        from . import bed_mesh as bed_mesh_module
        typed = {key: kind(params[key])
                 for key, kind in bed_mesh_module.PROFILE_OPTIONS.items()}
        return bed_mesh_module.ZMesh(typed, name)

    def cmd_NEBULAOS_PLR_LOAD_MESH(self, gcmd):
        """Rebuilds the mesh the print used from the checkpoint (C3), the
        same steps as upstream's BED_MESH_PROFILE LOAD. Only valid inside
        NEBULAOS_PLR_RESUME."""
        mesh = self._resume_mesh if self._resume_in_progress else None
        if not mesh or not mesh.get("points") or not mesh.get("mesh_params"):
            raise gcmd.error("_NEBULAOS_PLR_LOAD_MESH is internal to "
                             "NEBULAOS_PLR_RESUME")
        if self.bed_mesh is None:
            raise gcmd.error("NebulaOS PLR: no [bed_mesh] to restore the "
                             "recorded mesh into")
        try:
            z_mesh = self._make_mesh(mesh["mesh_params"], mesh["profile_name"])
            z_mesh.build_mesh([list(row) for row in mesh["points"]])
        except Exception as exc:
            raise gcmd.error("NebulaOS PLR: the recorded bed mesh '%s' could "
                             "not be rebuilt: %s" % (mesh["profile_name"], exc))
        self.bed_mesh.set_mesh(z_mesh)
        gcmd.respond_info("NebulaOS PLR: bed mesh '%s' restored from the "
                          "checkpoint" % (mesh["profile_name"],))

    def cmd_NEBULAOS_PLR_DISCARD(self, gcmd):
        self._tombstone_session("manual_discard")
        self._last_action = "discarded"
        gcmd.respond_info("NebulaOS PLR: checkpoint discarded (tombstoned)")

    def _resolve_sdcard_path(self, relative_path):
        sdcard_root = getattr(self.virtual_sdcard, 'sdcard_dirname', None)
        if sdcard_root:
            return os.path.join(sdcard_root, relative_path)
        return relative_path

    def get_status(self, eventtime):
        # Cached values only - no I/O. "position_safe" stays False: even the
        # physical resume ASSUMES Z held while unpowered; it never measures
        # it. "resume_mode" says which resume NEBULAOS_PLR_RESUME offers.
        return {
            "state": self._status_state,
            "position_safe": False,
            "resume_mode": ("physical" if self.physical_resume
                            else "state_restore_only"),
            "recovery": dict(self._recovery_info),
            "last_action": self._last_action,
            "last_error": self._last_error,
            "active_session": self._have_active_session,
            "resume_in_progress": self._resume_in_progress,
            "candidate_pending": self._pending_candidate is not None,
        }


def load_config(config):
    return NebulaOSPowerLossRecovery(config)
