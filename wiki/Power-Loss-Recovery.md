# Power-loss recovery

What happens to a print when the power goes off, and how NebulaOS continues it. The page starts
with what you see on the printer and gets more technical further down. The last sections are for
developers.

Code: `extras/nebulaos_power_loss_recovery.py` and `extras/nebulaos_plr_journal.py`, at
`6532091` (the `production` branch, shipped with NebulaOS 1.0.3). Configuration: the
`[nebulaos_power_loss_recovery]` section in the firmware's `machine.cfg`.

---

## In short

While a print runs, the printer saves where it is in the G-code file every few seconds. If the
power goes off, then after the next start the screen asks:

> **Power loss.**
> Resume *your_print.gcode* at 42%?
> It heats up, homes X and Y and continues. Z is NOT re-homed: only resume if the part is still
> on the bed.

- **Resume** heats the bed and nozzle, lifts the nozzle away from the part, homes X and Y, primes
  a little filament off the part, moves back and continues printing from the saved point.
- **Later** keeps the saved state. You can resume or discard it later from the **Power** page
  (on the screen: **Tune** tab → **Power Settings**).
- **Discard** (on the Power page) throws the saved state away.

Mainsail shows the same question as a dialog when Klipper starts.

## Before you press Resume

- **The part must still be on the bed, exactly where it was.** The printer cannot measure where
  the part is. It trusts that nothing moved while the power was off.
- **Nothing may lie on the bed around the part.** X and Y are homed, which moves the bed and the
  print head to their endstops.
- **Z is not homed again.** Homing Z would drive the nozzle into the part. The printer assumes the
  nozzle height did not change while it was off - the same assumption Creality's stock resume
  makes.
- **The same file, the same printer settings, the same NebulaOS version.** If any of these
  changed, Resume is refused (see [When Resume is refused](#when-resume-is-refused)).

## What gets saved

Every save (a *checkpoint*) records:

| What | Why it is needed |
|---|---|
| The G-code file: path, size and SHA-256 fingerprint | To continue in exactly the same file, and to refuse if it changed |
| The position in the file (byte offset) | Where printing continues |
| The nozzle position (X, Y, Z, E) and the coordinate modes | To move back to the exact point |
| Speed and flow factors (M220, M221) and the Z offset | Your adjustments during the print stay |
| Velocity and acceleration limits, pressure advance | Same motion as before |
| Nozzle and bed target temperatures, part-cooling fan speed | Same printing conditions |
| The bed mesh in use, including all probed points | The print's adaptive mesh was never saved as a profile, so the points themselves are kept |
| Objects defined, objects excluded, the object being printed | "Exclude object" keeps working after the resume |
| Firmware retraction settings | Same retraction as before |
| A fingerprint of the whole printer configuration and the NebulaOS version | To refuse a resume after a configuration change or an update |

Saving starts only once the print is really under way: the print is running, time has passed, and
the nozzle is above **0.6 mm** (`min_z_for_start`). A power loss during heating, homing, the bed
mesh or the purge line therefore leaves nothing to resume - start the print again.

When a print **finishes** or is **cancelled**, the saved state is invalidated. When a **new print
starts**, any state left from an earlier print is invalidated at once, before the new print saves
anything of its own.

## How often, and how much can be lost

A checkpoint is taken about every **5 seconds** (`checkpoint_interval`), but only written once the
printer has *physically finished* the moves up to that point. Klipper plans moves up to about a
second ahead, so the saved position always lags a little behind the nozzle. After a power loss
the printer continues from the last checkpoint. Moves made after it, before the power went off,
are printed a second time - at most a few seconds' worth. That is the deliberate choice: a
checkpoint that ran ahead of the nozzle would make the resume skip G-code and leave a gap.

## Where it is stored, and why in two places

The state is split between two storage places, and this split is what makes it reliable:

1. **A small record in the EEPROM** - a 2 KB memory chip on the KE's display board (BL24C16F).
   Each record is 16 bytes: a generation number, the file position and a checksum. The EEPROM
   is the *authority*: if the EEPROM has no valid record, nothing is resumed.
2. **A JSON file with everything else** on the eMMC, in `/usr/data/nebulaos/printer_data/plr/`.
   There are two files, `state-a.json` and `state-b.json`, used in turn.

The order of writing is fixed: first the JSON file is written completely, flushed to the disk and
renamed into place; **only then** the EEPROM record is written and read back to check it. A
power cut at any moment leaves one of two situations:

- the cut came before the EEPROM record: the EEPROM still points to the *previous* checkpoint,
  whose JSON file is untouched (it is the other one of the two files);
- the cut came after: the new checkpoint is complete in both places.

There is no moment where the EEPROM points to a half-written file.

## When Resume is refused

The checks run in this order. Each one stops the resume with a message on the screen and in the
console; none can be bypassed.

| Message (short form) | Meaning | What to do |
|---|---|---|
| no EEPROM checkpoint available | Nothing was saved, or it was invalidated | Start the print again |
| sidecar missing or unreadable | The JSON file is gone or damaged | Start the print again |
| sidecar schema_version mismatch | Saved by a NebulaOS version with a different format | Start the print again |
| stale sidecar generation | The JSON file does not belong to the EEPROM record | Start the print again |
| gcode file missing / size changed / file identity mismatch | The file was deleted, replaced or edited | Start the print again |
| printer configuration changed since the checkpoint | Something in `printer.cfg` changed: Z offset, mesh, any setting | Start the print again |
| NebulaOS firmware changed since the checkpoint | An update was installed in between | Start the print again |
| bed mesh '…' was not saved and was not recorded | The mesh in use cannot be restored | Start the print again |
| no room to lift above the part | The part is so tall that the nozzle cannot lift by at least 1 mm | Start the print again |
| no recorded nozzle temperature | The print had no nozzle target | Start the print again |

The configuration and firmware checks are deliberately strict. A changed Z offset or a different
NebulaOS version can change where the nozzle ends up, and a resumed print sits on a part you
cannot re-measure.

---

## How the resume works, step by step

The screen's **Resume** button sends `NEBULAOS_RESUME_POWER_LOSS ALLOW_UNSAFE=1 PHYSICAL=1`,
which is a macro for `NEBULAOS_PLR_RESUME`. With `PHYSICAL=1` it runs this sequence
(`build_physical_resume_lines()`):

1. **Heat bed and nozzle together** to the saved targets (`M140`, `M104`), then wait for both
   (`M190`, `M109`). The nozzle sits in the plastic of the part; a cold nozzle would tear the
   part off the bed when it lifts, so it softens first.
2. **Declare Z** at its saved height (`_NEBULAOS_PLR_SET_Z`). Klipper does not know where Z is
   after a restart; this tells it, without moving.
3. **Lift Z** by `resume_z_lift` (default **5 mm**), or less if the axis has no room, at least
   1 mm.
4. **Home X and Y** (`G28 X Y`). Z is declared as homed before this on purpose: Klipper's
   `safe_z_home` would otherwise reset Z.
5. **Restore the saved state**: select the file, re-define the objects, set the file position,
   speed and flow factors, Z offset, velocity limits, pressure advance, fan, bed mesh and
   retraction.
6. **Prime** `prime_length` (default **8 mm**) of filament at the X/Y endstop position, off the
   part. Ooze from heating goes there, not into the print.
7. **Move back**: first X/Y at the lifted height, then Z down to the saved height.
8. **Set the extruder position and coordinate modes** back to the saved ones.
9. If NebulaOS timelapse is active: restart the layer watcher.
10. **`M24`** - printing continues.

If any line fails part-way, the resume is undone: the nozzle heater is turned off and the file is
unloaded. The bed stays warm so the part stays attached, and the saved state is kept, so Resume
can be tried again.

### Timelapse across a power loss

The timelapse plugin empties its frame folder whenever a file is selected - including the
resume's own file selection. Before resuming, NebulaOS moves the frames taken before the power
loss aside (to `<frame_path>-plr/`). When the print ends (`END_PRINT` or `CANCEL_PRINT` call
`_NEBULAOS_PLR_TIMELAPSE_MERGE`), they are put back in front of the new frames, renumbered, before
the video is rendered. The video therefore covers the whole print.

---

## For developers and tinkerers

### Commands

| Command | What it does |
|---|---|
| `NEBULAOS_PLR_STATUS` | Reports whether a checkpoint exists (EEPROM + JSON file) |
| `NEBULAOS_PLR_RESUME [ALLOW_UNSAFE=0\|1] [PHYSICAL=0\|1]` | Validates and resumes. Without `ALLOW_UNSAFE=1` it always refuses (see below). `PHYSICAL=1` requires `physical_resume: True` |
| `NEBULAOS_PLR_DISCARD` | Invalidates the checkpoint (writes a tombstone record) |
| `NEBULAOS_RESUME_POWER_LOSS`, `NEBULAOS_CLEAR_POWER_LOSS_RECOVERY` | The firmware's public macros for the two commands above (`calibration.cfg`) |
| `_NEBULAOS_PLR_SET_Z`, `_NEBULAOS_PLR_LOAD_MESH` | Internal; refuse to run outside a resume |
| `_NEBULAOS_PLR_TIMELAPSE_MERGE` | Called by `END_PRINT` / `CANCEL_PRINT` |

**`ALLOW_UNSAFE=1`** acknowledges one specific fact: the Z position after a power loss cannot be
proven, only assumed. Every integrity check (file, configuration, firmware, generation) runs
first and cannot be bypassed by it. `ALLOW_UNSAFE` is checked last, so it can never skip past an
integrity failure.

**`PHYSICAL=0`** (the default) restores the state only: file, file position, factors, offsets,
mesh, temperatures - without any motion and without `M24`. It is a building block for manual
recovery from the console: you position the head yourself, then send `M24`.

### Configuration

`[nebulaos_power_loss_recovery]`, options and defaults from the module:

| Option | Default | NebulaOS sets | Meaning |
|---|---|---|---|
| `eeprom_path` | `/sys/bus/i2c/devices/2-0050/eeprom` | same | The EEPROM through Linux's `at24` driver |
| `sidecar_dir` | `/usr/data/nebulaos/printer_data/plr` | - | Where `state-a.json` / `state-b.json` live |
| `checkpoint_interval` | `5.0` s | - | Minimum time between checkpoints |
| `poll_interval` | `0.75` s | - | How often the state machine runs |
| `min_z_for_start` | `0.6` mm | - | Height the nozzle must pass before saving starts |
| `physical_resume` | `False` | **`True`** | Allows the full resume with motion and `M24`. The module's default is off; NebulaOS turns it on |
| `resume_z_lift` | `5.0` mm (1-20) | - | Lift before the X/Y homing |
| `prime_length` | `8.0` mm (0-50) | - | Filament primed at the park position |
| `timelapse_frame_path` | empty | `/opt/printer_data/misc/timelapse-frames/` | Enables the timelapse frame handling |

### Status object

`printer.nebulaos_power_loss_recovery` (what the screen and Mainsail read; never does I/O):

| Field | Values |
|---|---|
| `state` | `NONE`, `RECORDING`, `RECOVERY_AVAILABLE`, `RECOVERY_INCOMPATIBLE`, `RESUME_PREPARED`, `ERROR` |
| `resume_mode` | `physical` or `state_restore_only` |
| `position_safe` | always `False` - Z is assumed, never measured |
| `recovery` | `file`, `file_size`, `file_position`, `progress` (0-1), `generation`, `checked` (`light`/`full`), `reason` |
| `last_action`, `last_error` | e.g. `checkpoint`, `tombstone:print_complete`, `resumed_physical` |
| `active_session`, `resume_in_progress`, `candidate_pending` | booleans |

At startup only a *light* check runs (no file hashing, which takes seconds on this processor):
EEPROM record, JSON file, generation, file present with the same size, same firmware. The full
check including the SHA-256 of the G-code file and the configuration fingerprint runs when
Resume is pressed.

### The state machine

A timer runs every `poll_interval` and looks at Klipper's `print_stats`:

- **printing**, first tick of a print: if a record from an earlier print exists, it is
  tombstoned at once. Exception: the record a resume just restored, when the same file
  continues.
- **printing**: once `meaningful_progress()` is true (printing, file position > 0, print time
  > 0, Z > `min_z_for_start`), a *session* starts: the file is hashed on a background thread and
  the configuration and firmware fingerprints are taken.
- **printing**, during a session: a *candidate* checkpoint is snapshotted, then promoted to a
  durable checkpoint once its moves are physically done.
- **complete / cancelled**: the session ends with a tombstone.
- **paused / standby / error**: nothing new is taken; the last checkpoint stays.
- A different file loaded mid-session (`virtual_sdcard:reset_file`) also tombstones the session.

### Why a checkpoint waits for the motion

`virtual_sdcard.file_position` advances when a G-code line has been *handed to* the motion
planner, not when the printer has executed it - Klipper queues up to about one second of moves
ahead. Saving that position directly would let a resume skip moves that never happened.

So each checkpoint is two-stage:

1. **Candidate**: the file position and all status are read in one reactor callback, and
   `toolhead.register_lookahead_callback()` is attached to the last queued move. That callback
   delivers the move's `print_time` once it is scheduled.
2. **Promotion**: on later ticks, the candidate is written only when
   `mcu.estimated_print_time(now) >= print_time` - the same comparison Klipper's own `M400` uses,
   but polled instead of blocking the reactor.

A candidate that waits a full `checkpoint_interval` without being promotable is replaced by a
newer one, so a stalled move can never block checkpointing for good.

All EEPROM, file and hashing I/O runs on Klipper's `aio_executor` thread. A SHA-256 over a large
file or an I2C scan on the reactor thread would stall the move queue ("Timer too close").

### EEPROM journal format

`nebulaos_plr_journal.py`, BL24C16F, 2048 bytes in 128 pages of 16 bytes (the chip's write-page
size, so every record is one atomic page write):

| Page | Use |
|---|---|
| 0 | Reserved for Creality's stock power-loss recovery. NebulaOS never reads or writes it |
| 1-127 | NebulaOS journal: one record per page, used as a ring |

Record (little-endian):

| Offset | Size | Field |
|---|---|---|
| 0 | 2 | magic `NP` |
| 2 | 1 | schema version (1) |
| 3 | 1 | flags: `0x01` valid, `0x02` tombstone |
| 4 | 4 | generation (u32) |
| 8 | 4 | file position (u32) |
| 12 | 4 | CRC32 of bytes 0-11 |

- **No header page.** The current state is found by scanning all 127 pages and taking the
  valid record with the newest generation. Generations are compared wrap-safe (RFC 1982 serial
  arithmetic); generation 0 is never used.
- **Each commit writes the next page in the ring**, so a torn write only ever damages the newest
  page. Its CRC fails, the scan ignores it, and the previous record - on another page - is the
  current state again. Spreading writes over 127 pages also spreads EEPROM wear.
- **Every write is read back and compared** byte for byte; a mismatch fails the checkpoint.
- **A tombstone** is a record with the tombstone flag: if the newest record is a tombstone, there
  is nothing to resume.
- **The generation's parity selects the JSON file** (even: `state-a.json`, odd: `state-b.json`).
  The file being written is never the one the EEPROM currently points to.
- A valid record of a *newer* schema (after a downgrade to an older NebulaOS) is reported once in
  the console and `klippy.log` instead of silently showing "no recovery".

Linux owns the EEPROM through its standard `at24` driver (device tree: `atmel,24c16`). The
community `[bl24c16f]` module that talks to the chip directly over the MCU is not used for this.

### JSON file (schema 3)

Written atomically: `state-x.json.tmp`, `fsync`, rename, `fsync` of the directory. Top-level
keys: `schema_version`, `generation`, `file` (`path` relative to the G-code folder, `size`,
`sha256`), `file_position`, `gcode` (`gcode_position`, coordinate modes, `speed_factor`,
`extrude_factor`, `homing_origin`), `toolhead.position` (the kinematic position the physical
resume uses for Z), `identity` (`config_sha256`, `firmware_sha`), `motion`, `thermal`, `fan`,
`extruder`, `bed_mesh` (profile name, and the probed points with their parameters),
`exclude_object`, `firmware_retraction`.

`identity.config_sha256` is a SHA-256 over Klipper's parsed configuration (the `configfile`
status `config`: every section and option, includes and the `SAVE_CONFIG` block included). Any
setting Klipper actually reads changes it.

### Upstream Klipper interfaces used

Everything is read from public `get_status()` dictionaries and restored with standard G-code,
with no Klipper core patches. Two places go slightly further:

- **The bed mesh.** The adaptive mesh a print uses is never saved as a profile, so
  `BED_MESH_PROFILE LOAD` cannot bring it back. Its points come from
  `BedMesh.get_mesh().get_mesh_params()` and are rebuilt on resume exactly as upstream's profile
  load does (`ZMesh`, `build_mesh`, `BedMesh.set_mesh`).
- **Declaring Z.** `_NEBULAOS_PLR_SET_Z` calls `toolhead.set_position(..., homing_axes="z")`, which
  is what upstream `SET_KINEMATIC_POSITION` does. Upstream only registers that command with
  `[force_move] enable_force_move`, which NebulaOS does not turn on for everyone.

These symbols are listed in `nebulaos-extensions.json` under `required_klipper_symbols`. If a
Klipper update changes them, Klipper refuses to start instead of breaking a resume later.

### Tests

`extras/test_nebulaos_power_loss_recovery.py` and `extras/test_nebulaos_plr_journal.py` run the
pure logic (validation chain, resume G-code, journal ring, torn writes, generation wrap-around,
timelapse merge) with plain dictionaries and temporary files, without a printer. See
[Testing](Testing) for how to run them.

### See also

- [Architecture](Architecture) - how state is persisted across the extension set, and why the
  physical resume's order is what it is.
- [Commands in detail](Commands-Detail) - parameters and errors of every command above.
- [Configuration](Configuration) - all options with their constraints.
