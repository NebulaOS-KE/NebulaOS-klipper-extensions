<!-- Hand-written. Unlike COMMANDS*.md and CONFIG.md in this directory, this page
     is not generated - it explains, and explanation cannot be extracted. Every
     factual claim names the file it came from so it can be re-checked. -->

# How the extensions work

A developer's guide to `NebulaOS-klipper-extensions` at commit `178a0c2` - the NebulaOS 1.0.2
release pin, and also the tip of both `main` and `production`, so this describes exactly what
ships: what this repository is, how its code reaches a running printer, and how the pieces fit
together.

For exhaustive lists, see [COMMANDS-DETAIL.md](COMMANDS-DETAIL.md) and
[CONFIG.md](CONFIG.md). This page is the part those cannot give you - why the design is shaped
this way.

## The one thing to understand first

**NebulaOS does not fork Klipper.**

Host Klipper is pristine upstream [`Klipper3d/klipper`](https://github.com/Klipper3d/klipper),
pinned by the firmware build and owned by it. This repository contains **loadable modules**
that Klipper discovers exactly the way it discovers its own: files in `klippy/extras/`, each
providing a `[section]` that `printer.cfg` can use.

That choice drives nearly everything else on this page. It means upstream improvements arrive
without a merge, upstream documentation stays true for everything upstream does - and it means
NebulaOS has to be careful, because it is a guest in someone else's process.

Practical consequence when reading: if a behaviour is not described here or in the command
reference, look in upstream Klipper's documentation before assuming NebulaOS invented it.

## How the code gets into Klipper

The modules are not copied into the Klipper checkout at build time. They are **composed** into
it on the printer, and `nebulaos-extensions.json` describes exactly how:

```json
"composition": {
  "source_dir": "extras",
  "destination_dir": "klippy/extras",
  "link_type": "symlink",
  "exclude_file": ".git/info/exclude",
  "marker_file": ".nebulaos-composed",
  "require_symlink_resolving_inside_source": true
}
```

Reading that in order:

- Each file in `extras/` becomes a **symlink** in the Klipper checkout's `klippy/extras/`.
  Klipper then imports it as if it were its own.
- The symlinks are listed in `.git/info/exclude`, so the Klipper checkout stays clean. `git
  status` in Klipper does not show them, and Moonraker's update manager does not see a dirty
  tree it wants to fix.
- `.nebulaos-composed` marks a checkout as already composed, making the operation idempotent.
- `require_symlink_resolving_inside_source` means a link must resolve **inside** the source
  directory. A link pointing anywhere else is refused rather than followed.

The firmware's `S05nebulaos-activate` boot service performs this. Which is worth knowing when
a module seems absent: the question is usually whether composition ran, not whether the file
exists.

## The compatibility contract

This is the most interesting thing in the repository, and the direct consequence of not
forking.

If NebulaOS runs against unmodified upstream Klipper, then upstream can change under it. A
renamed method or a reorganised class would normally surface as an exception somewhere
unpredictable - quite possibly mid-print.

So the manifest declares what it needs:

```json
"required_klipper_symbols": [
  "mcu:MCU.register_serial_response",
  "mcu:MCU.create_oid",
  "mcu:MCU.register_config_callback",
  ...
],
"forbidden_klipper_symbols": []
```

26 required symbols at this commit, each as `module:Class.attribute`. `nebulaos_compat.py`
checks them **at every Klipper start**, as a preflight. If one is missing, Klipper stops with a
clear message instead of starting and failing later.

`forbidden_klipper_symbols` is currently empty, and is the mirror image: a list of symbols that
must **not** reappear. It exists so that a retired API cannot quietly return.

Three numbers in the manifest version the contract itself:

| Field | At `178a0c2` | Meaning |
|---|---|---|
| `compat_schema_version` | `1` | Version of the manifest format |
| `extensions_version` | `1.0.0` | Version of this extension set - **not** the NebulaOS release number, which is 1.0.2 at this same commit |
| `nebulaos_api_level` | `1` | Version of the interface other components rely on |

**This is the design decision worth taking away.** The cost of not forking is that your
dependency moves without telling you. The answer here is not pinning and hoping - it is
declaring the dependency precisely and refusing to start when it is not met. A documented
failure at startup beats an undocumented one during a print.

## What is in `extras/`

The manifest lists **62 modules**: 31 `runtime` and 31 `test`. One test file per runtime
module, almost exactly - unusually good for this kind of code, and the reason the command
reference can cite a test for most commands.

Roughly grouped by what they do:

**Calibration** - `nebulaos_calibration.py` is the coordinator; `nebulaos_bed_tram.py`,
`nebulaos_load_cell_calibration.py`, `nebulaos_z_offset_probe.py`, `z_compensate.py`,
`nozzle_clear.py` and `calibrate_shaper_config.py` do the individual procedures.
`nebulaos_calibration_history.py` and `nebulaos_calibration_journal.py` record what happened.

**Motion and drivers** - `autotune_tmc.py`, `nebulaos_tmc_autotune.py`, `tmcstatus.py`,
`motor_constants.py`, with `motor_database.cfg` holding the motor data.

**Hardware access** - `nebulaos_beeper.py` (buzzer PWM), `bl24c16f.py` (EEPROM),
`virtual_pins.py`, `nebulaos_temperature_mcu.py`.

**Resilience** - `nebulaos_power_loss_recovery.py` with `nebulaos_plr_journal.py`,
`nebulaos_memory.py`, `nebulaos_compat.py`.

**Support** - `gcode_shell_command.py`, `nebulaos_version.py`, `nebulaos_z_adjust.py`,
`nebulaos_bed_soak.py`, `prtouch_test_support.py`.

## How a calibration actually runs

Worth tracing once, because the layering is not obvious from any single file and it explains
why the command list has underscore-prefixed entries.

```
NebulaScreen panel
  -> a gcode_macro in the firmware's calibration.cfg      (the public name)
  -> _NEBULAOS_<THING>, registered by a module here       (the implementation)
  -> the module runs its sequence, writing a journal as it goes
  -> NebulaScreen polls the journal and the status object to draw progress
```

Three things follow from that shape:

**The public name may not be in this repository.** `NEBULAOS_BED_TRAM` is a macro in the
firmware's `calibration.cfg`; this repository registers `_NEBULAOS_BED_TRAM`. A command
reference built from this repository alone cannot see those macros, and
[COMMANDS-DETAIL.md](COMMANDS-DETAIL.md) says so rather than presenting itself as the complete
list of what a user can type.

**The underscore is a real convention, not decoration.** It marks a command as a step or an
implementation detail. `_NEBULAOS_PLR_SET_Z` raises an error if you call it directly, saying it
is internal to `NEBULAOS_PLR_RESUME PHYSICAL=1`.

**Progress is a journal, not a return value.** A calibration takes minutes, so the command
cannot simply return an answer. It writes progress to a journal file and a status object, and
the screen reads them. That is why `nebulaos_calibration_journal.py` and
`nebulaos_plr_journal.py` exist as modules of their own, and why a calibration can be cancelled
by a separate `_..._CANCEL` command rather than by interrupting the first one.

## How state is persisted

Two patterns, chosen per risk.

**Ordinary settings use a file**, written atomically: write a temporary file, `fsync`, rename
into place, `fsync` the directory. A crash leaves either the old file or the new one, never
half of either. `nebulaos_z_adjust.py` and `nebulaos_bed_soak.py` work this way.

**Power-loss recovery uses two tiers,** because it has to survive the power actually going
out mid-write. A small durable record on the board's EEPROM holds a generation counter, a file
position and a CRC. A larger JSON sidecar holds the full state, in A/B files selected by the
generation's parity.

The ordering is the whole point: **the sidecar is written and fsynced first, and the EEPROM
record last.** The EEPROM record is the only thing trusted as proof that a checkpoint landed,
so a cut between the two leaves a checkpoint that is simply not offered - rather than one that
is offered and then fails. If you ever wonder why a printer reports nothing to recover after a
power cut, that is the mechanism working.

## Reading values back out

Two routes, and they serve different readers.

**Status objects** - `get_status()` dicts, which Moonraker exposes and the screen subscribes
to. This is how live state reaches the UI.

**Journals** - append-only files recording what a long operation did, which survive a restart
and can be read after the fact. `NEBULAOS_CALIBRATION_STATUS` reports from these.

A rule the modules follow and that is worth following: **everything restored on resume goes
back through a standard G-code command**, so recovery uses the same paths as normal operation
rather than a private back door. `nebulaos_power_loss_recovery.py` documents the single
exception - the bed mesh, because an adaptive mesh is never saved as a profile, so
`BED_MESH_PROFILE LOAD` cannot bring it back. It is rebuilt exactly as upstream's own profile
load does, using symbols listed in `required_klipper_symbols` so that drift refuses to start
rather than breaking a resume.

## Two commands that replace upstream behaviour

Worth knowing before debugging either, because they do not behave as stock Klipper:

- **`SET_PIN`** - `nebulaos_bed_soak.py` de-registers upstream's and re-registers its own, so
  the bed warp soak switch is remembered across Klipper restarts.
- **`Z_OFFSET_APPLY_PROBE`** - `nebulaos_z_adjust.py` does the same, to apply the offset through
  NebulaOS's own persistence.

Both are registered by first calling `register_command(name, None)`, which removes the upstream
command. Anyone arriving from stock Klipper will expect the upstream semantics and should be
told otherwise - which is why
[COMMANDS-DETAIL.md](COMMANDS-DETAIL.md) marks them explicitly.

## The physical resume, and why its order is what it is

`nebulaos_power_loss_recovery.py` can do more than re-arm a file position: with
`physical_resume: True` (which the shipped `machine.cfg` sets) it brings the printer back to the
interrupted state itself. The sequence in `build_physical_resume_lines` is constrained at almost
every step, and changing it casually will break something that is not obvious:

1. **Both heaters start together, and both are waited for** - `M140` and `M104` first, then
   `M190` and `M109`, before anything moves. The nozzle is sitting *in* the plastic it was
   printing: lifting Z with it cold pulls the part off the bed. They do not reach their targets
   at the same moment, and that is accepted.
2. **`_NEBULAOS_PLR_SET_Z` declares Z at its recorded kinematic height, then lifts.** This is an
   ordering constraint from upstream's `safe_z_home`: a `G28` with Z unhomed resets Z to 0 and
   clears its homing, so Z has to be declared before X/Y are homed, and lifted before any XY
   move. Stock Creality's resume moves XY at print height instead.
3. **X/Y home, then the nozzle primes at the park position**, off the bed at the X/Y endstop.
   Priming there, rather than over the part, is what keeps ooze out of the print - including
   the ooze from heating in step 1.

   **This order was reversed once, deliberately, and the history is worth knowing.** Until
   `ed99372` the nozzle heated only after parking, because a power-cut test on the development
   printer found that heating it *over the part* marked the part and oozed onto it. `178a0c2`
   reverted that: a cold nozzle held in the part pulls the part off the bed the moment Z lifts,
   which is the worse of the two failures. Priming at the park position keeps the benefit that
   the earlier order was reaching for. If you are reading an older document that says "bed
   first, nozzle off", it described `ed99372`.
4. **Exclude-object state is replayed**, including `current_object` - without it the resumed
   file's `EXCLUDE_OBJECT_END` had nothing to match.
5. **The timelapse watcher is restarted** (`_NEBULAOS_TIMELAPSE ACTION=START`) when the macro
   exists, because a resume skips `START_PRINT`, which is what normally starts it.

`validate_recovery()` has already proven the configuration identity equal before any of this
runs; a mismatch raises `PhysicalResumeRefused` rather than moving the toolhead.

### Timelapse frames across a resume

A small piece of file juggling that exists entirely because of an upstream behaviour:
moonraker-timelapse empties its frame directory on every *"File selected"* - which the resume's
own `M23` triggers - and renumbers from 1, and its render reads `frame000001.jpg` onward without
tolerating a gap.

So `timelapse_set_aside_frames` moves the pre-cut frames to a sibling directory tagged with the
print's path *before* the `M23`, and `_NEBULAOS_PLR_TIMELAPSE_MERGE` - called from `END_PRINT`
and `CANCEL_PRINT` - renumbers both sets back into one sequence. An aside directory belonging to
a different print is stale and is dropped rather than merged into the wrong video.

Two details worth not re-deriving: the merge renames the *new* frames highest-first, so no rename
lands on a frame not yet moved; and after a failed resume the frames deliberately stay aside,
because Moonraker may still be emptying the directory - a retry merges them first.

This is off unless `timelapse_frame_path` is set. The shipped `machine.cfg` sets it to
moonraker-timelapse's own `frame_path`, so on a real printer it is on.

## Adding a module

What the repository's own structure requires:

1. Write `extras/your_module.py` with a `load_config(config)` (or `load_config_prefix`), reading
   its options in `__init__`.
2. Add it to `nebulaos-extensions.json` under `modules` with `"role": "runtime"`.
3. If it relies on an upstream Klipper symbol not already listed, add that to
   `required_klipper_symbols` - that is the preflight's only source of truth.
4. Write `extras/test_your_module.py`. Almost every runtime module has one, and the reference
   cites it.
5. Advance `extensions_version`. If the interface other components depend on changes, advance
   `nebulaos_api_level` too.

**Read the configuration at load time and keep it there.** Every option in this repository is
read in `__init__` or `load_config` and never re-read, which is what makes the single statement
*every option needs a Klipper restart* true. Re-reading config at runtime would quietly break
that promise for your module alone.

## Running the tests

Per module, the way the repository's own CI does it:

```sh
cd <repository root>
python3 -m unittest extras.test_nebulaos_beeper
```

(From the repository root, not from `extras/` - the module path is `extras.<name>`, which is
also how `scripts/ci-tests.sh` invokes it.)

At `178a0c2`: **29 of 31 suites, 912 tests, all passing**, re-run 2026-11-10, plus
`test_nebulaos_compat.ManagedModulePresenceTest` (5 tests), the part of that suite which is a
property of this tree rather than of Klipper. The two not in that run
(`test_nebulaos_compat`, `test_nebulaos_temperature_mcu`) import real Klipper modules and need
the pinned Klipper composed in, exactly as `scripts/ci-tests.sh` documents. They are a
different run, not failures.

Two traps that produce confidently wrong answers:

- **Strip ANSI before matching `OK`.** `unittest` colours that line when it has a TTY, so
  `grep '^OK'` never matches and every suite looks like it failed. CI is unaffected, having no
  TTY. A first run here reported 29 of 29 failing, entirely because of this.
- **Do not use `unittest discover`.** It reports dozens of errors on this suite that are
  rootdir and import artifacts. `test_tmcstatus` fails under discovery and passes perfectly as
  a module.

## What this page does not cover

- **Hardware behaviour.** Nothing here was observed on a printer.
- **The firmware side** - how composition is invoked, where `calibration.cfg` lives, how the
  screen reaches Klipper. See the firmware repository's `docs/reference/`.
- **Upstream Klipper.** Deliberately. If NebulaOS does not change it, upstream's own
  documentation is better than a paraphrase here.
