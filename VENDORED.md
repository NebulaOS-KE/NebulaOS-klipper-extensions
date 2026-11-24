# Vendored modules and provenance

Not everything in this repository is NebulaOS's own work, and this file says exactly which
parts are not.

The rule it exists to enforce is simple: **community-authored code is never presented as
NebulaOS's own.** Where a file arrived carrying its author's copyright header, that header is
preserved untouched. Where a file arrived with no header at all, the real author was traced
through the upstream project's own git history — not guessed at — and an accurate header was
added.

Everything here is GPLv3, matching [`LICENSE`](LICENSE). Individual file headers, not this
table, are authoritative for the file they appear in.

## Summary

| Module | Author | Upstream | Header | NebulaOS delta |
|---|---|---|---|---|
| `extras/gcode_shell_command.py` | Eric Callahan, © 2019 | via `pellcorp/klipper` | present upstream, preserved | none beyond the Pellcorp lineage carried in this repo's history |
| `extras/virtual_pins.py` | Pedro Lamas, © 2023 | via `pellcorp/klipper` | present upstream, preserved | none |
| `extras/calibrate_shaper_config.py` | ballaswag, © 2023 | `ballaswag/guppyscreen` | **absent upstream — added** | header only |
| `extras/guppy_config_helper.py` | ballaswag, © 2024 | `ballaswag/guppyscreen` | **absent upstream — added**; **no longer shipped** (removed from `extras/`, absent from `nebulaos-extensions.json`) | header only |
| `extras/guppy_module_loader.py` | ballaswag, © 2024 | `ballaswag/guppyscreen` | **absent upstream — added**; **no longer shipped** (removed from `extras/`, absent from `nebulaos-extensions.json`) | header only |
| `extras/tmcstatus.py` | ballaswag, © 2024 | `ballaswag/guppyscreen` | **absent upstream — added** | header, plus a `klippy:connect` deferral fix |
| `extras/nebulaos_temperature_mcu.py` | Kevin O'Connor © 2020-2024 (base), NebulaOS © 2026 (GD32 curves) | `Klipper3d/klipper` | derived, both credited | subclass + three GD32 calibration curves |
| `extras/autotune_tmc.py`, `extras/motor_constants.py`, `extras/motor_database.cfg` | Andrew McGregor and contributors, © 2023-2026 | `andrewmcgr/klipper_tmc_autotune` @ `b6c7cfa98c2ef880812d5279a9117cbe67d6d4d5` | **absent upstream — added** | header only (code unmodified). Upstream, not OpenKE's `evgarthub/klipper_tmc_autotune_k1` fork: that fork only back-ports `TMCtstepHelper` for Creality's old stock Klipper, and NebulaOS runs current upstream Klipper. Both have the KE motor `creality-42-40`, which the KE uses on X, Y and Z (#34; NebulaOS said `creality-42-34` for X/Y before 1.0.5). Loaded only when an `[autotune_tmc <stepper>]` section exists (decision D8, 2026-10-10) |
| `extras/bl24c16f.py` | Eric Callahan, © 2020 | community Klipper extra, shipped by Creality in the KE's stock firmware | present upstream, preserved | none — byte-identical to the copy pulled from the printer's own stock rootfs partition |

Two of the rows above, `guppy_config_helper.py` and `guppy_module_loader.py`, are **historical**:
those files were removed from `extras/` and are not in `nebulaos-extensions.json`. Their
attribution is kept here deliberately — an attribution record should not lose an author because
the file stopped shipping. Six vendored community modules ship today: `tmcstatus`,
`gcode_shell_command`, `virtual_pins`, `calibrate_shaper_config`, `bl24c16f`, and TMC
autotune (`autotune_tmc` + `motor_constants` + `motor_database.cfg`).

Everything else in `extras/` — `z_compensate`, the nozzle-contact/Z-offset/calibration set,
power-loss recovery, `nebulaos_version`, `nebulaos_compat`, and the test suite — is NebulaOS's
own work. (The PRTouch runtime modules this sentence once counted were deleted in Phase 1.8B;
only the unrelated offline test scaffolding `prtouch_test_support.py` still carries the name.)

## Per-module detail

### `gcode_shell_command.py` — Eric Callahan, © 2019

Runs a shell command from gcode. GPLv3 header present in the original and preserved verbatim.

Not in mainline Klipper, so it cannot be dropped by waiting for upstream. It reached NebulaOS
through the Pellcorp base fork, and this repository's filtered history carries the real
Pellcorp commits — including Jason Pell's `02799c2` ("expand variables in gcode shell
command"), which is the one line by which this copy differs from the copy shipped inside
`NebulaOS-guppyscreen/k1/k1_mods/`. That delta is inherited deliberately, not silently: the
`os.path.expandvars()` call is load-bearing for the shipped macro set.

Live in five sections of the shipped config, and invoked from GuppyScreen's compiled C++
(`inputshaper_panel.cpp`, `belts_calibration_panel.cpp`) as a Klipper gcode command. That is
why it cannot simply be replaced by Moonraker's own shell-command machinery — doing so means
changing GuppyScreen.

### `virtual_pins.py` — Pedro Lamas, © 2023

Virtual pin support. GPLv3 header present in the original and preserved verbatim. Inherited
from the Pellcorp base. Not in mainline. Live via `[virtual_pins]` and
`pin: virtual_pin:BED_WARP_STABILISE_pin` in the shipped config.

### `calibrate_shaper_config.py` — ballaswag, © 2023

Persists input-shaper calibration results to `printer.cfg`, behind `SAVE_INPUT_SHAPER`.

**Had no copyright or licence header of any kind.** Traced to `ballaswag/guppyscreen` commit
`4a5cf94` ("initial guppy screen code commit. code at 0.0.11-beta", 2023-12-12), whose
repository is GPLv3. A header naming ballaswag was added; the code is otherwise byte-identical
to the upstream copy.

Note that this is **not** a Pellcorp inheritance, contrary to an earlier NebulaOS audit that
grouped all five community extras together. NebulaOS added it itself, and the real upstream is
GuppyScreen.

### `guppy_config_helper.py` — ballaswag, © 2024 — *historical, no longer shipped*

`_GUPPY_SAVE_CONFIG` / `_GUPPY_DELETE_CONFIG`, driven from GuppyScreen's TMC tuning panel.
**Had no header.** Traced to `ballaswag/guppyscreen` commit `a20edd7` ("add guppy config
helper", 2024-01-17). Header added; code otherwise byte-identical.

### `guppy_module_loader.py` — ballaswag, © 2024 — *historical, no longer shipped*

`_GUPPY_LOAD_MODULE` / `_GUPPY_UNLOAD_MODULE`. **Had no header.** Traced to
`ballaswag/guppyscreen` commit `1d7e584` (2024-02-01). Header added; code otherwise
byte-identical.

The name invites a wrong assumption worth correcting here, because it matters to this
repository's architecture: this is **not** a general external-module loading mechanism and
does not help Klipper load extras from outside `klippy/extras/`. It registers two gcode
commands that call `printer.load_object()` and `printer.objects.pop()`, so it routes through
Klipper's own loader and hits the same filesystem gate as everything else. It *depends on*
NebulaOS's symlink composition rather than relieving it — which also means `tmcstatus.py` must
be composed in like every other module, even though no config section declares it.

### `tmcstatus.py` — ballaswag, © 2024, with a NebulaOS fix

TMC driver status reporting for GuppyScreen's TMC panel, loaded on demand via
`_GUPPY_LOAD_MODULE SECTION=tmcstatus`. **Had no header.** Traced to `ballaswag/guppyscreen`
commit `1d7e584` (2024-02-01). Header added.

**This module was previously misclassified as NebulaOS-authored** (in
`_project/missions/2026-08-phase1-klipper-no-fork-analysis.md` §4.1). It is not. Direct
comparison against the upstream GuppyScreen tree shows the two files differ by exactly one
change, which is NebulaOS's: `handle_connect()` is registered on the `klippy:connect` event
rather than called directly from `__init__`. The original's direct call depends on
`printer.cfg` section order and broke a real boot on this printer with `Unknown config object
'tmc2208 stepper_x'`, because `[tmcstatus]` loaded before the TMC driver sections it looks up.

### `nebulaos_temperature_mcu.py` — derived from Klipper

MCU die-temperature support for the GD32 chips this printer uses. Subclasses Klipper's own
`klippy/extras/temperature_mcu.py` (© 2020-2024 Kevin O'Connor, GPLv3) rather than copying it,
so the derivation is a live import and upstream fixes are inherited automatically. Both
copyrights appear in the file header.

The original content is the three GD32 calibration curves, carried forward historically from the
retired `NebulaOS-klipper` fork's own `temperature_mcu.py`. (That fork is not the shipped host
Klipper and has no relationship to `KLIPPER_PIN`, which points at official upstream
`Klipper3d/klipper`; the curves simply originated there before the no-fork migration.)

Upstream contribution is worth doing independently of this repository: the curves are ~15
lines and mechanically identical in form to the entries mainline already carries. If
`Klipper3d/klipper` accepts them, this module can go away entirely.

### `bl24c16f.py` — Eric Callahan, © 2020

I2C EEPROM driver for the BL24C16F chip (8 selectable I2C sub-addresses, 256-byte pages each,
2KB total) wired directly to this printer's SoC. GPLv3 header present in the original and
preserved verbatim.

Not in mainline Klipper (checked directly against the `Klipper3d/klipper` tree at our own
pinned commit — absent). Pulled read-only from the KE's own stock firmware
(`/usr/share/klipper/klippy/extras/bl24c16f.py` on the inactive stock rootfs partition), not
from any upstream Klipper PR or fork — Creality ships this file, but did not author it, and did
not patch it: byte-for-byte identical to Eric Callahan's original, confirmed by diff.

Phase 1.9A vendors this purely as the generic hardware driver (`EEPROM_READ`/`EEPROM_WRITE_*`/
`EEPROM_DEBUG_*` gcode commands, `read_reg`/`write_reg`). The class also carries a handful of
methods with power-loss-recovery-shaped names (`eepromReadHeader`, `eepromReadBody`,
`setEepromDisable`, `checkEepromFirstEnable`) that are part of the original author's own file,
not a NebulaOS addition — they are vendored as-is because "verbatim" is the safer choice than
selectively stripping methods from a community-authored driver, but nothing in Phase 1.9A calls
them. Power-loss recovery itself (a periodic checkpoint writer, a boot-time resume flow) is
explicitly out of scope for this phase and lives in a later one; see
`_project/missions/phase1.9-host-mcu-accelerometer-plr-analysis.md` for that design.

## Why vendor these at all

Every one of these seven modules is absent from mainline Klipper, so none of them can be
dropped by waiting for upstream, and each is live in the shipped configuration, invoked from
GuppyScreen's compiled code, or (bl24c16f.py) needed to talk to real, physically-present
hardware.

Pinning "the original author's repository" instead would be fiction for four of them: their
real source is GuppyScreen, whose copies already ship duplicated inside
`NebulaOS-guppyscreen/k1/k1_mods/`. And `gcode_shell_command.py` carries a deliberate
one-line lineage delta that must be preserved knowingly rather than inherited by accident.
`bl24c16f.py`'s real source is neither GuppyScreen nor a Klipper PR we can point at — it was
pulled from the printer's own stock firmware, the only place NebulaOS actually has it.
Vendoring with explicit provenance is the honest version of a situation that already existed.

## Filename collision risk

The vendored names are kept as-is for now — `gcode_shell_command`, `virtual_pins`,
`calibrate_shaper_config`, `tmcstatus`, `bl24c16f`. GuppyScreen's compiled C++ hardcodes several
of the gcode commands the first four register, so renaming those is a coordinated change across
repositories, not a free rename; `bl24c16f`'s own name is dictated by the physical chip it
drives and by `printer.cfg`'s `[bl24c16f]` section. (`guppy_config_helper` and
`guppy_module_loader` were also on this list; both have since been removed from `extras/`.)

That leaves a real, if unlikely, hazard: if mainline Klipper ever ships a file at one of these
paths, git will silently replace NebulaOS's symlink with upstream's regular file, and the
vendored module is shadowed with no error anywhere. `gcode_shell_command` and `virtual_pins`
are exactly the kind of module mainline could adopt — and `bl24c16f` arguably more so than
either: its real author, Eric Callahan, is an active Klipper maintainer, and this is a
plausible file for `Klipper3d/klipper` to accept directly someday, at which point this vendored
copy should simply be deleted in favor of upstream's.

The mitigation is mandatory and lives in the platform layer, not here: the composition step
must verify that every managed destination path is still a symlink resolving inside this
repository, and refuse to activate otherwise. That requirement is declared in
`nebulaos-extensions.json`'s `composition.require_symlink_resolving_inside_source`. See
[`docs/COMPATIBILITY.md`](docs/COMPATIBILITY.md).
