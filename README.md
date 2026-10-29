# NebulaOS Klipper extensions

The Klipper extras that [NebulaOS](https://github.com/NebulaOS-KE/NebulaOS-firmware) adds to the
Creality Ender-3 V3 KE. They are composed into a pristine upstream
[`Klipper3d/klipper`](https://github.com/Klipper3d/klipper) checkout at boot.

**This is not a Klipper fork.** There is no copy of Klipper here and no patch to any Klipper core
file. NebulaOS runs unmodified upstream Klipper (pinned as `KLIPPER_PIN` in the firmware's
`manifests/dependencies.conf`) and links these modules into it. The retired
`coreflake1/NebulaOS-klipper` fork is history only. This repository was split out of it with
`git filter-repo`, so `git log`/`git blame` still work, but commit SHAs differ.

## Composition contract

[`nebulaos-extensions.json`](nebulaos-extensions.json) is the authoritative, machine-readable
contract. Its `composition` block:

| Key | Value | Meaning |
|---|---|---|
| `source_dir` | `extras` | where the modules live in this repository |
| `destination_dir` | `klippy/extras` | where they appear in the Klipper checkout |
| `link_type` | `symlink` | one symlink per module, pointing back into this repository |
| `exclude_file` | `.git/info/exclude` | the links are listed here, so the Klipper checkout stays clean in `git status` |
| `marker_file` | `.nebulaos-composed` | generation marker written last by the platform; it only lets an unchanged composition be skipped, verification runs every time |
| `require_symlink_resolving_inside_source` | `true` | a regular file at a managed path (e.g. upstream adding a module with the same name) is a hard error, never overwritten |

The rest of the manifest: `compat_schema_version` 1, `nebulaos_api_level` 1,
`extensions_version` `0.9.0`, `required_klipper_symbols` (the Klipper functions these modules
call), `forbidden_klipper_symbols` (currently empty), `sensor_types` (`nebulaos_temperature_mcu`)
and `chelper` (the prebuilt `c_helper.so` must be newer than every chelper source; the platform
decides this and writes `.nebulaos-chelper-verdict.json`).

Composition itself is done by the firmware (`/etc/nebulaos-klipper-compose.sh`), not by anything
in this repository. This repository is never written to at runtime. Inside Klippy,
**`[nebulaos_compat]`** is the preflight. It should be the first NebulaOS section in `printer.cfg`.
It checks the manifest schema, that every declared module is present, that every required Klipper
symbol exists, that every declared sensor type is registered, the composition integrity (second
layer) and the platform's chelper verdict. Any failure stops Klipper with a specific message.
There is no degraded mode. Compatibility is checked by symbol, not by Klipper commit.
Full contract: [`docs/COMPATIBILITY.md`](docs/COMPATIBILITY.md).

## Branches and releases

- `main` is development. Work branches merge into it.
- `production` is what images ship and what Moonraker's update manager follows
  (`[update_manager nebulaos_klipper_extensions]`, `primary_branch: production`). The firmware
  pins its exact commit as `KLIPPER_EXTENSIONS_PIN` (`KLIPPER_EXTENSIONS_BRANCH=production`).
  `production` is fast-forwarded to the pinned commit.

On a printer, Klipper and this repository are updated and rolled back as a pair by the NebulaOS
update supervisor. See `docs/NEBULAOS_EXTENSIONS_BRANCH_STRATEGY.md` in the firmware repository.

## Modules

`nebulaos-extensions.json` lists 60 managed files: 30 `role: runtime` and 30 `role: test`. All of
them are composed (the tests import runtime modules relative to the `extras` package). Runtime
modules:

| Module | Purpose |
|---|---|
| **Platform** | |
| `nebulaos_compat` | Startup compatibility preflight (above) |
| `nebulaos_version` | `nebulaos_version` printer object: firmware, component pins and live Klipper commit |
| `nebulaos_temperature_mcu` | MCU die temperature for the KE's GD32 chips (subclasses upstream `temperature_mcu`) |
| `nebulaos_memory` | Klippy memory policy: resets OOM priority for forked helpers; optional `mlockall` |
| `nebulaos_beeper` | Owner of the KE buzzer (kernel PWM channel `/sys/class/pwm/pwmchip0/pwm3`): `NEBULAOS_BEEP`, `NEBULAOS_PLAY_TUNE` (RTTTL / `songs.conf`), `NEBULAOS_BEEPER_STOP`, API endpoint `nebulaos_beeper/click` for the screen's touch click. Sounds start at the print position and never hold the G-code queue (G108) |
| **Z offset, contact and calibration** (BLTouch/CR-Touch stays the global probe; the load cell is a secondary contact reference) | |
| `nebulaos_calibration` | `[nebulaos_calibration]` coordinator for the `NEBULAOS_*` calibration commands (Z offset, auto-calibrate sequence) |
| `nebulaos_calibration_journal` | Persistent JSON journal of calibration workflow progress |
| `nebulaos_shaper_graph` | Not a Klipper module: script that `nebulaos_calibration` runs as a nice'd child process after each Input Shaper axis - draws the measured vibration and the chosen shaper from upstream's CSV (screen PNG 480x272 next to the journal, 1200x700 in `config/input_shaper/` for Mainsail, G109) |
| `nebulaos_load_cell_calibration` | Guided load-cell calibration with a weighed water bottle (G86); `NEBULAOS_LOAD_CELL_MODE` / `NEBULAOS_LOAD_CELL_AUTO_CHECK` (G99) |
| `nebulaos_load_cell_mode` | The persisted load-cell contact choice auto (default) / manual and the last automatic check (G99, own JSON state file) |
| `nebulaos_bed_tram` | Guided bed tramming, one corner instruction at a time (G87) |
| `nebulaos_z_offset_probe` | Nozzle load cell (HX711) as a contact probe for Z-offset calibration only |
| `nebulaos_probe_pair` | Pairs a BLTouch reading with a nozzle contact at the same point to derive the Z offset |
| `nebulaos_raw_contact` | Weight-free (automatic) load-cell contact detection (G83b/G99): qualified profile `ke-stock-v1` (6400 counts, polarity -1), the per-mode arming rule for every touch |
| `z_compensate` | Per-print auto Z offset, compatible with the stock `[z_compensate]` section |
| `nozzle_clear` | Nozzle wipe routine using the load-cell contact probe |
| **Power-loss recovery** | |
| `nebulaos_power_loss_recovery` | Power-loss recovery on public Klipper interfaces (EEPROM record + JSON sidecar) |
| `nebulaos_plr_journal` | EEPROM journal codec for power-loss recovery (pure stdlib) |
| **Drivers and tuning** | |
| `nebulaos_tmc_autotune` | Stages `[autotune_tmc <stepper>]` sections for the screen's TMC Autotune panel |
| `autotune_tmc`, `motor_constants`, `motor_database.cfg` | Vendored Klipper TMC Autotune (andrewmcgr), loaded only when an `[autotune_tmc]` section exists |
| `tmcstatus` | TMC driver status for the screen app's TMC panel (vendored, ballaswag) |
| `calibrate_shaper_config` | `SAVE_INPUT_SHAPER`: persists input-shaper results (vendored, ballaswag) |
| `bl24c16f` | I2C driver for the BL24C16F EEPROM (vendored, Eric Callahan) |
| `gcode_shell_command` | Run a shell command from G-code (vendored, Eric Callahan); the firmware's camera quality macros still use it |
| `virtual_pins` | Virtual pins (vendored, Pedro Lamas) |
| **Test infrastructure** | |
| `prtouch_test_support` | Offline fakes (config, MCU, toolhead, reactor) shared by tests. The name is legacy: the PRTouch runtime was removed (see `extras/PRTOUCH_REMOVAL_PLAN.md`) |

`role` is a composition attribute: `runtime` files must be composed for Klipper to start.
Klipper imports an extra only when a config section names it.

## Tests

From the repository root:

```sh
python3 -m unittest extras.test_nebulaos_bed_tram -v
for t in extras/test_*.py; do python3 -m unittest "extras.$(basename "$t" .py)"; done
```

The suites use fakes and need neither a printer nor a Klipper checkout. There are two
exceptions: `test_nebulaos_compat` and `test_nebulaos_temperature_mcu` import real Klipper
modules, so they **fail when run standalone**. Run them inside a composed Klipper tree (from
`klippy/`: `python3 -m unittest extras.test_nebulaos_compat`).

## License

GPLv3, see [`LICENSE`](LICENSE). Vendored upstream files keep their original copyright and
licence headers, and those headers are authoritative for their files. Per-file author, upstream
source and any NebulaOS change are listed in [`VENDORED.md`](VENDORED.md).
