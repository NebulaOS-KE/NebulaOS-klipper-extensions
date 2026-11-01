# NebulaOS Klipper extensions - developer reference

Technical reference for `NebulaOS-klipper-extensions`, at commit `ed99372`.

This repository's GitHub wiki is **disabled**, so this reference lives in the repository
itself. That is the better place anyway: the mission treats Git as authoritative and a wiki as
a publishing destination.

## The pages

| Page | What it is |
|---|---|
| [ARCHITECTURE.md](ARCHITECTURE.md) | **Start here.** What this repository is, how its code reaches a running printer, the compatibility contract, how a calibration actually runs, how state is persisted |
| [COMMANDS.md](COMMANDS.md) | Summary table: every command with its module, kind, test coverage and description |
| [COMMANDS-DETAIL.md](COMMANDS-DETAIL.md) | One section per command: handler, help text, parameters with types/defaults/valid ranges, errors raised, side effects, example invocation |
| [CONFIG.md](CONFIG.md) | Every configuration option, with its literal default and constraints |

`ARCHITECTURE.md` is hand-written: explanation cannot be extracted. Every factual claim in it
names the file it came from, so any sentence can be re-checked.

The other three are **generated** by tools in the central NebulaOS repository
(`gen_extensions_reference.py`, `gen_extensions_commands_detail.py`), parsed with Python's
`ast`. Do not edit them by hand - a
regeneration will overwrite the edit, and the point of generating them is that no default is
transcribed by a human.

## What this repository is, and is not

**NebulaOS is not a fork of Klipper.** Host Klipper is pristine upstream
[`Klipper3d/klipper`](https://github.com/Klipper3d/klipper), pinned by the firmware build. This
repository holds **loadable `extras/` modules** that Klipper discovers the same way it
discovers its own: the firmware composes `extras/` into the Klipper checkout's
`klippy/extras/` at boot, driven by `nebulaos-extensions.json`.

That distinction matters when reading this reference. Upstream Klipper's own documentation
still applies to everything upstream does, and should be the first place you look for anything
not listed here.

## Attribution: three kinds of command

The command reference separates these, because getting it wrong misleads anyone arriving from
stock Klipper:

1. **NebulaOS commands** - registered here, do not exist upstream.
2. **Commands that replace upstream behaviour** - registered with a `None` handler to
   de-register the upstream command, then re-registered here. `SET_PIN` and
   `Z_OFFSET_APPLY_PROBE` are the two. They behave differently from stock Klipper, and that is
   deliberate.
3. **Public names that are not here at all** - several user-facing commands are `gcode_macro`
   definitions in the firmware's own configuration, which call the underscore-prefixed command
   registered here. `NEBULAOS_BED_TRAM` is one: this repository registers
   `_NEBULAOS_BED_TRAM`. A reference built only from this repository cannot see those, and
   `COMMANDS.md` says so rather than presenting itself as the complete list of what a user can
   type.

## What the detail pages do not contain

`COMMANDS-DETAIL.md` derives everything from the handler the registration points at, so a
parameter cannot be attributed to the wrong command. Four things the mission asks for are
deliberately absent, and each command section says so rather than leaving silence to be read
as reassurance:

- **Expected output.** Most handlers reply through `respond_info` with runtime values, so the
  text depends on machine state. An example response would be invented.
- **Preconditions** beyond the guards visible in the handler. A command may also depend on
  calibration state or configuration the handler does not check.
- **Safety warnings.** Whether a command is dangerous is a judgement about physical
  consequence, not a fact in the file. Commands are instead flagged **Runs G-code: yes**,
  which is the closest honest proxy - it means the command can move the machine or change
  temperatures.
- **Hardware behaviour.** Nothing here was run on a printer.

## `nozzle_clear` is live and has been run on hardware

Worth stating, because the flag name inverts and reads like its opposite:
`HARDWARE_BEHAVIOR_BLOCKED = False` means the routine is **not** blocked and does run. The guard
raises only when it is `True`.

`_NEBULAOS_NOZZLE_CLEAN` is step 3 of the auto-calibration sequence in
`nebulaos_calibration.py`, so anyone running auto calibration exercises it. The project owner
reports it has run on the development printer several times.

The module's header previously stated it had never been exercised on hardware. That note was
stale and has been corrected in the source at `ed99372`; disregard it if you are reading an
older checkout.

As elsewhere here: the call site is code-verified, while the hardware runs are the owner's
report rather than this documentation's observation. Those are different kinds of evidence and
this reference keeps them apart.

## Verification state

| | |
|---|---|
| Source commit | `ed99372` |
| Shipped by firmware | **yes** - the firmware manifest advanced to `ed99372` in firmware `832314d`, so this reference describes exactly what ships |
| Test suite | 29 of 31 suites, **899 tests, all passing** |
| Modules with a test | 21 of 30 |
| Hardware verification | **none** - see below |

The two suites not in that run (`test_nebulaos_compat`,
`test_nebulaos_temperature_mcu`) import real Klipper modules and need the pinned Klipper
composed in, exactly as `scripts/ci-tests.sh` documents. They are a different run, not
failures.

**No claim here is hardware-verified.** Everything is established from the source and from the
repository's own tests. A passing test proves the logic is exercised; it does not prove a
printer responds as expected.

## Running the tests

The way the repository's own CI does it, per-module:

```sh
cd extras
python3 -m unittest extras.test_nebulaos_beeper
```

Two traps, both of which produced wrong answers here before being understood:

- **Strip ANSI before matching `OK`.** `unittest` colours that line when it has a TTY, so
  `grep '^OK'` never matches and every suite looks like it failed. CI is unaffected because CI
  has no TTY.
- **Do not use `unittest discover`.** It reports dozens of errors on this suite that are
  rootdir and import artifacts - `test_tmcstatus` fails under discovery and passes perfectly
  as a module.

The full sequence, including the two Klipper-dependent suites, is `scripts/ci-tests.sh <klipper
checkout at the firmware's KLIPPER_PIN>`.

## Regenerating this reference

From the central NebulaOS repository:

```sh
python3 documentation/agent/tools/gen_extensions_reference.py \
    <path-to-NebulaOS-klipper-extensions> <path-to>/docs/reference
```

Regenerate whenever this repository's tip moves, and re-run the test suite with it.
