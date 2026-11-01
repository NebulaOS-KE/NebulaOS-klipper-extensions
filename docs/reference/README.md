# NebulaOS Klipper extensions - developer reference

Technical reference for `NebulaOS-klipper-extensions`, at commit `a2e7634`.

This repository's GitHub wiki is **disabled**, so this reference lives in the repository
itself. That is the better place anyway: the mission treats Git as authoritative and a wiki as
a publishing destination.

## The pages

| Page | What it is |
|---|---|
| [COMMANDS.md](COMMANDS.md) | Every G-code command these extensions register, with its module, kind, test coverage and description, plus the parameters each module reads |
| [CONFIG.md](CONFIG.md) | Every configuration option, with its literal default and constraints |

Both are **generated** by `documentation/agent/tools/gen_extensions_reference.py` in the
central NebulaOS repository, parsed with Python's `ast`. Do not edit them by hand - a
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

## Verification state

| | |
|---|---|
| Source commit | `a2e7634` |
| Shipped by firmware | **yes** - the manifest pins exactly this commit |
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
