# NebulaOS-klipper-extensions

The Klipper modules NebulaOS adds to a printer: guided calibration, load-cell probing,
power-loss recovery, the buzzer, TMC autotune, and the rest. **NebulaOS is not a fork of
Klipper** - this repository holds loadable `extras/` modules that Klipper discovers the same way
it discovers its own.

This wiki is the developer documentation. The end-user manual is the
[NebulaOS wiki](https://github.com/NebulaOS-KE/NebulaOS/wiki).

## Which code this describes

| | |
|---|---|
| Commit | **`178a0c2`** |
| Also | the tip of `main` **and** of `production` |
| Ships as | NebulaOS **1.0.2**: `KLIPPER_EXTENSIONS_PIN=178a0c2559de20bc23719471727051162cc53a90`, branch `production`, in firmware `3bdf3ff` (tag `v1.0.2`) |

All three agree, so this wiki describes the shipped code with no pin gap to carry. That is
unusual and not guaranteed to last: the firmware manifest is the shipping authority.

## The pages

| Page | What it is |
|---|---|
| [Architecture](Architecture) | **Start here.** What this repository is, how its code reaches a running printer, the compatibility contract, how a calibration actually runs, how state is persisted, and why the physical resume's order is what it is |
| [Modules](Modules) | One section per file in `extras/`: what it is, the commands it registers, the options it reads, which tests refer to it, and who wrote it when it is vendored |
| [Commands](Commands) | Summary table: **50** commands with module, kind, test coverage and description |
| [Commands in detail](Commands-Detail) | One section per command: handler, help text, parameters with types, defaults and valid ranges, errors raised, side effects, example invocation |
| [Configuration](Configuration) | **156** configuration options, each with the literal default and the constraints the source gives it |
| [Testing](Testing) | How to run the suites, what passes at this commit, and two traps that produce confidently wrong answers |

[Architecture](Architecture) is hand-written: explanation cannot be extracted. Every factual
claim in it names the file it came from, so any sentence can be re-checked.

The other four are **generated** from source by tools in the central NebulaOS repository
(`documentation/agent/tools/gen_extensions_*.py`), parsed with Python's `ast`. **Do not edit
them by hand** - regeneration overwrites, and the point of generating them is that no default is
transcribed by a human.

## How the code reaches a printer

Host Klipper is pristine upstream
[`Klipper3d/klipper`](https://github.com/Klipper3d/klipper), pinned by the firmware build. The
firmware composes this repository's `extras/` into the Klipper checkout's `klippy/extras/` at
boot, driven by `nebulaos-extensions.json`.

That distinction matters when reading this wiki: upstream Klipper's own documentation still
applies to everything upstream does, and is the first place to look for anything not listed
here.

## Attribution: three kinds of command

The command reference keeps these apart, because getting it wrong misleads anyone arriving from
stock Klipper.

1. **NebulaOS commands** - registered here, do not exist upstream.
2. **Commands that replace upstream behaviour** - registered with a `None` handler to
   de-register the upstream command, then re-registered here. `SET_PIN` and
   `Z_OFFSET_APPLY_PROBE` are the two. They behave differently from stock Klipper, and that is
   deliberate.
3. **Public names that are not here at all** - several user-facing commands are `gcode_macro`
   definitions in the firmware's own configuration, which call the underscore-prefixed command
   registered here. `NEBULAOS_BED_TRAM` is one: this repository registers
   `_NEBULAOS_BED_TRAM`, and the public macro lives in the firmware's
   `etc/nebulaos/klipper/calibration.cfg`, forwarding `{rawparams}`. A reference built only
   from this repository cannot see those, and [Commands](Commands) says so rather than
   presenting itself as the complete list of what a user can type.

## What the detail pages deliberately do not contain

Each command section says so rather than leaving silence to be read as reassurance:

- **Expected output.** Most handlers reply through `respond_info` with runtime values, so the
  text depends on machine state. An example response would be invented.
- **Preconditions** beyond the guards visible in the handler. A command may also depend on
  calibration state or on configuration the handler does not check.
- **Safety warnings.** Whether a command is dangerous is a judgement about physical
  consequence, not a fact in the file. Commands are instead flagged **Runs G-code: yes**, the
  closest honest proxy - it means the command can move the machine or change temperatures.
- **Hardware behaviour.** Nothing in this wiki was run on a printer by its author.

## Verification state

| | |
|---|---|
| Source commit | `178a0c2` |
| Shipped by firmware | **yes**, at 1.0.2 |
| Test suite | 29 of 31 suites, **912 tests, all passing** (re-run at this commit on 2026-11-10), plus `test_nebulaos_compat.ManagedModulePresenceTest` (5) |
| Modules with a test | 21 of 30 |
| Hardware verification | **none** |

**No claim here is hardware-verified.** Everything is established from the source and from the
repository's own tests. A passing test proves the logic is exercised; it does not prove a
printer responds as expected. One exception is labelled as what it is:
`_NEBULAOS_NOZZLE_CLEAN` is step 3 of the auto-calibration sequence, and the project owner
reports it has run on the development printer several times - the call site is code-verified,
the runs are the owner's report, and those are different kinds of evidence.

## Related

- [NebulaOS-firmware wiki](https://github.com/NebulaOS-KE/NebulaOS-firmware/wiki) - the image,
  the boot composition and the Klipper configuration values these commands run with
- [Repository map](https://github.com/NebulaOS-KE/NebulaOS-firmware/wiki/Repository-Map)
- [`docs/COMPATIBILITY.md`](https://github.com/NebulaOS-KE/NebulaOS-klipper-extensions/blob/main/docs/COMPATIBILITY.md)
  and [`VENDORED.md`](https://github.com/NebulaOS-KE/NebulaOS-klipper-extensions/blob/main/VENDORED.md)
  in this repository - the compatibility contract and the provenance of vendored modules
