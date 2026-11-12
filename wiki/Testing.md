# Testing

The test suites, what they prove, and two traps that produce confidently wrong answers. Written
against `178a0c2`, the commit NebulaOS 1.0.2 ships.

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
