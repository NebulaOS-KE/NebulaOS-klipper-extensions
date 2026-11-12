# The developer reference moved to this repository's wiki

The reference that used to live in this directory is now the
**[NebulaOS-klipper-extensions wiki](https://github.com/NebulaOS-KE/NebulaOS-klipper-extensions/wiki)**,
kept as Markdown pages in the repository's top-level [`wiki/`](../../wiki) directory and
published from there. One copy, one place to read it.

| Was | Now |
|---|---|
| `docs/reference/ARCHITECTURE.md` | [Architecture](https://github.com/NebulaOS-KE/NebulaOS-klipper-extensions/wiki/Architecture) and [Testing](https://github.com/NebulaOS-KE/NebulaOS-klipper-extensions/wiki/Testing) |
| `docs/reference/MODULES.md` | [Modules](https://github.com/NebulaOS-KE/NebulaOS-klipper-extensions/wiki/Modules) |
| `docs/reference/COMMANDS.md` | [Commands](https://github.com/NebulaOS-KE/NebulaOS-klipper-extensions/wiki/Commands) |
| `docs/reference/COMMANDS-DETAIL.md` | [Commands in detail](https://github.com/NebulaOS-KE/NebulaOS-klipper-extensions/wiki/Commands-Detail) |
| `docs/reference/CONFIG.md` | [Configuration](https://github.com/NebulaOS-KE/NebulaOS-klipper-extensions/wiki/Configuration) |
| `docs/reference/README.md` (this index) | [Home](https://github.com/NebulaOS-KE/NebulaOS-klipper-extensions/wiki) |

`docs/COMPATIBILITY.md` stays where it is: it is the compatibility contract, not part of the
generated reference.

## Regenerating

Four of the wiki pages are generated and must not be edited by hand. From a checkout of the
central NebulaOS repository:

```sh
python3 documentation/agent/tools/gen_extensions_reference.py       <ext> <ext>/wiki
python3 documentation/agent/tools/gen_extensions_commands_detail.py <ext> <ext>/wiki/Commands-Detail.md
python3 documentation/agent/tools/gen_extensions_modules.py         <ext> <ext>/wiki/Modules.md
```

The first writes both `Commands.md` and `Configuration.md`. Regenerate whenever this
repository's tip moves, and re-run the test suite with it.
