#!/usr/bin/env bash
#
# Every test of this repository, the way CI runs them (Run 23):
#
#   scripts/ci-tests.sh <klipper checkout at the firmware's KLIPPER_PIN>
#
# 1. standalone, from the repository root: every suite except the two that
#    import real Klipper modules (README "Tests"), plus the manifest-vs-tree
#    check of test_nebulaos_compat (a property of this tree, not of Klipper's)
# 2. composed, as the firmware image does: extras/ and the manifest copied into
#    a scratch copy of the pinned Klipper's klippy/, then test_nebulaos_compat
#    (the rest of it) and test_nebulaos_temperature_mcu
set -uo pipefail

KLIPPER=${1:?usage: ci-tests.sh <klipper checkout>}
ROOT=$(cd "$(dirname "$0")/.." && pwd)
[ -f "$KLIPPER/klippy/mcu.py" ] || { echo "not a Klipper checkout: $KLIPPER" >&2; exit 2; }
fail=0
run() {    # run <dir> <test id>...
  local dir=$1 what=$2; shift
  [ "$#" -gt 1 ] && what="${what%.*.*} ($# tests)"
  if (cd "$dir" && python3 -m unittest "$@" 2>&1 | tail -1 | grep -q '^OK'); then
    echo "PASS $what"
  else
    echo "FAIL $what"; (cd "$dir" && python3 -m unittest "$@" 2>&1 | tail -30); fail=1
  fi
}

for t in "$ROOT"/extras/test_*.py; do
  n=$(basename "$t" .py)
  case "$n" in test_nebulaos_compat|test_nebulaos_temperature_mcu) continue ;; esac
  run "$ROOT" "extras.$n"
done
run "$ROOT" extras.test_nebulaos_compat.ManagedModulePresenceTest

TREE=$(mktemp -d)
trap 'rm -rf "$TREE"' EXIT
cp -r "$KLIPPER/klippy" "$TREE/"
cp -r "$ROOT"/extras/. "$TREE/klippy/extras/"
cp "$ROOT/nebulaos-extensions.json" "$TREE/klippy/"
compat=$(cd "$TREE/klippy" && python3 - <<'PY'
import unittest
from extras import test_nebulaos_compat as m
names = []
for case in unittest.defaultTestLoader.loadTestsFromModule(m):
    for t in case:
        if type(t).__name__ != "ManagedModulePresenceTest":
            names.append(t.id())
print(" ".join(names))
PY
)
[ -n "$compat" ] || { echo "FAIL: no composed compat tests found"; fail=1; }
run "$TREE/klippy" $compat
run "$TREE/klippy" extras.test_nebulaos_temperature_mcu

[ "$fail" -eq 0 ] && echo "EXTENSIONS_TESTS=PASS" || echo "EXTENSIONS_TESTS=FAIL"
exit "$fail"
