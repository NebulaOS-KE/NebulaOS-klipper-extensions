# nebulaos_version - runtime version-truth printer object, host-side Klipper extra
#
# Clean-Update + Virgin Baseline mission, Phase 6 (2026-08-08, see
# docs/NEBULAOS_PERSISTENT_LIFECYCLE.md). Answers "what is actually running on this printer
# right now" from one place, queryable via Moonraker's ordinary
# /printer/objects/query?nebulaos_version - firmware tag/SHA and kernel/GuppyScreen pins come
# from /opt/nebulaos-version.json (immutable, squashfs-resident, written at build time by
# 04-cross-compile-app-stack.sh); persistent app generation/migration_version come from
# $NEBULAOS_ROOT/system/app-generation.json (written by S04nebulaos-factory-seed/
# S04nebulaos-migrate); Klipper's own commit and dirty state are read live from this checkout's
# real .git directory, since - unlike the other three components - Klipper is the one thing
# whose live state can legitimately differ from what was last recorded (e.g. between an update
# landing and Moonraker's own update_manager reporting it).
#
# Governing rule (the mission's own words): "a healthy system must not depend on dirty git state
# for accepted functionality." This module does not enforce that itself - it has no authority to
# refuse to start Klipper - it exists so that IF the live checkout is ever dirty, that fact is
# visible in one obvious, queryable place (klipper_dirty: true) rather than silently hidden
# behind whatever klipper_sha happens to report. A CI/deployment check (see Phase 8's own build
# verification) is what actually enforces cleanliness before something ships; this module is the
# on-device instrument that makes a violation visible after the fact.
#
# All reads are best-effort and non-fatal by design: a missing or malformed file/git repo must
# never prevent Klipper from starting - this is a diagnostics object, not a safety gate. Every
# field that cannot be determined reports "unknown" (or null for booleans) rather than raising.
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import json
import logging
import os
import subprocess


class NebulaOSVersion:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.version_file = config.get('version_file', '/opt/nebulaos-version.json')
        self.generation_file = config.get(
            'generation_file', '/usr/data/nebulaos/system/app-generation.json')

        # This file's own location is klippy/extras/nebulaos_version.py - the
        # klipper checkout root is two directories up. Computed once, not
        # reconstructed per-query.
        self.klipper_repo_dir = os.path.normpath(
            os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))

        # Static for the life of this Klippy process (nothing here changes
        # without a full restart) - read once at load time rather than on
        # every get_status() poll, which Klipper/Moonraker may call several
        # times a second.
        self._cached = self._collect()

    def _read_json(self, path):
        try:
            with open(path, 'r') as f:
                return json.load(f)
        except (IOError, OSError, ValueError) as e:
            logging.info("nebulaos_version: could not read %s: %s", path, e)
            return {}

    def _git(self, *args):
        try:
            out = subprocess.check_output(
                ['git', '-C', self.klipper_repo_dir] + list(args),
                stderr=subprocess.STDOUT, timeout=5)
            return out.decode('utf-8', 'replace').strip()
        except (OSError, subprocess.CalledProcessError, subprocess.TimeoutExpired) as e:
            logging.info("nebulaos_version: git %s failed: %s", " ".join(args), e)
            return None

    def _klipper_git_state(self):
        sha = self._git('rev-parse', 'HEAD')
        if sha is None:
            return "unknown", None
        # Same known-safe exception as everywhere else in this project
        # (make-seed-archive.sh, S04nebulaos-migrate): the cross-compiled
        # MIPS klippy/chelper/c_helper.so legitimately always differs from
        # git's tracked bytes on every device - excluding it here matches
        # what every other dirty-tree check in this project already does,
        # so a real, unrelated source modification is never masked by it.
        status = self._git('status', '--porcelain', '--',
                            '.', ':!klippy/chelper/c_helper.so')
        dirty = bool(status)
        return sha, dirty

    def _collect(self):
        version = self._read_json(self.version_file)
        generation = self._read_json(self.generation_file)
        klipper_sha, klipper_dirty = self._klipper_git_state()

        return {
            'firmware_tag': version.get('firmware_tag', 'unknown'),
            'firmware_sha': version.get('firmware_sha', 'unknown'),
            'kernel_sha': version.get('kernel_sha', 'unknown'),
            # G82 (Run 15): the screen is NebulaScreen. Images from before the
            # rename only record guppyscreen_sha; both keys stay reported.
            'nebulascreen_sha': version.get(
                'nebulascreen_sha', version.get('guppyscreen_sha', 'unknown')),
            'guppyscreen_sha': version.get('guppyscreen_sha', 'unknown'),
            'build_date': version.get('build_date', 'unknown'),
            'klipper_sha': klipper_sha,
            'klipper_dirty': klipper_dirty,
            'app_generation': generation.get('migration_version', 'unknown'),
            'generation_recorded_at': generation.get('recorded_at', 'unknown'),
        }

    def get_status(self, eventtime):
        return self._cached


def load_config(config):
    return NebulaOSVersion(config)
