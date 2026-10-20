# nebulaos_version.py audit - version-truth collection from real fixture files and a real
# (throwaway) git repository, so this exercises the actual git subprocess path, not a mock.
#
# Run from this directory: python3 -m unittest test_nebulaos_version -v
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import json
import os
import shutil
import subprocess
import tempfile
import unittest

try:
    from . import nebulaos_version
except ImportError:
    import nebulaos_version


class FakePrinter(object):
    def lookup_object(self, name):
        raise KeyError(name)


class FakeConfig(object):
    def __init__(self, values=None):
        self._values = values or {}
        self._printer = FakePrinter()

    def get_printer(self):
        return self._printer

    def get(self, key, default=None):
        return self._values.get(key, default)


def _run_git(repo_dir, *args):
    subprocess.check_call(
        ['git', '-C', repo_dir] + list(args),
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


class VersionCollectionTest(unittest.TestCase):
    def setUp(self):
        self.work = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.work, ignore_errors=True)

    def _make_repo_dir(self):
        # nebulaos_version.py derives its repo root as two directories up
        # from its own __file__ path - build a fixture with that exact
        # klippy/extras/nebulaos_version.py layout so the real (unpatched)
        # path-derivation logic is exercised, not bypassed.
        repo_root = os.path.join(self.work, 'klipper-repo')
        os.makedirs(os.path.join(repo_root, 'klippy', 'extras'))
        return repo_root

    def _instantiate_against(self, repo_root, version_data=None, generation_data=None):
        version_file = os.path.join(self.work, 'nebulaos-version.json')
        generation_file = os.path.join(self.work, 'app-generation.json')
        if version_data is not None:
            with open(version_file, 'w') as f:
                json.dump(version_data, f)
        if generation_data is not None:
            with open(generation_file, 'w') as f:
                json.dump(generation_data, f)

        config = FakeConfig({
            'version_file': version_file,
            'generation_file': generation_file,
        })
        nv = nebulaos_version.NebulaOSVersion.__new__(nebulaos_version.NebulaOSVersion)
        nv.printer = config.get_printer()
        nv.version_file = config.get('version_file')
        nv.generation_file = config.get('generation_file')
        nv.klipper_repo_dir = repo_root
        nv._cached = nv._collect()
        return nv

    def test_reports_build_time_version_fields(self):
        repo_root = self._make_repo_dir()
        _run_git(repo_root, 'init', '-q', '-b', 'master')
        _run_git(repo_root, 'config', 'user.email', 'test@localhost')
        _run_git(repo_root, 'config', 'user.name', 'test')
        with open(os.path.join(repo_root, 'klippy', 'extras', 'placeholder.py'), 'w') as f:
            f.write("# x\n")
        _run_git(repo_root, 'add', '-A')
        _run_git(repo_root, 'commit', '-q', '-m', 'initial')
        expected_sha = subprocess.check_output(
            ['git', '-C', repo_root, 'rev-parse', 'HEAD']).decode().strip()

        nv = self._instantiate_against(repo_root, version_data={
            'firmware_tag': 'nebulaos-canonical-baseline-2026-08-07-5-gdc241c8',
            'firmware_sha': 'dc241c8abc',
            'kernel_sha': '295b7101d751fd888ae39e6f1746a4a940664a5f',
            'guppyscreen_sha': 'be5d372c0d0c693adff3c23adf2655584bb2961e',
            'build_date': '2026-08-08T00:00:00Z',
        }, generation_data={
            'migration_version': 'abc1234567890def',
            'recorded_at': '2026-08-08T00:05:00Z',
        })
        status = nv.get_status(0)

        self.assertEqual(status['firmware_tag'], 'nebulaos-canonical-baseline-2026-08-07-5-gdc241c8')
        self.assertEqual(status['kernel_sha'], '295b7101d751fd888ae39e6f1746a4a940664a5f')
        self.assertEqual(status['guppyscreen_sha'], 'be5d372c0d0c693adff3c23adf2655584bb2961e')
        # An image from before the G82 rename: nebulascreen_sha falls back.
        self.assertEqual(status['nebulascreen_sha'], 'be5d372c0d0c693adff3c23adf2655584bb2961e')
        self.assertEqual(status['app_generation'], 'abc1234567890def')
        self.assertEqual(status['klipper_sha'], expected_sha)
        self.assertFalse(status['klipper_dirty'])

    def test_dirty_klipper_checkout_is_reported_true(self):
        repo_root = self._make_repo_dir()
        _run_git(repo_root, 'init', '-q', '-b', 'master')
        _run_git(repo_root, 'config', 'user.email', 'test@localhost')
        _run_git(repo_root, 'config', 'user.name', 'test')
        tracked = os.path.join(repo_root, 'klippy', 'extras', 'placeholder.py')
        with open(tracked, 'w') as f:
            f.write("# x\n")
        _run_git(repo_root, 'add', '-A')
        _run_git(repo_root, 'commit', '-q', '-m', 'initial')
        with open(tracked, 'a') as f:
            f.write("# a real, unrelated local edit\n")

        nv = self._instantiate_against(repo_root, version_data={}, generation_data={})
        status = nv.get_status(0)
        self.assertTrue(status['klipper_dirty'])

    def test_c_helper_so_drift_alone_does_not_count_as_dirty(self):
        # Same known-safe exception as make-seed-archive.sh/S04nebulaos-
        # migrate: a cross-compiled klippy/chelper/c_helper.so legitimately
        # always differs from git's tracked bytes - must not, by itself,
        # make a genuinely clean checkout report klipper_dirty: true.
        repo_root = self._make_repo_dir()
        _run_git(repo_root, 'init', '-q', '-b', 'master')
        _run_git(repo_root, 'config', 'user.email', 'test@localhost')
        _run_git(repo_root, 'config', 'user.name', 'test')
        os.makedirs(os.path.join(repo_root, 'klippy', 'chelper'))
        chelper = os.path.join(repo_root, 'klippy', 'chelper', 'c_helper.so')
        with open(chelper, 'w') as f:
            f.write("original tracked bytes\n")
        _run_git(repo_root, 'add', '-A')
        _run_git(repo_root, 'commit', '-q', '-m', 'initial')
        with open(chelper, 'w') as f:
            f.write("a different, locally cross-compiled MIPS binary\n")

        nv = self._instantiate_against(repo_root, version_data={}, generation_data={})
        status = nv.get_status(0)
        self.assertFalse(status['klipper_dirty'])

    def test_missing_files_degrade_to_unknown_not_a_crash(self):
        repo_root = os.path.join(self.work, 'not-a-repo-at-all')
        os.makedirs(repo_root)
        # No version_data/generation_data written at all - both files are
        # simply missing, and klipper_repo_dir is not a real git repo
        # either. Must not raise.
        nv = self._instantiate_against(repo_root)
        status = nv.get_status(0)
        self.assertEqual(status['firmware_tag'], 'unknown')
        self.assertEqual(status['app_generation'], 'unknown')
        self.assertEqual(status['klipper_sha'], 'unknown')
        self.assertIsNone(status['klipper_dirty'])
        self.assertEqual(status['nebulascreen_sha'], 'unknown')

    def test_renamed_image_reports_nebulascreen_sha(self):
        repo_root = os.path.join(self.work, 'not-a-repo-either')
        os.makedirs(repo_root)
        nv = self._instantiate_against(repo_root, version_data={
            'nebulascreen_sha': 'c50c60952b481c44f18db2a9c1cee98ac74b2278',
            'guppyscreen_sha': 'c50c60952b481c44f18db2a9c1cee98ac74b2278',
        })
        status = nv.get_status(0)
        self.assertEqual(status['nebulascreen_sha'], 'c50c60952b481c44f18db2a9c1cee98ac74b2278')
        self.assertEqual(status['guppyscreen_sha'], 'c50c60952b481c44f18db2a9c1cee98ac74b2278')


if __name__ == '__main__':
    unittest.main()
