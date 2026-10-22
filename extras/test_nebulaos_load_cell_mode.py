# Tests for extras/nebulaos_load_cell_mode.py (G99): the persisted
# auto|manual choice and the last automatic check. A missing file is the
# default (auto, never checked); a corrupt or implausible file is the default
# plus a warning - never an exception (Klipper must always start).
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import json
import os
import shutil
import tempfile
import unittest

from . import nebulaos_load_cell_mode as lcm


class StateFileTest(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.dir, True)
        self.path = os.path.join(self.dir, 'calibration', 'load_cell_mode.json')

    def _write_raw(self, text):
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        with open(self.path, 'w') as fh:
            fh.write(text)

    def test_missing_file_is_auto_never_checked_without_warning(self):
        state, warning = lcm.read_state(self.path)
        self.assertEqual(warning, '')
        self.assertEqual(state, lcm.default_state())
        self.assertEqual(state['mode'], 'auto')
        self.assertEqual(state['auto_check'], 'never')

    def test_round_trip_creates_the_directory(self):
        state = lcm.default_state()
        state.update(mode='manual', auto_check='ok', reference_counts=-240426,
                     checked_at=1761100000.5, profile_id='ke-stock-v1')
        lcm.write_state(self.path, state)
        read, warning = lcm.read_state(self.path)
        self.assertEqual(warning, '')
        self.assertEqual(read, state)
        self.assertFalse(os.path.exists(self.path + '.tmp'))
        with open(self.path) as fh:
            self.assertEqual(json.load(fh)['schema'], lcm.SCHEMA)

    def test_failed_check_keeps_its_reason_and_no_reference(self):
        state = lcm.default_state()
        state.update(auto_check='failed', auto_check_reason='stuck',
                     reference_counts=123, checked_at=5.)
        lcm.write_state(self.path, state)
        read, _ = lcm.read_state(self.path)
        self.assertEqual(read['auto_check'], 'failed')
        self.assertEqual(read['auto_check_reason'], 'stuck')
        self.assertIsNone(read['reference_counts'])

    def test_corrupt_files_are_auto_with_a_warning(self):
        for text in ('{not json', '', '[1, 2]', '"auto"', 'null',
                     '[' * 100000):
            self._write_raw(text)
            state, warning = lcm.read_state(self.path)
            self.assertEqual(state, lcm.default_state(), text[:20])
            self.assertTrue(warning, text[:20])

    def test_unknown_mode_is_auto_with_a_warning_rest_kept(self):
        self._write_raw(json.dumps({'mode': 'turbo', 'auto_check': 'ok',
                                    'reference_counts': -240000}))
        state, warning = lcm.read_state(self.path)
        self.assertEqual(state['mode'], 'auto')
        self.assertEqual(state['reference_counts'], -240000)
        self.assertIn('turbo', warning)

    def test_ok_without_a_usable_reference_is_not_ok(self):
        for ref in (None, 'x', 1.5, True, 1 << 30, -(1 << 30)):
            self._write_raw(json.dumps({'mode': 'auto', 'auto_check': 'ok',
                                        'reference_counts': ref}))
            state, warning = lcm.read_state(self.path)
            self.assertEqual(state['auto_check'], 'never', ref)
            self.assertIsNone(state['reference_counts'])
            self.assertTrue(warning)

    def test_unreadable_path_is_auto_with_a_warning(self):
        os.makedirs(self.path)          # a directory where the file should be
        state, warning = lcm.read_state(self.path)
        self.assertEqual(state, lcm.default_state())
        self.assertTrue(warning)


if __name__ == '__main__':
    unittest.main()
