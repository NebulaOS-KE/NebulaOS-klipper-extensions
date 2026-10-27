# Tests for extras/nebulaos_calibration_history.py (G104/G105, Run 19).
#
# Run from the repository root: python3 -m unittest extras.test_nebulaos_calibration_history
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import json
import os
import shutil
import tempfile
import unittest

from . import nebulaos_calibration_history as history


class HistoryFileTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.tmp, True)
        self.path = os.path.join(self.tmp, 'cal', history.FILE_NAME)

    def test_missing_file_is_an_empty_history_without_warning(self):
        state, warning = history.load(self.path)
        self.assertIsNone(warning)
        self.assertEqual(state, history.empty_state())
        self.assertEqual(set(state['history']), set(history.FLOWS))

    def test_round_trip_creates_the_directory(self):
        state = history.empty_state()
        history.record(state, 'pid', 'Bed 65 C', 100.)
        state['last_result'] = history.make_result(
            'esteps', True, 101., ['Rotation distance 7.1'])
        history.save(self.path, state)
        loaded, warning = history.load(self.path)
        self.assertIsNone(warning)
        self.assertEqual(loaded, state)
        self.assertFalse(os.path.exists(self.path + '.tmp'))

    def test_corrupt_or_foreign_file_is_empty_with_a_warning(self):
        os.makedirs(os.path.dirname(self.path))
        for payload in (b'{broken', b'[]', b'{"schema": 99, "history": {}}',
                        b'{"schema": 1, "history": 3}', b'\xff\xfe'):
            with open(self.path, 'wb') as f:
                f.write(payload)
            state, warning = history.load(self.path)
            self.assertEqual(state, history.empty_state(), payload)
            self.assertIn('unreadable', warning)

    def test_bad_entries_are_cleaned_unknown_keys_dropped(self):
        os.makedirs(os.path.dirname(self.path))
        with open(self.path, 'w') as f:
            json.dump({'schema': 1, 'history': {
                'pid': {'done_at': 'yesterday', 'summary': 'a\nb',
                        'redo': 'x'},
                'bogus': {'done_at': 1}},
                'last_result': {'flow': 'nope'}}, f)
        state, warning = history.load(self.path)
        self.assertIsNone(warning)
        self.assertEqual(state['history']['pid'],
                         {'done_at': None, 'summary': 'a b', 'redo': 'x'})
        self.assertNotIn('bogus', state['history'])
        self.assertIsNone(state['last_result'])


class RulesTest(unittest.TestCase):
    def test_record_clears_its_own_redo(self):
        state = history.empty_state()
        history.mark_redo(state, ('load_cell', 'auto_calibrate'),
                          history.REDO_SHIM_FRONT_LEFT)
        history.record(state, 'load_cell', 'Automatic check passed', 5.)
        self.assertIsNone(state['history']['load_cell']['redo'])
        self.assertEqual(state['history']['auto_calibrate']['redo'],
                         "a shim was added at front left")
        history.record(state, 'auto_calibrate', 'Z offset 1.7 mm', 6.)
        self.assertIsNone(state['history']['auto_calibrate']['redo'])

    def test_pid_keeps_both_heaters(self):
        # G120: PID_BED and PID_HOTEND share the one 'pid' key
        state = history.empty_state()
        history.record(state, 'pid', 'Hotend tuned at 230 C', 1.)
        history.record(state, 'pid', 'Bed tuned at 65 C', 2.)
        entry = state['history']['pid']
        self.assertEqual(entry['summary'],
                         'Bed tuned at 65 C; Hotend tuned at 230 C')
        self.assertEqual(entry['done_at'], 2.)
        # the same heater again replaces only its own part
        history.record(state, 'pid',
                       'Bed PID tuned at 70 C - SAVE_CONFIG to keep', 3.)
        self.assertEqual(state['history']['pid']['summary'],
                         'Bed PID tuned at 70 C - SAVE_CONFIG to keep; '
                         'Hotend tuned at 230 C')
        # a summary without a known heater replaces everything (as before)
        history.record(state, 'pid', 'Chamber tuned', 4.)
        self.assertEqual(state['history']['pid']['summary'], 'Chamber tuned')
        history.record(state, 'pid', 'Hotend tuned at 230 C', 5.)
        self.assertEqual(state['history']['pid']['summary'],
                         'Hotend tuned at 230 C')

    def test_pid_merge_stays_within_the_summary_limit(self):
        state = history.empty_state()
        history.record(state, 'pid', 'Bed ' + 'x' * 200, 1.)
        history.record(state, 'pid', 'Hotend ' + 'y' * 200, 2.)
        summary = state['history']['pid']['summary']
        self.assertLessEqual(len(summary), history.MAX_SUMMARY_LEN)
        self.assertTrue(summary.startswith('Bed '))

    def test_axis_twist_marks_auto_calibrate(self):
        state = history.empty_state()
        history.record(state, 'auto_calibrate', 'Z', 1.)
        history.record(state, 'axis_twist', 'X', 2.)
        self.assertEqual(state['history']['auto_calibrate']['redo'],
                         "Axis Twist changed")
        self.assertEqual(state['history']['auto_calibrate']['done_at'], 1.)

    def test_unknown_flow_is_refused(self):
        with self.assertRaises(ValueError):
            history.record(history.empty_state(), 'nope', '', 1.)

    def test_external_flows_are_a_subset(self):
        self.assertTrue(set(history.EXTERNAL_FLOWS) < set(history.FLOWS))

    def test_sanitize(self):
        self.assertEqual(history.sanitize_text('"a  b"'), 'a b')
        self.assertEqual(history.sanitize_text(None), '')
        long = history.sanitize_text('y' * 500)
        self.assertEqual(len(long), history.MAX_SUMMARY_LEN)

    def test_snapshot_is_a_copy(self):
        state = history.empty_state()
        h, r = history.snapshot(state)
        h['pid']['summary'] = 'x'
        self.assertEqual(state['history']['pid']['summary'], '')


if __name__ == '__main__':
    unittest.main()
