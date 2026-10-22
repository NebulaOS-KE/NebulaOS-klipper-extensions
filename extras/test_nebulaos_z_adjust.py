# Tests for extras/nebulaos_z_adjust.py (G92): the screen's Z offset
# adjustment, saved in one small JSON file and applied at every start.
#
# Run from klippy/: python3 -m unittest extras.test_nebulaos_z_adjust -v
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import collections
import json
import os
import shutil
import tempfile
import unittest
from unittest import mock

from . import prtouch_test_support as fake
from . import nebulaos_z_adjust as za

Coord = collections.namedtuple('Coord', 'x y z e')


class FakeGCodeMove:
    def __init__(self):
        self.z = 0.

    def get_status(self, eventtime):
        return {'homing_origin': Coord(0., 0., self.z, 0.)}


class OffsetGCode(fake.FakeGCode):
    """Executes SET_GCODE_OFFSET Z=/Z_ADJUST= against FakeGCodeMove."""
    def __init__(self, gcode_move):
        super().__init__()
        self.gcode_move = gcode_move
        self.info = []
        self.run_script_calls = []

    def _offset(self, script):
        if not script.startswith('SET_GCODE_OFFSET'):
            return
        params = dict(t.split('=', 1) for t in script.split()[1:])
        if 'Z' in params:
            self.gcode_move.z = float(params['Z'])
        if 'Z_ADJUST' in params:
            self.gcode_move.z += float(params['Z_ADJUST'])

    def run_script_from_command(self, script):
        self.scripts_run.append(script)
        self._offset(script)

    def run_script(self, script):
        self.run_script_calls.append(script)
        self._offset(script)

    def respond_info(self, msg):
        self.info.append(msg)

    def register_command(self, name, handler, desc=None):
        # Klipper semantics: handler None unregisters and returns the old one.
        if handler is None:
            return self.commands.pop(name, None)
        self.commands[name] = handler


class FakeToolhead:
    def __init__(self, homed='xyz'):
        self.homed = homed

    def get_status(self, eventtime):
        return {'homed_axes': self.homed}


class CallbackReactor(fake.FakeReactor):
    def __init__(self):
        super().__init__()
        self.callbacks = []

    def register_callback(self, cb):
        self.callbacks.append(cb)

    def run_callbacks(self):
        while self.callbacks:
            self.callbacks.pop(0)(self.monotonic())


def build(path, homed='xyz', ready=True, overrides=None):
    printer = fake.FakePrinter()
    printer.reactor = CallbackReactor()
    gm = FakeGCodeMove()
    gcode = OffsetGCode(gm)
    printer.add_object('gcode', gcode)
    printer.add_object('gcode_move', gm)
    printer.add_object('toolhead', FakeToolhead(homed))
    values = {'filename': path}
    values.update(overrides or {})
    config = fake.FakeConfig(values, section='nebulaos_z_adjust',
                             printer=printer)
    obj = za.NebulaOSZAdjust(config)
    config.assert_all_consumed()
    if ready:
        printer.send_event('klippy:ready')
        printer.reactor.run_callbacks()
    return printer, gcode, gm, obj


def write_raw(path, text):
    with open(path, 'w') as handle:
        handle.write(text)


class Base(unittest.TestCase):
    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()
        self.path = os.path.join(self.tmpdir, 'sub', 'z_offset_adjust.json')

    def tearDown(self):
        shutil.rmtree(self.tmpdir, ignore_errors=True)

    def saved(self):
        with open(self.path) as handle:
            return json.load(handle)


class FileTest(Base):
    def test_write_is_atomic_and_creates_the_directory(self):
        calls = []
        real_replace, real_fsync = os.replace, os.fsync

        def spy_replace(src, dst):
            calls.append(('replace', src, dst))
            self.assertEqual(os.path.dirname(src), os.path.dirname(dst))
            return real_replace(src, dst)

        def spy_fsync(fd):
            calls.append(('fsync',))
            return real_fsync(fd)
        with mock.patch('os.replace', spy_replace), \
                mock.patch('os.fsync', spy_fsync):
            za.write_value(self.path, 0.15)
        kinds = [c[0] for c in calls]
        self.assertEqual(kinds, ['fsync', 'replace', 'fsync'])
        self.assertEqual(calls[1][1], self.path + '.tmp')
        self.assertEqual(self.saved(), {'z_adjust_mm': 0.15})
        self.assertFalse(os.path.exists(self.path + '.tmp'))

    def test_missing_file_is_zero_without_warning(self):
        self.assertEqual(za.read_value(self.path, 2.), (0., ''))

    def test_corrupt_files_are_zero_with_a_warning(self):
        os.makedirs(os.path.dirname(self.path))
        for text in ('{"z_adjust_mm": 0.1', '', '[]', '{"other": 1}',
                     '{"z_adjust_mm": "abc"}', '{"z_adjust_mm": NaN}',
                     '{"z_adjust_mm": 7.5}'):
            write_raw(self.path, text)
            value, warning = za.read_value(self.path, 2.)
            self.assertEqual(value, 0., text)
            self.assertTrue(warning, text)

    def test_corrupt_file_never_breaks_startup(self):
        os.makedirs(os.path.dirname(self.path))
        write_raw(self.path, '{"z_adjust_mm": 0.')
        printer, gcode, gm, obj = build(self.path)
        status = obj.get_status(0.)
        self.assertEqual(status['value'], 0.)
        self.assertTrue(status['error'])
        self.assertEqual(gm.z, 0.)
        self.assertEqual(gcode.run_script_calls, [])
        self.assertTrue(any(m.startswith('!! ') for m in gcode.info))


class ReadyTest(Base):
    def test_saved_value_applied_absolute_at_ready(self):
        za.write_value(self.path, -0.12)
        printer, gcode, gm, obj = build(self.path, ready=False)
        self.assertFalse(obj.get_status(0.)['applied'])
        gm.z = 0.5          # whatever was live: the apply is absolute
        printer.send_event('klippy:ready')
        # Not inside the event handler: on a reactor callback.
        self.assertEqual(gcode.run_script_calls, [])
        printer.reactor.run_callbacks()
        self.assertEqual(gcode.run_script_calls,
                         ['SET_GCODE_OFFSET Z=-0.120000 MOVE=0'])
        self.assertAlmostEqual(gm.z, -0.12)
        self.assertEqual(obj.get_status(0.),
                         {'value': -0.12, 'applied': True, 'error': ''})

    def test_zero_is_not_applied(self):
        za.write_value(self.path, 0.)
        printer, gcode, gm, obj = build(self.path)
        self.assertEqual(gcode.run_script_calls, [])
        self.assertTrue(obj.get_status(0.)['applied'])


class CommandTest(Base):
    def test_delta_adjusts_live_and_saves_the_total(self):
        printer, gcode, gm, obj = build(self.path)
        obj.cmd_z_adjust(fake.FakeGCmd({'DELTA': '0.05'}))
        obj.cmd_z_adjust(fake.FakeGCmd({'DELTA': '-0.02'}))
        self.assertEqual(gcode.scripts_run[-1],
                         'SET_GCODE_OFFSET Z_ADJUST=-0.020000 MOVE=1')
        self.assertAlmostEqual(gm.z, 0.03)
        self.assertAlmostEqual(self.saved()['z_adjust_mm'], 0.03)
        status = obj.get_status(0.)
        self.assertAlmostEqual(status['value'], 0.03)
        self.assertTrue(status['applied'])

    def test_saved_total_is_the_live_homing_origin(self):
        printer, gcode, gm, obj = build(self.path)
        gm.z = 0.2       # e.g. restored by PLR in the same session
        obj.cmd_z_adjust(fake.FakeGCmd({'DELTA': '0.01'}))
        self.assertAlmostEqual(self.saved()['z_adjust_mm'], 0.21)

    def test_delta_refused_when_z_not_homed(self):
        printer, gcode, gm, obj = build(self.path, homed='xy')
        with self.assertRaises(fake.CommandError) as ctx:
            obj.cmd_z_adjust(fake.FakeGCmd({'DELTA': '0.05'}))
        self.assertIn('home Z first', str(ctx.exception))
        self.assertEqual(gcode.scripts_run, [])
        self.assertFalse(os.path.exists(self.path))

    def test_delta_beyond_the_limit_is_refused(self):
        printer, gcode, gm, obj = build(self.path)
        with self.assertRaises(fake.CommandError):
            obj.cmd_z_adjust(fake.FakeGCmd({'DELTA': '2.5'}))
        self.assertEqual(gcode.scripts_run, [])

    def test_reset_homed_moves_and_saves_zero(self):
        za.write_value(self.path, 0.1)
        printer, gcode, gm, obj = build(self.path)
        obj.cmd_z_adjust(fake.FakeGCmd({'RESET': '1'}))
        self.assertEqual(gcode.scripts_run, ['SET_GCODE_OFFSET Z=0 MOVE=1'])
        self.assertEqual(gm.z, 0.)
        self.assertEqual(self.saved(), {'z_adjust_mm': 0.0})
        self.assertEqual(obj.get_status(0.)['value'], 0.)

    def test_reset_not_homed_does_not_move(self):
        za.write_value(self.path, 0.1)
        printer, gcode, gm, obj = build(self.path, homed='')
        obj.cmd_z_adjust(fake.FakeGCmd({'RESET': '1'}))
        self.assertEqual(gcode.scripts_run, ['SET_GCODE_OFFSET Z=0 MOVE=0'])
        self.assertEqual(self.saved(), {'z_adjust_mm': 0.0})

    def test_no_arguments_reports_the_saved_value(self):
        za.write_value(self.path, -0.07)
        printer, gcode, gm, obj = build(self.path)
        gcmd = fake.FakeGCmd({})
        obj.cmd_z_adjust(gcmd)
        self.assertEqual(gcmd.responses,
                         ['Z offset adjustment: -0.070 mm, saved'])
        self.assertEqual(gcode.scripts_run, [])

    def test_apply_after_resume_restores_the_saved_value(self):
        # C4: upstream RESUME restores the gcode state saved at PAUSE.
        printer, gcode, gm, obj = build(self.path)
        obj.cmd_z_adjust(fake.FakeGCmd({'DELTA': '0.04'}))
        gm.z = 0.         # RESTORE_GCODE_STATE NAME=PAUSE_STATE
        obj.cmd_z_adjust(fake.FakeGCmd({'APPLY': '1'}))
        self.assertEqual(gcode.scripts_run[-1],
                         'SET_GCODE_OFFSET Z=0.040000 MOVE=1')
        self.assertAlmostEqual(gm.z, 0.04)

    def test_apply_not_homed_does_not_move(self):
        za.write_value(self.path, 0.04)
        printer, gcode, gm, obj = build(self.path, homed='')
        obj.cmd_z_adjust(fake.FakeGCmd({'APPLY': '1'}))
        self.assertEqual(gcode.scripts_run[-1],
                         'SET_GCODE_OFFSET Z=0.040000 MOVE=0')

    def test_failed_save_is_reported_not_hidden(self):
        printer, gcode, gm, obj = build(self.path)
        with mock.patch.object(za, 'write_value',
                               side_effect=OSError(28, 'No space left')):
            with self.assertRaises(fake.CommandError) as ctx:
                obj.cmd_z_adjust(fake.FakeGCmd({'DELTA': '0.05'}))
        self.assertIn('NOT saved', str(ctx.exception))
        self.assertTrue(obj.get_status(0.)['error'])
        self.assertEqual(obj.get_status(0.)['value'], 0.)

    def test_reset_for_calibration_zeroes_file_and_live_without_moving(self):
        za.write_value(self.path, 0.15)
        printer, gcode, gm, obj = build(self.path)
        obj.reset_for_calibration()
        self.assertEqual(gcode.scripts_run, ['SET_GCODE_OFFSET Z=0 MOVE=0'])
        self.assertEqual(self.saved(), {'z_adjust_mm': 0.0})
        self.assertEqual(obj.get_status(0.)['value'], 0.)

    def test_reset_for_calibration_raises_when_it_cannot_save(self):
        za.write_value(self.path, 0.15)
        printer, gcode, gm, obj = build(self.path)
        with mock.patch.object(za, 'write_value', side_effect=OSError(5, 'EIO')):
            with self.assertRaises(fake.CommandError):
                obj.reset_for_calibration()


if __name__ == '__main__':
    unittest.main()


class PauseResumeTest(Base):
    """C4 (Run 17 verifier finding 1): RESUME re-adds only what the screen
    changed while paused; offsets from Mainsail or the print file stay."""
    def test_mainsail_babystep_survives_a_pause(self):
        printer, gcode, gm, obj = build(self.path)
        obj.cmd_z_adjust(fake.FakeGCmd({'DELTA': '0.04'}))     # screen, saved
        gm.z += 0.02                                            # Mainsail babystep
        obj.cmd_z_adjust(fake.FakeGCmd({'MARK': '1'}))         # PAUSE
        before = gm.z                                           # RESUME_BASE restores this
        obj.cmd_z_adjust(fake.FakeGCmd({'RESUME': '1'}))
        self.assertAlmostEqual(gm.z, before)
        self.assertAlmostEqual(gm.z, 0.06)

    def test_screen_change_while_paused_is_added_on_resume(self):
        printer, gcode, gm, obj = build(self.path)
        obj.cmd_z_adjust(fake.FakeGCmd({'MARK': '1'}))
        restored = gm.z
        obj.cmd_z_adjust(fake.FakeGCmd({'DELTA': '-0.03'}))
        gm.z = restored                                         # upstream restore undoes it
        obj.cmd_z_adjust(fake.FakeGCmd({'RESUME': '1'}))
        self.assertAlmostEqual(gm.z, restored - 0.03)
        self.assertAlmostEqual(self.saved()['z_adjust_mm'], -0.03)

    def test_resume_without_mark_changes_nothing(self):
        printer, gcode, gm, obj = build(self.path)
        gm.z = 0.123
        obj.cmd_z_adjust(fake.FakeGCmd({'RESUME': '1'}))
        self.assertAlmostEqual(gm.z, 0.123)


class ApplyProbeTest(Base):
    """C6 (verifier finding 2): Z_OFFSET_APPLY_PROBE folds the adjustment into
    the probe z_offset, so the saved adjustment must become 0."""
    def test_apply_probe_folds_and_zeroes_the_saved_value(self):
        write_raw(self.path_ready(), json.dumps({'z_adjust_mm': 0.05}))
        printer = fake.FakePrinter()
        printer.reactor = CallbackReactor()
        gm = FakeGCodeMove()
        gcode = OffsetGCode(gm)
        calls = []
        gcode.register_command('Z_OFFSET_APPLY_PROBE',
                               lambda g: calls.append(gm.z))
        printer.add_object('gcode', gcode)
        printer.add_object('gcode_move', gm)
        printer.add_object('toolhead', FakeToolhead())
        config = fake.FakeConfig({'filename': self.path},
                                 section='nebulaos_z_adjust', printer=printer)
        obj = za.NebulaOSZAdjust(config)
        printer.send_event('klippy:ready')
        printer.reactor.run_callbacks()
        gcode.commands['Z_OFFSET_APPLY_PROBE'](fake.FakeGCmd({}))
        self.assertEqual(calls, [0.05])                 # upstream ran with the live offset
        self.assertEqual(self.saved()['z_adjust_mm'], 0.)
        self.assertEqual(obj.value, 0.)

    def test_no_probe_command_is_harmless(self):
        printer, gcode, gm, obj = build(self.path)
        self.assertNotIn('Z_OFFSET_APPLY_PROBE', gcode.commands)

    def path_ready(self):
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        return self.path


class PathologicalFileTest(Base):
    """Verifier finding 3: no file content may stop Klipper starting."""
    def test_overflow_and_deep_nesting_give_zero(self):
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        for text in ('{"z_adjust_mm": 1' + '0' * 400 + '}',
                     '[' * 100000 + ']' * 100000):
            write_raw(self.path, text)
            value, warning = za.read_value(self.path, 2.0)
            self.assertEqual(value, 0.)
            self.assertTrue(warning)
