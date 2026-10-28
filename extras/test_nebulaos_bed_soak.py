# Tests for extras/nebulaos_bed_soak.py (Run 21): the bed warp soak switch,
# remembered across Klipper restarts.
#
# Run from klippy/: python3 -m unittest extras.test_nebulaos_bed_soak -v
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import json
import os
import shutil
import tempfile
import unittest
from unittest import mock

from . import prtouch_test_support as fake
from . import nebulaos_bed_soak as bs


class PinGCode(fake.FakeGCode):
    """An upstream-like SET_PIN (mux PIN=) and run_script through the
    registered handlers, so the wrapper is exercised as in Klipper."""
    def __init__(self):
        super().__init__()
        self.info = []
        self.pins = {}
        self.commands['SET_PIN'] = self._upstream_set_pin

    def _upstream_set_pin(self, gcmd):
        self.pins[gcmd.get('PIN')] = gcmd.get_float('VALUE')

    def register_command(self, name, handler, desc=None):
        if handler is None:
            return self.commands.pop(name, None)
        self.commands[name] = handler

    def run_script(self, script):
        words = script.split()
        params = dict(t.split('=', 1) for t in words[1:])
        gcmd = fake.FakeGCmd(params)
        self.commands[words[0]](gcmd)
        self.info.extend(gcmd.responses)

    def respond_info(self, msg):
        self.info.append(msg)


class CallbackReactor(fake.FakeReactor):
    def __init__(self):
        super().__init__()
        self.callbacks = []

    def register_callback(self, cb):
        self.callbacks.append(cb)

    def run_callbacks(self):
        while self.callbacks:
            self.callbacks.pop(0)(self.monotonic())


def start(path):
    """A Klipper start: new objects, klippy:ready, callbacks run."""
    printer = fake.FakePrinter()
    printer.reactor = CallbackReactor()
    gcode = PinGCode()
    printer.add_object('gcode', gcode)
    config = fake.FakeConfig({'filename': path}, section='nebulaos_bed_soak',
                             printer=printer)
    obj = bs.NebulaOSBedSoak(config)
    config.assert_all_consumed()
    printer.send_event('klippy:ready')
    printer.reactor.run_callbacks()
    return gcode, obj


def set_pin(gcode, pin, value):
    gcode.run_script("SET_PIN PIN=%s VALUE=%s" % (pin, value))


class BedSoakTest(unittest.TestCase):
    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()
        self.path = os.path.join(self.tmpdir, 'calibration', 'bed_soak.json')

    def tearDown(self):
        shutil.rmtree(self.tmpdir, ignore_errors=True)

    def test_off_by_default_without_a_file(self):
        gcode, obj = start(self.path)
        self.assertEqual(obj.get_status(0.), {'enabled': False, 'error': ''})
        self.assertNotIn(bs.DEFAULT_PIN, gcode.pins)       # nothing switched
        self.assertFalse(os.path.exists(self.path))

    def test_switch_on_survives_a_klipper_restart_until_switched_off(self):
        gcode, obj = start(self.path)
        set_pin(gcode, bs.DEFAULT_PIN, '1.00')             # Mainsail toggle
        self.assertEqual(gcode.pins[bs.DEFAULT_PIN], 1.)   # upstream still ran
        self.assertTrue(obj.enabled)
        with open(self.path) as fh:
            self.assertEqual(json.load(fh), {'enabled': True})
        gcode, obj = start(self.path)                      # restart
        self.assertEqual(gcode.pins[bs.DEFAULT_PIN], 1.)
        self.assertTrue(obj.get_status(0.)['enabled'])
        set_pin(gcode, bs.DEFAULT_PIN, '0')
        gcode, obj = start(self.path)                      # restart again
        self.assertNotIn(bs.DEFAULT_PIN, gcode.pins)
        self.assertFalse(obj.enabled)

    def test_other_pins_pass_through_and_are_not_saved(self):
        gcode, obj = start(self.path)
        set_pin(gcode, 'fan0', '0.5')
        self.assertEqual(gcode.pins['fan0'], .5)
        self.assertFalse(os.path.exists(self.path))

    def test_bad_file_is_off_with_a_warning_and_klipper_starts(self):
        os.makedirs(os.path.dirname(self.path))
        for text in ('{not json', '[]', '{"enabled": "yes"}', '{"enabled": 1}'):
            with open(self.path, 'w') as fh:
                fh.write(text)
            gcode, obj = start(self.path)
            self.assertFalse(obj.enabled, text)
            self.assertTrue(obj.error, text)
            self.assertTrue(any('switch is off' in m for m in gcode.info), text)
        set_pin(gcode, bs.DEFAULT_PIN, '1')                # a valid save heals it
        self.assertEqual(bs.read_enabled(self.path), (True, ''))

    def test_failed_save_keeps_the_live_switch_and_says_so(self):
        gcode, obj = start(self.path)
        with mock.patch.object(bs.journal, 'atomic_write_json',
                               side_effect=OSError("disk full")):
            set_pin(gcode, bs.DEFAULT_PIN, '1')
        self.assertEqual(gcode.pins[bs.DEFAULT_PIN], 1.)
        self.assertFalse(obj.enabled)
        self.assertIn('NOT saved', gcode.info[-1])

    def test_write_is_atomic(self):
        calls = []
        real = os.replace
        with mock.patch('os.replace', lambda s, d: (calls.append((s, d)), real(s, d))):
            bs.write_enabled(self.path, True)
        self.assertEqual(calls, [(self.path + '.tmp', self.path)])


if __name__ == '__main__':
    unittest.main()
