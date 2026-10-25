# nebulaos_beeper tests (G108): RTTTL/songs parsing, the sysfs PWM write
# order, non-blocking scheduling, caps, the API click and failure handling.
#
# Run from the repository root: python3 -m unittest extras.test_nebulaos_beeper -v
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import os
import shutil
import tempfile
import unittest

try:
    from . import nebulaos_beeper as nb
except ImportError:
    import nebulaos_beeper as nb


class FakeGcmd(object):
    class error(Exception):
        pass

    def __init__(self, params=None):
        self.params = params or {}
        self.info = []

    def get(self, key, default=None):
        return self.params.get(key, default)

    def get_int(self, key, default=None, minval=None, maxval=None):
        v = int(self.params.get(key, default))
        if minval is not None and v < minval:
            raise self.error(key)
        if maxval is not None and v > maxval:
            raise self.error(key)
        return v

    def get_float(self, key, default=None, above=None, maxval=None):
        v = float(self.params.get(key, default))
        if above is not None and v <= above:
            raise self.error(key)
        if maxval is not None and v > maxval:
            raise self.error(key)
        return v

    def respond_info(self, msg):
        self.info.append(msg)


class FakeReactor(object):
    NOW = 0.
    NEVER = 9999999999999999.

    def __init__(self):
        self.now = 100.
        self.timers = {}

    def monotonic(self):
        return self.now

    def register_timer(self, cb, waketime=None):
        self.timers[cb] = self.NEVER if waketime is None else waketime
        return cb

    def update_timer(self, timer, waketime):
        self.timers[timer] = waketime

    def run_until(self, t):
        """Fire due timers in order until reactor time t."""
        while True:
            due = [(w, cb) for cb, w in self.timers.items() if w <= t]
            if not due:
                break
            w, cb = min(due, key=lambda x: x[0])
            self.now = max(self.now, w)
            self.timers[cb] = cb(self.now)
        self.now = t


class FakeGcode(object):
    def __init__(self):
        self.commands = {}

    def register_command(self, name, func, desc=None):
        self.commands[name] = func


class FakeWebhooks(object):
    def __init__(self):
        self.endpoints = {}

    def register_endpoint(self, path, cb):
        self.endpoints[path] = cb


class FakeWebRequest(object):
    def __init__(self):
        self.sent = None

    def send(self, data):
        self.sent = data


class FakeToolhead(object):
    """Lookahead callbacks fire when flush() is called (moves executed)."""
    def __init__(self):
        self.pending = []

    def register_lookahead_callback(self, cb):
        self.pending.append(cb)

    def flush(self, print_time):
        cbs, self.pending = self.pending, []
        for cb in cbs:
            cb(print_time)


class FakeMcu(object):
    def __init__(self, reactor):
        self.reactor = reactor

    def estimated_print_time(self, eventtime):
        return eventtime - 50.    # print time = reactor time - 50


class FakePrintStats(object):
    def __init__(self):
        self.state = 'standby'

    def get_status(self, eventtime):
        return {'state': self.state}


class FakePrinter(object):
    def __init__(self):
        self.reactor = FakeReactor()
        self.objects = {'gcode': FakeGcode(), 'webhooks': FakeWebhooks(),
                        'toolhead': FakeToolhead(),
                        'mcu': FakeMcu(self.reactor),
                        'print_stats': FakePrintStats()}
        self.handlers = {}

    def get_reactor(self):
        return self.reactor

    def lookup_object(self, name, default=None):
        return self.objects.get(name, default)

    def register_event_handler(self, event, cb):
        self.handlers.setdefault(event, []).append(cb)

    def send_event(self, event):
        for cb in self.handlers.get(event, []):
            cb()


class FakeConfig(object):
    def __init__(self, printer, values):
        self.printer = printer
        self.values = values

    def get_printer(self):
        return self.printer

    def get(self, key, default=None):
        return self.values.get(key, default)

    def getint(self, key, default=None, minval=None, maxval=None):
        return int(self.values.get(key, default))

    def getfloat(self, key, default=None, above=None, maxval=None):
        return float(self.values.get(key, default))


class FakeSysfs(object):
    """A pwmchip directory whose export creates pwmN like the kernel."""
    def __init__(self, export_works=True):
        self.root = tempfile.mkdtemp()
        self.chip = os.path.join(self.root, 'pwmchip0')
        os.mkdir(self.chip)
        self.export_works = export_works
        open(os.path.join(self.chip, 'export'), 'w').close()

    def cleanup(self):
        shutil.rmtree(self.root)


class RecordingPwm(nb.SysfsPwm):
    def __init__(self, *a, **kw):
        nb.SysfsPwm.__init__(self, *a, **kw)
        self.log = []
        self.fail = False

    def _write(self, name, value):
        if self.fail:
            raise OSError(5, "EIO")
        self.log.append((name, int(value)))


class RtttlTest(unittest.TestCase):
    def test_defaults_and_notes(self):
        segs = nb.parse_rtttl("t:d=4,o=5,b=120:8c,p,a#4,8e6.", 0.5)
        # whole = 2000 ms; 8c = 250 ms -> 238 + 12 gap
        self.assertEqual(segs[0], (524, 238, 0.5))
        self.assertEqual(segs[1], (0, 12, 0.))
        self.assertEqual(segs[2], (0, 500, 0.))          # p, quarter
        self.assertEqual(segs[3], (466, 488, 0.5))       # a#4
        self.assertEqual(segs[5], (1320, 375 - 12, 0.5))  # dotted 8e6

    def test_s_is_sharp_and_matches_hash(self):
        a = nb.parse_rtttl("x:d=8,o=5,b=100:as,cs6", 0.5)
        b = nb.parse_rtttl("x:d=8,o=5,b=100:a#,c#6", 0.5)
        self.assertEqual(a, b)
        self.assertEqual(a[0][0], 932)

    def test_short_notes_have_no_gap(self):
        segs = nb.parse_rtttl("x:d=32,o=5,b=200:c", 0.5)
        self.assertEqual(segs, [(524, 37, 0.5)])

    def test_b_sharp_wraps_octave(self):
        self.assertEqual(nb.parse_rtttl("x:d=4,o=5,b=60:b#", 0.5)[0][0], 1048)

    def test_garbage_skipped(self):
        self.assertEqual(nb.parse_rtttl("x:d=4,o=5,b=60:zz,,", 0.5), [])

    def test_songs_conf_shipped_tunes_parse(self):
        here = os.path.dirname(os.path.abspath(__file__))
        tmp = tempfile.mkdtemp()
        try:
            path = os.path.join(tmp, 'songs.conf')
            with open(path, 'w') as f:
                f.write("# comment\n; also\n\nMario = mario:d=4,o=5,b=100:"
                        "16e6,16e6,32p\nerror = error:d=4,o=5,b=120:8c,8p,8c\n")
            self.assertTrue(nb.find_song(path, 'mario').startswith('mario:'))
            self.assertIsNone(nb.find_song(path, 'nope'))
            self.assertTrue(nb.parse_rtttl(nb.find_song(path, 'error'), .5))
        finally:
            shutil.rmtree(tmp)
        self.assertTrue(here)

    def test_cap(self):
        segs = [(440, 600, .5), (0, 600, 0.), (440, 600, .5)]
        out, cut = nb.cap_segments(segs, 1000)
        self.assertTrue(cut)
        self.assertEqual(out, [(440, 600, .5), (0, 400, 0.)])
        self.assertEqual(nb.cap_segments(segs, 5000), (segs, False))


class SysfsPwmTest(unittest.TestCase):
    def test_setup_exports_and_primes(self):
        fs = FakeSysfs()
        try:
            p = nb.SysfsPwm(fs.chip, 3)
            # simulate the kernel creating pwm3 on export
            orig_open = open

            def kernel_open(path, mode='r', *a, **kw):
                f = orig_open(path, mode, *a, **kw)
                if path.endswith('export') and 'w' in mode:
                    os.mkdir(os.path.join(fs.chip, 'pwm3'))
                return f
            nb.open = kernel_open
            try:
                p.setup()
            finally:
                del nb.open
            self.assertTrue(os.path.isdir(p.dir))
            with open(os.path.join(fs.chip, 'export')) as f:
                self.assertEqual(f.read(), '3')
            with open(os.path.join(p.dir, 'enable')) as f:
                self.assertEqual(f.read(), '0')
            self.assertFalse(p.enabled)
        finally:
            fs.cleanup()

    def test_setup_fails_cleanly_without_channel(self):
        fs = FakeSysfs()
        try:
            p = nb.SysfsPwm(fs.chip, 3)
            with self.assertRaises(nb.BeeperError):
                p.setup()
        finally:
            fs.cleanup()

    def test_frequency_change_order(self):
        p = RecordingPwm('/x', 3)
        p.period, p.duty, p.enabled = 40000, 100, False
        p.tone(1000, 0.5)
        self.assertEqual(p.log, [('duty_cycle', 0), ('period', 1000000),
                                 ('duty_cycle', 500000), ('enable', 1)])
        p.log = []
        p.tone(2000, 0.5)   # running: off first, then duty 0 before period
        self.assertEqual(p.log, [('enable', 0), ('duty_cycle', 0),
                                 ('period', 500000), ('duty_cycle', 250000),
                                 ('enable', 1)])
        p.log = []
        p.tone(2000, 0.02)  # same pitch, new volume: off, duty, on (A5:
        # never reprogram an enabled channel - the driver would spin)
        self.assertEqual(p.log, [('enable', 0), ('duty_cycle', 10000),
                                 ('enable', 1)])
        p.log = []
        p.tone(2000, 0.02)  # nothing changes: nothing written
        self.assertEqual(p.log, [])
        p.log = []
        p.off()
        p.off()
        self.assertEqual(p.log, [('enable', 0)])


class BeeperTest(unittest.TestCase):
    def make(self, values=None, available=True):
        self.printer = FakePrinter()
        cfg = FakeConfig(self.printer, values or {})
        b = nb.NebulaOSBeeper(cfg)
        b.pwm = RecordingPwm('/x', 3)
        b.pwm.period, b.pwm.duty, b.pwm.enabled = 40000, 0, False
        b.toolhead = self.printer.objects['toolhead']
        b.mcu = self.printer.objects['mcu']
        b.print_stats = self.printer.objects['print_stats']
        b.available = available
        self.r = self.printer.reactor
        return b

    def tones(self, b):
        return [v for k, v in b.pwm.log if k == 'period']

    def test_registers_commands_and_endpoint(self):
        b = self.make()
        g = self.printer.objects['gcode'].commands
        for c in ('NEBULAOS_BEEP', 'NEBULAOS_PLAY_TUNE',
                  'NEBULAOS_BEEPER_STOP'):
            self.assertIn(c, g)
        self.assertIn('nebulaos_beeper/click',
                      self.printer.objects['webhooks'].endpoints)
        self.assertTrue(b.get_status(0)['available'])

    def test_beep_waits_for_print_position_without_blocking(self):
        b = self.make()
        b.cmd_NEBULAOS_BEEP(FakeGcmd({'FREQ': '1000', 'DURATION': '200'}))
        # command returned; nothing sounds until the queued moves are done
        self.assertEqual(b.pwm.log, [])
        th = self.printer.objects['toolhead']
        th.flush(print_time=52.)          # = reactor time 102
        self.r.run_until(101.9)
        self.assertEqual(b.pwm.log, [])
        self.r.run_until(102.1)
        self.assertEqual(self.tones(b), [1000000])
        self.assertTrue(b.pwm.enabled)
        self.r.run_until(102.25)
        self.assertFalse(b.pwm.enabled)

    def test_sounds_queue_back_to_back(self):
        b = self.make()
        th = self.printer.objects['toolhead']
        b.cmd_NEBULAOS_BEEP(FakeGcmd({'FREQ': '1000', 'DURATION': '100'}))
        b.cmd_NEBULAOS_BEEP(FakeGcmd({'FREQ': '0', 'DURATION': '100'}))
        b.cmd_NEBULAOS_BEEP(FakeGcmd({'FREQ': '2000', 'DURATION': '100'}))
        th.flush(print_time=50.)
        self.r.run_until(100.05)
        self.assertTrue(b.pwm.enabled)
        self.r.run_until(100.15)
        self.assertFalse(b.pwm.enabled)   # the S0 rest
        self.r.run_until(100.25)
        self.assertTrue(b.pwm.enabled)
        self.assertEqual(self.tones(b), [1000000, 500000])
        self.r.run_until(100.5)
        self.assertFalse(b.pwm.enabled)

    def test_beep_duration_capped(self):
        b = self.make({'max_beep_ms': 500})
        b.cmd_NEBULAOS_BEEP(FakeGcmd({'FREQ': '1000', 'DURATION': '60000'}))
        self.printer.objects['toolhead'].flush(50.)
        self.r.run_until(100.4)
        self.assertTrue(b.pwm.enabled)
        self.r.run_until(100.6)
        self.assertFalse(b.pwm.enabled)

    def _songs(self, text):
        d = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, d)
        p = os.path.join(d, 'songs.conf')
        with open(p, 'w') as f:
            f.write(text)
        return p

    def test_play_song_and_inline(self):
        path = self._songs("mario = m:d=4,o=5,b=240:c,e,g\n")
        b = self.make({'songs_file': path})
        b.cmd_NEBULAOS_PLAY_TUNE(FakeGcmd({}))       # default song
        self.printer.objects['toolhead'].flush(50.)
        self.r.run_until(101.)
        self.assertEqual(self.tones(b), [NS(524), NS(660), NS(784)])
        b.pwm.log = []
        b.cmd_NEBULAOS_PLAY_TUNE(FakeGcmd({'RTTTL': 'x:d=4,o=6,b=240:as'}))
        self.printer.objects['toolhead'].flush(51.)
        self.r.run_until(102.)
        self.assertEqual(self.tones(b), [NS(1864)])

    def test_unknown_song_and_unreadable_file(self):
        b = self.make({'songs_file': self._songs("a = a:d=4,o=5,b=60:c\n")})
        with self.assertRaises(FakeGcmd.error):
            b.cmd_NEBULAOS_PLAY_TUNE(FakeGcmd({'SONG': 'zzz'}))
        b.songs_file = '/nonexistent/songs.conf'
        with self.assertRaises(FakeGcmd.error):
            b.cmd_NEBULAOS_PLAY_TUNE(FakeGcmd({'SONG': 'a'}))

    def test_tune_shortened_while_printing(self):
        path = self._songs("long = l:d=1,o=5,b=60:c,d,e,f,g,a,b\n")  # 28 s
        b = self.make({'songs_file': path,
                       'max_tune_seconds_printing': 5.})
        self.printer.objects['print_stats'].state = 'printing'
        g = FakeGcmd({'SONG': 'long'})
        b.cmd_NEBULAOS_PLAY_TUNE(g)
        self.assertIn('while printing', g.info[0])
        self.printer.objects['toolhead'].flush(50.)
        self.r.run_until(104.9)
        self.assertTrue(b.pwm.enabled)
        self.r.run_until(105.1)
        self.assertFalse(b.pwm.enabled)
        self.assertEqual(self.r.timers[b.timer], self.r.NEVER)

    def test_stop(self):
        b = self.make()
        b.cmd_NEBULAOS_BEEP(FakeGcmd({'FREQ': '1000', 'DURATION': '5000'}))
        self.printer.objects['toolhead'].flush(50.)
        self.r.run_until(100.1)
        self.assertTrue(b.pwm.enabled)
        b.cmd_NEBULAOS_BEEPER_STOP(FakeGcmd())
        self.assertFalse(b.pwm.enabled)
        self.assertEqual(b.get_status(self.r.now)['queued_ms'], 0)

    def test_click_is_immediate_and_dropped_while_busy(self):
        b = self.make()
        click = self.printer.objects['webhooks'].endpoints[
            'nebulaos_beeper/click']
        wr = FakeWebRequest()
        click(wr)
        self.assertEqual(wr.sent, {})
        self.r.run_until(self.r.now)       # no toolhead/lookahead wait
        self.assertEqual(self.tones(b), [NS(260)])
        self.assertEqual(b.pwm.duty, int(NS(260) * 0.02))
        self.r.run_until(100.01)
        self.assertFalse(b.pwm.enabled)
        b.cmd_NEBULAOS_BEEP(FakeGcmd({'FREQ': '1000', 'DURATION': '500'}))
        self.printer.objects['toolhead'].flush(50.01)
        b.pwm.log = []
        click(FakeWebRequest())
        self.r.run_until(100.2)
        self.assertEqual(self.tones(b), [NS(1000)])   # no click mixed in

    def test_unavailable_never_fails_a_print(self):
        b = self.make(available=False)
        b.last_error = "beeper unavailable: no pwm3"
        g = FakeGcmd({'FREQ': '1000'})
        b.cmd_NEBULAOS_BEEP(g)                 # no exception
        self.assertIn('unavailable', g.info[0])
        click = self.printer.objects['webhooks'].endpoints[
            'nebulaos_beeper/click']
        wr = FakeWebRequest()
        click(wr)
        self.assertEqual(wr.sent, {})
        self.assertEqual(b.pwm.log, [])

    def test_connect_without_channel_is_not_fatal(self):
        fs = FakeSysfs()
        self.addCleanup(fs.cleanup)
        printer = FakePrinter()
        b = nb.NebulaOSBeeper(FakeConfig(printer, {'pwmchip': fs.chip}))
        printer.send_event('klippy:connect')
        self.assertFalse(b.available)
        self.assertIn('unavailable', b.get_status(0)['error'])

    def test_write_error_mid_sound_silences(self):
        b = self.make()
        b.cmd_NEBULAOS_BEEP(FakeGcmd({'FREQ': '1000', 'DURATION': '500'}))
        self.printer.objects['toolhead'].flush(50.)
        b.pwm.fail = True
        self.r.run_until(100.1)
        self.assertFalse(b.available)
        self.assertIn('write failed', b.last_error)
        self.assertEqual(self.r.timers[b.timer], self.r.NEVER)

    def test_shutdown_silences(self):
        b = self.make()
        b.cmd_NEBULAOS_BEEP(FakeGcmd({'FREQ': '1000', 'DURATION': '5000'}))
        self.printer.objects['toolhead'].flush(50.)
        self.r.run_until(100.1)
        self.printer.send_event('klippy:shutdown')
        self.assertFalse(b.pwm.enabled)
        self.assertEqual(self.r.timers[b.timer], self.r.NEVER)

    def test_queue_limit(self):
        b = self.make({'max_queue_seconds': 1.})
        for _ in range(5):
            b.cmd_NEBULAOS_BEEP(FakeGcmd({'FREQ': '1000', 'DURATION': '600'}))
        self.printer.objects['toolhead'].flush(50.)
        # 600 + 600 queued (> 1 s) -> the rest are dropped
        self.assertLessEqual(b.get_status(self.r.now)['queued_ms'], 1800)


def NS(freq):
    return nb.NS_PER_S // freq


if __name__ == '__main__':
    unittest.main()
