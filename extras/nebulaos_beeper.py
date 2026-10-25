# nebulaos_beeper - Klipper-owned KE buzzer (G108), host-side Klipper extra
#
# The Ender-3 V3 KE buzzer is a passive piezo on SoC pin GPC3 = PWM channel
# 3. The NebulaOS kernel (run19-g108) muxes GPC3 to the PWM function and the
# vendor PWM driver exposes it as a standard Linux PWM channel:
#   /sys/class/pwm/pwmchip0/pwm3/{period,duty_cycle,enable}
# This module is that channel's only user. It replaces the former
# gcode_shell_command + nebulabeep path (a shell process poking registers
# through /dev/mem, which held Klipper's G-code queue for the whole tune).
#
# Why not an upstream [output_pin] on the host MCU (host:pwmchip0/pwm3):
# Klipper's Linux hard_pwm.c writes the PWM period exactly once, at MCU
# config time, and current upstream has no runtime cycle-time change (SET_PIN
# takes VALUE only). The pitch of a passive piezo IS the PWM period, so an
# output_pin can play one fixed pitch and nothing else - no M300 S, no songs.
# Klippy and the PWM channel live on the same Linux host, so this module
# writes the kernel's sysfs interface directly from the reactor instead.
#
# Nothing here blocks the G-code queue:
#   - NEBULAOS_BEEP / NEBULAOS_PLAY_TUNE build a list of (freq, ms, duty)
#     segments and register a toolhead lookahead callback, exactly like
#     output_pin's SET_PIN: the sound starts when the moves queued before it
#     have been executed (print position), and the command itself returns
#     immediately. A reactor timer then walks the segments.
#   - Commands arriving while a sound plays are appended after it.
#   - The screen's touch click arrives over the Klippy API socket
#     (endpoint "nebulaos_beeper/click"), which is served by the reactor and
#     never waits for the G-code mutex (e.g. during M109), and is dropped
#     while another sound plays.
#   - Tunes are capped (max_tune_seconds; max_tune_seconds_printing while a
#     print runs) so a long song cannot keep the reactor busy for minutes.
# A missing PWM channel (old kernel) is reported, never fatal: Klipper
# starts and prints normally; the commands answer "beeper unavailable".
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import collections
import logging
import os

NS_PER_S = 1000000000

# integer frequencies for octave 4 (c4..b4); other octaves are exact
# power-of-two shifts (same table as the former nebulabeep player).
OCT4 = (262, 277, 294, 311, 330, 349, 370, 392, 415, 440, 466, 494)
NOTE_INDEX = {'c': 0, 'd': 2, 'e': 4, 'f': 5, 'g': 7, 'a': 9, 'b': 11}
ARTICULATION_GAP_MS = 12
MIN_FREQ = 20
MAX_FREQ = 20000


class BeeperError(Exception):
    pass


######################################################################
# RTTTL / songs.conf parsing (pure functions)
######################################################################

def note_freq(idx, octave):
    if idx < 0:
        return 0
    f = OCT4[idx]
    if octave >= 4:
        return f << (octave - 4)
    return f >> (4 - octave)


def parse_rtttl(text, duty):
    """'name:d=4,o=5,b=100:8c6,8p,...' -> [(freq, ms, duty), ...].

    Sharps may be '#' (standard) or 's' (G-code safe: Klipper treats '#' as
    a comment). Unknown characters are skipped, like the former player."""
    d_def, o_def, bpm = 4, 5, 63
    parts = text.split(':')
    if len(parts) >= 3:
        defaults, body = parts[1], ':'.join(parts[2:])
        for item in defaults.split(','):
            key, _, val = item.strip().partition('=')
            try:
                v = int(val)
            except ValueError:
                continue
            if v <= 0:
                continue
            if key == 'd':
                d_def = v
            elif key == 'o':
                o_def = v
            elif key == 'b':
                bpm = v
    elif len(parts) == 2:
        body = parts[1]
    else:
        body = parts[0]
    whole = 4 * 60000 // bpm
    segments = []
    for tok in body.split(','):
        p = tok.strip().lower()
        if not p:
            continue
        i = 0
        num = ''
        while i < len(p) and p[i].isdigit():
            num += p[i]
            i += 1
        dur = int(num) if num else d_def
        if dur <= 0:
            dur = d_def
        if i >= len(p):
            continue
        if p[i] == 'p':
            idx = -1
            i += 1
        elif p[i] in NOTE_INDEX:
            idx = NOTE_INDEX[p[i]]
            i += 1
        else:
            continue
        if idx >= 0 and i < len(p) and p[i] in '#s':
            idx += 1
            i += 1
        dotted = False
        if i < len(p) and p[i] == '.':
            dotted = True
            i += 1
        octave = o_def
        if i < len(p) and p[i].isdigit():
            octave = int(p[i])
            i += 1
        if i < len(p) and p[i] == '.':
            dotted = True
        ms = whole // dur
        if dotted:
            ms += ms // 2
        freq = note_freq(idx, octave) if idx < 12 else note_freq(0, octave + 1)
        if freq < MIN_FREQ:
            segments.append((0, ms, 0.))
            continue
        gap = ARTICULATION_GAP_MS if ms > 40 else 0
        segments.append((freq, ms - gap, duty))
        if gap:
            segments.append((0, gap, 0.))
    return segments


def find_song(path, name):
    """songs.conf: 'name = <rtttl>' per line, '#'/';' comment lines.
    Returns the RTTTL string or None."""
    with open(path, 'r') as f:
        for line in f:
            s = line.strip()
            if not s or s[0] in '#;':
                continue
            key, sep, val = s.partition('=')
            if not sep:
                continue
            if key.strip().lower() == name.lower():
                return val.strip()
    return None


def cap_segments(segments, max_ms):
    out = []
    total = 0
    for freq, ms, duty in segments:
        if total >= max_ms:
            return out, True
        ms = min(ms, max_ms - total)
        out.append((freq, ms, duty))
        total += ms
    return out, False


######################################################################
# Linux PWM sysfs channel
######################################################################

class SysfsPwm:
    def __init__(self, chip_dir, channel):
        self.chip_dir = chip_dir
        self.channel = channel
        self.dir = os.path.join(chip_dir, 'pwm%d' % (channel,))
        self.period = None
        self.duty = None
        self.enabled = None

    def _write(self, name, value):
        with open(os.path.join(self.dir, name), 'w') as f:
            f.write(str(value))

    def setup(self):
        if not os.path.isdir(self.dir):
            with open(os.path.join(self.chip_dir, 'export'), 'w') as f:
                f.write(str(self.channel))
        if not os.path.isdir(self.dir):
            raise BeeperError("%s did not appear after export" % (self.dir,))
        # Known state: off. Then "prime" the output once: the kernel driver
        # enables the pad output (PWM_OEN) on the first apply, after which a
        # disabled channel drives its idle level (low) instead of leaving
        # the pad undriven. 25 kHz with a 100 ns pulse is inaudible.
        self._write('enable', 0)
        self.enabled = False
        self._write('duty_cycle', 0)
        self.duty = 0
        self._set_period(40000)
        self._write('duty_cycle', 100)
        self.duty = 100
        self._write('enable', 1)
        self._write('enable', 0)
        self.enabled = False

    def _set_period(self, period):
        if period != self.period:
            self._write('period', period)
            self.period = period

    def tone(self, freq, duty):
        period = NS_PER_S // freq
        duty_ns = max(1, min(period - 1, int(period * duty)))
        if period != self.period:
            # duty must never exceed the period (the PWM core refuses it),
            # and the vendor driver programs the prescale only while the
            # channel is off: off, duty 0, period, duty, on.
            if self.enabled:
                self._write('enable', 0)
                self.enabled = False
            if self.duty:
                self._write('duty_cycle', 0)
                self.duty = 0
            self._set_period(period)
        if duty_ns != self.duty:
            # Run 19 architect A5: the vendor driver spins on pwm_busy() when
            # an ENABLED channel is reprogrammed (up to one period on the
            # reactor) - so a duty-only change also goes through off first.
            if self.enabled:
                self._write('enable', 0)
                self.enabled = False
            self._write('duty_cycle', duty_ns)
            self.duty = duty_ns
        if not self.enabled:
            self._write('enable', 1)
            self.enabled = True

    def off(self):
        if self.enabled is not False:
            self._write('enable', 0)
            self.enabled = False


######################################################################
# Klipper object
######################################################################

class NebulaOSBeeper:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        chip_dir = config.get('pwmchip', '/sys/class/pwm/pwmchip0')
        channel = config.getint('channel', 3, minval=0, maxval=15)
        self.pwm = SysfsPwm(chip_dir, channel)
        self.songs_file = config.get(
            'songs_file', '/opt/printer_data/config/songs.conf')
        self.default_song = config.get('default_song', 'mario')
        self.duty = config.getfloat('duty', 0.5, above=0., maxval=0.5)
        self.click_freq = config.getint('click_frequency', 260,
                                        minval=MIN_FREQ, maxval=MAX_FREQ)
        self.click_ms = config.getint('click_duration_ms', 4,
                                      minval=1, maxval=200)
        self.click_duty = config.getfloat('click_duty', 0.02,
                                          above=0., maxval=0.5)
        self.max_beep_ms = config.getint('max_beep_ms', 10000, minval=1)
        self.max_tune_ms = int(1000. * config.getfloat(
            'max_tune_seconds', 120., above=0.))
        self.max_tune_printing_ms = int(1000. * config.getfloat(
            'max_tune_seconds_printing', 20., above=0.))
        self.max_queue_ms = int(1000. * config.getfloat(
            'max_queue_seconds', 180., above=0.))
        self.available = False
        self.last_error = ""
        self.toolhead = self.mcu = self.print_stats = None
        # (start, end, freq, duty) in reactor time, sorted, non-overlapping
        self.segments = collections.deque()
        self.queue_end = 0.
        self.playing = None
        self.timer = self.reactor.register_timer(self._timer_event)
        self.printer.register_event_handler('klippy:connect',
                                            self._handle_connect)
        self.printer.register_event_handler('klippy:shutdown', self._silence)
        self.printer.register_event_handler('klippy:disconnect', self._silence)
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('NEBULAOS_BEEP', self.cmd_NEBULAOS_BEEP,
                               desc=self.cmd_NEBULAOS_BEEP_help)
        gcode.register_command('NEBULAOS_PLAY_TUNE',
                               self.cmd_NEBULAOS_PLAY_TUNE,
                               desc=self.cmd_NEBULAOS_PLAY_TUNE_help)
        gcode.register_command('NEBULAOS_BEEPER_STOP',
                               self.cmd_NEBULAOS_BEEPER_STOP,
                               desc=self.cmd_NEBULAOS_BEEPER_STOP_help)
        webhooks = self.printer.lookup_object('webhooks')
        webhooks.register_endpoint('nebulaos_beeper/click',
                                   self._handle_click)

    # -- lifecycle ------------------------------------------------------
    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead', None)
        self.mcu = self.printer.lookup_object('mcu', None)
        self.print_stats = self.printer.lookup_object('print_stats', None)
        try:
            self.pwm.setup()
            self.available = True
        except (OSError, IOError, BeeperError) as e:
            self.available = False
            self.last_error = "beeper unavailable: %s" % (e,)
            logging.warning("nebulaos_beeper: %s", self.last_error)

    def _silence(self):
        self.segments.clear()
        self.queue_end = 0.
        self.playing = None
        self.reactor.update_timer(self.timer, self.reactor.NEVER)
        if self.available:
            try:
                self.pwm.off()
            except (OSError, IOError):
                logging.exception("nebulaos_beeper: off failed")

    def _fail(self, e):
        # A hardware write failed mid-sound: stop, report once, stay quiet.
        self.last_error = "beeper write failed: %s" % (e,)
        logging.warning("nebulaos_beeper: %s", self.last_error)
        self.available = False
        self.segments.clear()
        self.playing = None
        try:
            self.pwm.off()
        except (OSError, IOError):
            pass

    def _is_printing(self):
        if self.print_stats is None:
            return False
        try:
            st = self.print_stats.get_status(self.reactor.monotonic())
        except Exception:
            return False
        return st.get('state') == 'printing'

    # -- scheduling -----------------------------------------------------
    def _queued_ms(self, now):
        return max(0., self.queue_end - now) * 1000.

    def _enqueue(self, start, segments):
        now = self.reactor.monotonic()
        if self._queued_ms(now) > self.max_queue_ms:
            logging.info("nebulaos_beeper: queue full, sound dropped")
            return False
        t = max(start, now, self.queue_end)
        for freq, ms, duty in segments:
            end = t + ms / 1000.
            if freq >= MIN_FREQ and ms > 0:
                self.segments.append((t, end, freq, duty))
            t = end
        self.queue_end = t
        self.reactor.update_timer(self.timer, self.reactor.NOW)
        return True

    def _enqueue_at_print_time(self, print_time, segments):
        now = self.reactor.monotonic()
        start = now
        if self.mcu is not None:
            start = now + (print_time - self.mcu.estimated_print_time(now))
        self._enqueue(start, segments)

    def _schedule(self, segments):
        # Sound at the print position of this command, without holding the
        # G-code queue (same mechanism as output_pin SET_PIN).
        if self.toolhead is None:
            self._enqueue(self.reactor.monotonic(), segments)
            return
        self.toolhead.register_lookahead_callback(
            lambda pt: self._enqueue_at_print_time(pt, segments))

    def _timer_event(self, eventtime):
        try:
            while self.segments and self.segments[0][1] <= eventtime:
                self.segments.popleft()
            if not self.segments:
                self.playing = None
                self.pwm.off()
                return self.reactor.NEVER
            start, end, freq, duty = self.segments[0]
            if start > eventtime:
                self.playing = None
                self.pwm.off()
                return start
            if self.playing != (freq, duty):
                self.pwm.tone(freq, duty)
                self.playing = (freq, duty)
            # back-to-back segment with the same tone keeps the PWM running
            return end
        except (OSError, IOError) as e:
            self._fail(e)
            return self.reactor.NEVER

    # -- commands -------------------------------------------------------
    def _require(self, gcmd):
        if not self.available:
            gcmd.respond_info(self.last_error or "beeper unavailable")
            return False
        return True

    cmd_NEBULAOS_BEEP_help = (
        "Beep the buzzer: FREQ=<Hz> DURATION=<ms> [DUTY=<0..0.5>]"
        " (does not hold the G-code queue)")

    def cmd_NEBULAOS_BEEP(self, gcmd):
        freq = gcmd.get_int('FREQ', 1000, minval=0, maxval=MAX_FREQ)
        ms = gcmd.get_int('DURATION', 100, minval=0)
        duty = gcmd.get_float('DUTY', self.duty, above=0., maxval=0.5)
        if not self._require(gcmd):
            return
        ms = min(ms, self.max_beep_ms)
        if freq < MIN_FREQ:
            # M300 S0 = a rest of P ms between beeps
            self._schedule([(0, ms, 0.)])
        else:
            self._schedule([(freq, ms, duty)])

    cmd_NEBULAOS_PLAY_TUNE_help = (
        "Play SONG=<name from songs.conf> or RTTTL=<string> without holding"
        " the G-code queue")

    def cmd_NEBULAOS_PLAY_TUNE(self, gcmd):
        rtttl = gcmd.get('RTTTL', '')
        song = gcmd.get('SONG', self.default_song)
        if not self._require(gcmd):
            return
        if not rtttl:
            try:
                rtttl = find_song(self.songs_file, song)
            except (OSError, IOError) as e:
                raise gcmd.error("Cannot read %s: %s" % (self.songs_file, e))
            if rtttl is None:
                raise gcmd.error("Song '%s' not found in %s"
                                 % (song, self.songs_file))
        segments = parse_rtttl(rtttl, self.duty)
        if not segments:
            raise gcmd.error("No notes in tune")
        printing = self._is_printing()
        limit = self.max_tune_printing_ms if printing else self.max_tune_ms
        segments, cut = cap_segments(segments, limit)
        if cut:
            gcmd.respond_info("Tune shortened to %.0f s%s" % (
                limit / 1000., " while printing" if printing else ""))
        self._schedule(segments)

    cmd_NEBULAOS_BEEPER_STOP_help = "Stop the buzzer and drop queued sounds"

    def cmd_NEBULAOS_BEEPER_STOP(self, gcmd):
        self._silence()

    # -- API (screen touch click) ----------------------------------------
    def _handle_click(self, web_request):
        now = self.reactor.monotonic()
        if self.available and self.queue_end <= now:
            self._enqueue(now, [(self.click_freq, self.click_ms,
                                 self.click_duty)])
        web_request.send({})

    def get_status(self, eventtime):
        return {
            'available': self.available,
            'playing': self.playing is not None,
            'queued_ms': int(self._queued_ms(eventtime)),
            'error': self.last_error,
        }


def load_config(config):
    return NebulaOSBeeper(config)
