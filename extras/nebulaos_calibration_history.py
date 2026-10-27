# nebulaos_calibration_history.py - "what was calibrated when, with what
# result, and what must be redone" (G104/G105, Run 19).
#
# Pure stdlib, zero Klipper dependencies (like nebulaos_calibration_journal,
# whose atomic write it reuses). One small JSON document next to the
# calibration journal - never Klipper's save_variables, never printer.cfg:
#
#   {"schema": 1,
#    "history": {<flow key>: {"done_at": epoch|null, "summary": str,
#                             "redo": reason|null}, ...},
#    "last_result": null | {"flow", "ok", "finished_at", "summary": [str],
#                           "error_step", "error", "advice"}}
#
# A missing file is an empty history; a corrupt one is an empty history plus
# a warning (load() returns it) - never a start failure.
#
# Staleness rules (Tim 2026-10-23, PLAN Run 19):
#   axis_twist recorded                -> auto_calibrate.redo
#                                         "Axis Twist changed"
#   bed tram shim at front left        -> load_cell.redo + auto_calibrate.redo
#                                         "a shim was added at front left"
#   any flow recorded                  -> its own redo is cleared
#
# pid (G120, Run 21): one key for both heaters. A summary that starts with
# "Bed" or "Hotend" (the screen and the PID_BED/PID_HOTEND macros) replaces
# only that heater's part: "Bed tuned at 65 C; Hotend tuned at 230 C".
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import copy
import json
import os

from . import nebulaos_calibration_journal as calibration_journal

SCHEMA = 1
FILE_NAME = "calibration_history.json"
DEFAULT_HISTORY_PATH = calibration_journal.DEFAULT_JOURNAL_DIR + "/" + FILE_NAME

# Every history key, in the Calibration menu order (G105).
FLOWS = ('bed_tram', 'pid', 'axis_twist', 'auto_calibrate', 'input_shaper',
         'esteps', 'skew', 'load_cell', 'tmc_autotune', 'bed_mesh')
# Flows NebulaOS extensions do not run itself: recorded by the screen or a
# firmware macro with NEBULAOS_CALIBRATION_RECORD. The others record
# themselves on success and cannot be recorded from outside.
EXTERNAL_FLOWS = ('pid', 'axis_twist', 'skew', 'tmc_autotune', 'bed_mesh')

MAX_SUMMARY_LEN = 120
MAX_LINES = 6

REDO_AXIS_TWIST = "Axis Twist changed"
REDO_SHIM_FRONT_LEFT = "a shim was added at front left"


def sanitize_text(text, limit=MAX_SUMMARY_LEN):
    """One printable line: surrounding quotes removed, control characters
    dropped, whitespace collapsed, at most `limit` characters."""
    text = str(text if text is not None else '').strip()
    if len(text) >= 2 and text[0] == text[-1] and text[0] in '"\'':
        text = text[1:-1]
    text = ''.join(c if (c.isprintable() and c not in '\r\n\t') else ' '
                   for c in text)
    text = ' '.join(text.split())
    if len(text) > limit:
        text = text[:limit - 3].rstrip() + '...'
    return text


def empty_entry():
    return {'done_at': None, 'summary': '', 'redo': None}


def empty_state():
    return {'schema': SCHEMA,
            'history': {k: empty_entry() for k in FLOWS},
            'last_result': None}


def _clean_entry(raw):
    entry = empty_entry()
    if not isinstance(raw, dict):
        return entry
    done_at = raw.get('done_at')
    if isinstance(done_at, (int, float)) and not isinstance(done_at, bool):
        entry['done_at'] = done_at
    entry['summary'] = sanitize_text(raw.get('summary', ''))
    redo = raw.get('redo')
    if redo:
        entry['redo'] = sanitize_text(redo)
    return entry


def _clean_last_result(raw):
    if not isinstance(raw, dict) or raw.get('flow') not in FLOWS:
        return None
    summary = raw.get('summary')
    if not isinstance(summary, list):
        summary = []
    finished = raw.get('finished_at')
    if not isinstance(finished, (int, float)) or isinstance(finished, bool):
        finished = None
    return {
        'flow': raw['flow'],
        'ok': bool(raw.get('ok')),
        'finished_at': finished,
        'summary': [sanitize_text(s) for s in summary[:MAX_LINES]],
        'error_step': sanitize_text(raw.get('error_step', '')),
        'error': sanitize_text(raw.get('error', ''), limit=200),
        'advice': sanitize_text(raw.get('advice', ''), limit=200),
    }


def load(path):
    """(state, warning). warning is None, or a sentence for the log/console
    when the file existed but could not be used (the history starts
    empty)."""
    try:
        with open(path, 'rb') as handle:
            payload = handle.read()
    except (IOError, OSError):
        return empty_state(), None       # never written yet
    try:
        data = json.loads(payload.decode('utf-8'))
        if not isinstance(data, dict) or data.get('schema') != SCHEMA:
            raise ValueError("unknown schema")
        history = data.get('history')
        if not isinstance(history, dict):
            raise ValueError("no history")
    except Exception as e:
        return empty_state(), (
            "calibration history %s is unreadable (%s) - starting with an "
            "empty history" % (path, e.__class__.__name__))
    state = empty_state()
    for key in FLOWS:
        state['history'][key] = _clean_entry(history.get(key))
    state['last_result'] = _clean_last_result(data.get('last_result'))
    return state, None


def save(path, state):
    directory = os.path.dirname(path)
    if directory and not os.path.isdir(directory):
        os.makedirs(directory)
    calibration_journal.atomic_write_json(path + ".tmp", path, state)


def mark_redo(state, keys, reason):
    for key in keys:
        state['history'][key]['redo'] = sanitize_text(reason)
    return state


PID_HEATERS = ('bed', 'hotend')


def _pid_heater(clause):
    word = clause.split(' ', 1)[0].lower()
    return word if word in PID_HEATERS else None


def merge_pid_summary(old, new):
    """The new heater's part replaces only its own part of the old summary
    (Bed first); a summary without a known heater replaces everything."""
    heater = _pid_heater(new)
    if heater is None:
        return new
    parts = dict((_pid_heater(c), c) for c in old.split('; ')
                 if _pid_heater(c) is not None)
    parts[heater] = new
    return '; '.join(parts[h] for h in PID_HEATERS if h in parts)


def record(state, key, summary, now):
    """A flow finished successfully: done now, its own redo cleared, then
    the staleness rules it triggers."""
    if key not in FLOWS:
        raise ValueError("unknown calibration flow %r" % (key,))
    summary = sanitize_text(summary)
    if key == 'pid':
        summary = merge_pid_summary(
            state['history'].get(key, empty_entry())['summary'], summary)
    state['history'][key] = {'done_at': now,
                             'summary': sanitize_text(summary),
                             'redo': None}
    if key == 'axis_twist':
        mark_redo(state, ('auto_calibrate',), REDO_AXIS_TWIST)
    return state


def make_result(flow, ok, now, summary=(), error_step='', error='',
                advice=''):
    return _clean_last_result({
        'flow': flow, 'ok': ok, 'finished_at': now, 'summary': list(summary),
        'error_step': error_step, 'error': error, 'advice': advice})


def snapshot(state):
    """Copies for get_status (Klipper compares status values with the
    previous poll: a mutated shared dict would never be reported)."""
    return (copy.deepcopy(state['history']),
            copy.deepcopy(state['last_result']))
