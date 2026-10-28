# NebulaOS load-cell contact mode: the user's persisted choice (G99)
#
# Tim's decision 2026-10-22: automatic load-cell contact is the default for
# everyone. The user can switch between
#   auto   - automatic contact (raw ADC counts, qualified profile
#            nebulaos_raw_contact.KE_STOCK_V1); needs a passed automatic
#            check (no motion, empty bed: sensor health + the unloaded
#            reference), no bottle calibration;
#   manual - the bottle calibration (grams, NEBULAOS_LOAD_CELL_CALIBRATE),
#            used until automatic is selected again.
#
# One small JSON file holds the choice AND the last automatic check (its
# result and the unloaded reference), so neither needs SAVE_CONFIG or a
# Klipper restart. Same rules as nebulaos_z_adjust: written atomically (tmp
# in the same directory + fsync + rename + directory fsync), never with
# Klipper's save_variables; a missing file is the default (auto, never
# checked); a corrupt or implausible file is the default plus a warning -
# never a start failure. The file sits next to the auto-calibrate journal
# (machine data, not in the user's config root).
#
# Plain data in, plain data out: no Klipper imports (tested offline).
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import json
import math
import os

from . import nebulaos_calibration_journal as journal
from . import nebulaos_raw_contact as raw_contact

DEFAULT_PATH = journal.DEFAULT_JOURNAL_DIR + "/load_cell_mode.json"
SCHEMA = 1

CHECK_NEVER = 'never'
CHECK_OK = 'ok'
CHECK_FAILED = 'failed'
CHECK_STATES = (CHECK_NEVER, CHECK_OK, CHECK_FAILED)

# A 24-bit HX711 reading; anything outside is not a reference.
_MAX_COUNTS = 1 << 23
_MAX_REASON_LEN = 300


def default_state():
    return {'mode': raw_contact.USER_AUTO,
            'auto_check': CHECK_NEVER,
            'auto_check_reason': '',
            'reference_counts': None,
            'checked_at': None,
            'profile_id': None,
            # G102 (Run 21, Tim): the sensitivity of the first automatic
            # touch after a passed check (counts per mm of press) - recorded
            # only, never a refusal (yet)
            'depress_slope': None,
            'depress_slope_at': None}


def read_state(path):
    """(state, warning). Never raises: Klipper must always start. Missing
    file -> default without a warning; anything unusable -> default (or the
    usable part) WITH a warning."""
    try:
        return _read_state(path)
    except Exception as e:
        return default_state(), ("%s could not be used (%s) - automatic "
                                 "mode, check not run" % (path, e.__class__.__name__))


def _read_state(path):
    state = default_state()
    try:
        with open(path) as handle:
            data = json.load(handle)
    except (IOError, OSError) as e:
        if not os.path.exists(path):
            return state, ''
        return state, "cannot read %s (%s) - automatic mode" % (path, e)
    except ValueError as e:
        return state, "%s is not valid JSON (%s) - automatic mode" % (path, e)
    if not isinstance(data, dict):
        return state, "%s is not a JSON object - automatic mode" % (path,)
    problems = []
    mode = data.get('mode')
    if mode in raw_contact.USER_MODES:
        state['mode'] = mode
    else:
        problems.append("unknown mode %r" % (mode,))
    check = data.get('auto_check')
    ref = data.get('reference_counts')
    ref_ok = (isinstance(ref, int) and not isinstance(ref, bool)
              and -_MAX_COUNTS <= ref < _MAX_COUNTS)
    if check == CHECK_OK and ref_ok:
        state['auto_check'] = CHECK_OK
        state['reference_counts'] = ref
    elif check == CHECK_FAILED:
        state['auto_check'] = CHECK_FAILED
    elif check not in (None, CHECK_NEVER):
        # an 'ok' without a usable reference is not ok: check again
        problems.append("unusable automatic check result")
    reason = data.get('auto_check_reason')
    if isinstance(reason, str) and state['auto_check'] == CHECK_FAILED:
        state['auto_check_reason'] = reason[:_MAX_REASON_LEN]
    at = data.get('checked_at')
    if isinstance(at, (int, float)) and not isinstance(at, bool) \
            and math.isfinite(at) and state['auto_check'] != CHECK_NEVER:
        state['checked_at'] = at
    pid = data.get('profile_id')
    if isinstance(pid, str) and state['auto_check'] != CHECK_NEVER:
        state['profile_id'] = pid[:64]
    slope = data.get('depress_slope')
    slope_at = data.get('depress_slope_at')
    if _finite(slope) and _finite(slope_at):
        state['depress_slope'] = float(slope)
        state['depress_slope_at'] = slope_at
    if problems:
        return state, "%s: %s - using defaults for it" % (
            path, "; ".join(problems))
    return state, ''


def _finite(value):
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(value))


def write_state(path, state):
    """Atomic: tmp in the same directory + fsync + os.replace + dir fsync."""
    data = {'schema': SCHEMA}
    data.update({k: state.get(k) for k in default_state()})
    directory = os.path.dirname(path)
    if directory:
        os.makedirs(directory, exist_ok=True)
    journal.atomic_write_json(path + ".tmp", path, data)
