# NebulaOS weight-free load-cell contact detection - pure helpers (G83b)
#
# Stages A and B of NebulaOS-firmware/docs/NEBULAOS_RAW_CONTACT_BRIEF.md:
#   A - no-motion evidence: idle-signal statistics and health checks, polarity
#       normalisation, and a replay parser for the contact traces
#       nebulaos_z_offset_probe already writes to klippy.log ("raw collector
#       samples" / "ascent fit window data"), so a raw-count fit can be
#       compared with the calibrated grams fit on the SAME real samples.
#   B - the qualified-profile registry raw contact needs before it may move.
#
# G99 (Tim 2026-10-22, final - overrides the brief's "qualify first" rule and
# the G83b-auto qualification deferral): automatic (raw-count) load-cell
# contact is the DEFAULT for every user, with the profile "ke-stock-v1"
# below. The user choice auto|manual is persisted by
# nebulaos_load_cell_mode.py; resolve_arming() follows it for EVERY caller
# (Z-offset alone and inside auto-calibrate, nozzle clean, any touch_probe
# caller). Manual = the unit's own grams (bottle) calibration, as before.
# Every safety check stays mandatory in automatic mode: sensor health
# window, drift/noise refusal with the settle loop, the reference-anchored
# raw safety window and the reference drift check (nebulaos_z_offset_probe).
#
# No Klipper imports here: everything is plain data in, plain data out, so it
# is tested offline (test_nebulaos_raw_contact.py) and used by
# nebulaos_z_offset_probe.py at runtime.
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import ast
import math
import re

MODE_CALIBRATED = 'calibrated_force'
MODE_RAW = 'raw_contact'
# [nebulaos_z_offset_probe] contact_detection_mode (config). 'auto' (the
# default) follows the user's persisted choice (USER_MODES below);
# 'calibrated_force' / 'raw_contact' force manual / automatic (developer
# override, never written into an image-owned cfg).
MODE_AUTO = 'auto'
MODES = (MODE_AUTO, MODE_CALIBRATED, MODE_RAW)

# G99: the user's choice for load-cell contact (NEBULAOS_LOAD_CELL_MODE).
USER_AUTO = 'auto'          # automatic raw-count contact (default)
USER_MANUAL = 'manual'      # the bottle calibration (grams)
USER_MODES = (USER_AUTO, USER_MANUAL)

# Callers of a nozzle touch. Since G99 they all follow the user's mode; the
# name is kept for logs and status only.
CALLER_Z_OFFSET_PAIR = 'z_offset_pair'
CALLER_AUTO_CALIBRATE = 'auto_calibrate'
CALLER_NOZZLE_CLEAN = 'nozzle_clean'
CALLER_TOUCH = 'touch'      # any other touch_probe() caller
ARM_CALIBRATED = 'calibrated'
ARM_RAW = 'raw'

# A raw-contact profile (brief section 5). Every field is required.
PROFILE_FIELDS = ('id', 'polarity', 'trigger_counts', 'safety_counts',
                  'max_reference_drift_counts', 'expected_sps',
                  'sps_tolerance', 'max_idle_sigma', 'max_idle_drift_per_s',
                  'evidence')

# Rejection codes for a no-motion idle check (stable strings for status/UI).
NO_SAMPLES = 'no_samples'
TOO_FEW_SAMPLES = 'too_few_samples'
SENSOR_ERRORS = 'sensor_errors'
SATURATED = 'saturated'
STUCK = 'stuck_signal'
NON_FINITE = 'non_finite_sample'
SAMPLE_RATE_MISMATCH = 'sample_rate_mismatch'
TIME_NOT_MONOTONIC = 'time_not_monotonic'
# Profile limits in the settle loop (nebulaos_z_offset_probe): prefixes.
NOISE_ABOVE_PROFILE = 'noise_above_profile'
DRIFT_ABOVE_PROFILE = 'drift_above_profile'

# Readable text for the codes above (the screen and Mainsail show it).
IDLE_REASON_TEXT = {
    NO_SAMPLES: "the sensor sent no readings",
    TOO_FEW_SAMPLES: "the sensor sent too few readings",
    SENSOR_ERRORS: "the sensor reported read errors",
    SATURATED: "the sensor reading is at its limit",
    STUCK: "the sensor reading does not change at all",
    NON_FINITE: "the sensor sent invalid readings",
    SAMPLE_RATE_MISMATCH: "the sensor reads at the wrong speed",
    TIME_NOT_MONOTONIC: "the sensor readings arrive out of order",
    NOISE_ABOVE_PROFILE: "the reading is too noisy",
    DRIFT_ABOVE_PROFILE: "the reading does not settle (something touches "
                         "the bed or the printer moves?)",
}


def describe_idle_reasons(reasons):
    """'a, b' in words for validate_idle()/settle-loop reason codes."""
    out = []
    for r in reasons:
        key = next((k for k in (NOISE_ABOVE_PROFILE, DRIFT_ABOVE_PROFILE)
                    if r.startswith(k)), r)
        text = IDLE_REASON_TEXT.get(key, r)
        if text not in out:
            out.append(text)
    return ", ".join(out)

# Rejection codes for raw-contact motion (stage B gate).
RAW_NO_PROFILE = 'raw_contact_no_qualified_profile'
RAW_UNKNOWN_PROFILE = 'raw_contact_unknown_profile'
RAW_NO_REFERENCE = 'raw_contact_no_reference'
RAW_REFERENCE_DRIFT = 'raw_contact_reference_drift'
RAW_UNHEALTHY = 'raw_contact_sensor_unhealthy'
RAW_BAD_PROFILE = 'raw_contact_invalid_profile'
RAW_AUTO_CHECK_FAILED = 'load_cell_auto_check_failed'
NOT_CALIBRATED = 'load_cell_not_calibrated'

# Versioned raw-contact profiles, keyed by id. Every safety value of a touch
# comes from here; an id named in a config file is not enough by itself.
#
# ke-stock-v1 - Creality Ender-3 V3 KE, stock under-bed HX711 (front-left
# screw), upstream hx71x at 80 SPS configured. Tim's decision 2026-10-22
# (G99, final): automatic contact is the default for everyone with these
# values. Evidence: two supervised NEBULAOS_RAW_CONTACT_COMPARE runs on
# ke-dev 2026-10-21 (TRIGGER_COUNTS=6400 POLARITY=-1 SAMPLES=3): automatic vs
# calibrated Z offset within 0.006 mm, automatic scatter 0.014 mm, the same
# ~255 g peak force; 9 real contact traces (testdata/ke-dev-contact-traces-
# 20261019.log): contact LOWERS the counts (polarity -1), peaks 15000-25000
# counts, idle sigma 110 / p2p 604 counts, 90.6 SPS measured. The window
# values are the ones the compare's stage-C candidate used on ke-dev
# (counts_per_gram 85.72084, trigger_force 75 g, force_safety_limit 2000 g):
# safety = 2000 g, reference drift = 75 g, idle sigma <= trigger/10, idle
# drift <= trigger/8 per s. NOT a multi-unit force qualification (G83b-auto
# q stays open): a unit whose cell is much more or less sensitive gets a
# lower or higher contact force - Manual (bottle) remains for that.
KE_STOCK_V1 = 'ke-stock-v1'
DEFAULT_PROFILE_ID = KE_STOCK_V1
QUALIFIED_PROFILES = {
    KE_STOCK_V1: {
        'id': KE_STOCK_V1,
        'polarity': -1,
        'trigger_counts': 6400,
        'safety_counts': 171441,             # int(85.72084 * 2000)
        'max_reference_drift_counts': 6429,  # int(85.72084 * 75)
        'expected_sps': 80.,
        'sps_tolerance': 0.25,
        'max_idle_sigma': 640.,              # 6400 / 10
        'max_idle_drift_per_s': 800.,        # 6400 / 8
        'evidence': "ke-dev supervised compares 2026-10-21 (within 0.006 mm "
                    "of the calibrated method); Tim's decision 2026-10-22 "
                    "(G99): default for everyone",
    },
}


def effective_user_mode(config_mode, user_mode):
    """The user mode a touch follows: the config override, else the
    persisted choice (anything unknown -> automatic, the default)."""
    if config_mode == MODE_CALIBRATED:
        return USER_MANUAL
    if config_mode == MODE_RAW:
        return USER_AUTO
    return user_mode if user_mode in USER_MODES else USER_AUTO


def validate_profile(profile):
    """[] for a usable profile, else the problems. A profile without
    polarity is refused: the raw fit needs it (compression_signal)."""
    if not isinstance(profile, dict):
        return ["profile is not a dict"]
    problems = ["missing %s" % (f,) for f in PROFILE_FIELDS
                if profile.get(f) is None]
    if problems:
        return problems
    if profile['polarity'] not in (1, -1):
        problems.append("polarity must be +1 or -1")
    for f in ('trigger_counts', 'safety_counts', 'max_reference_drift_counts',
              'max_idle_sigma', 'max_idle_drift_per_s', 'expected_sps'):
        v = profile[f]
        if not isinstance(v, (int, float)) or not math.isfinite(v) or v <= 0:
            problems.append("%s must be a positive number" % (f,))
    if not problems and profile['trigger_counts'] >= profile['safety_counts']:
        problems.append("trigger_counts must be below safety_counts")
    return problems


def candidate_profile(trigger_counts, polarity, counts_per_gram,
                      trigger_force, force_safety_limit, expected_sps,
                      force_tolerance=0.10):
    """A stage-C CANDIDATE for the supervised comparison on a CALIBRATED unit
    (brief PR C). Never shipped as constants and never reachable through
    lookup_profile(): built at run time and cross-checked against this
    unit's calibrated force envelope. Returns (profile, problems).

    - trigger_counts / counts_per_gram may not exceed trigger_force (+10%);
    - safety_counts = force_safety_limit in counts (the calibrated window);
    - the reference may have moved at most trigger_force since calibration;
    - idle noise must be 10x below the trigger and drift over a ~2 s
      descent below a quarter of it (relative rules, candidate only)."""
    problems = []
    if counts_per_gram is None or counts_per_gram <= 0:
        return None, ["the load cell must be calibrated (manual is the "
                      "reference for the comparison)"]
    if not isinstance(trigger_counts, int) or trigger_counts <= 0:
        problems.append("TRIGGER_COUNTS must be a positive whole number")
    elif trigger_counts / counts_per_gram > trigger_force * (1. + force_tolerance):
        problems.append(
            "TRIGGER_COUNTS %d is %.0f g on this unit, above trigger_force "
            "%.0f g - refused" % (trigger_counts,
                                  trigger_counts / counts_per_gram,
                                  trigger_force))
    if problems:
        return None, problems
    profile = {
        'id': 'candidate-%d' % (trigger_counts,),
        'polarity': polarity,
        'trigger_counts': trigger_counts,
        'safety_counts': int(counts_per_gram * force_safety_limit),
        'max_reference_drift_counts': int(counts_per_gram * trigger_force),
        'expected_sps': expected_sps,
        'sps_tolerance': 0.25,
        'max_idle_sigma': trigger_counts / 10.,
        'max_idle_drift_per_s': trigger_counts / 8.,
        'evidence': "run-time stage-C candidate on a calibrated unit; "
                    "abs_ge trigger (an opposite-sign event can only stop "
                    "a touch early)",
    }
    problems = validate_profile(profile)
    return (None if problems else profile), problems


def resolve_arming(mode, caller, calibrated, profile_id=None,
                   reference_counts=None, auto_check_failed=False):
    """How a touch is armed: (ARM_CALIBRATED, None), (ARM_RAW, profile) or
    (None, (code, message)) for a refusal. Pure.

    mode: the user mode ('auto' | 'manual'; the config names raw_contact /
    calibrated_force are accepted as their equivalents). G99: the same rule
    for EVERY caller (the Run 14 "auto-calibrate / nozzle clean never raw"
    rule is lifted by Tim's decision 2026-10-22).
    - manual: the unit's grams calibration, else a refusal (bottle popup);
    - auto: a qualified profile + an unloaded reference from a passed
      automatic check, else a refusal. Never falls back to the other mode
      silently."""
    if mode in (USER_MANUAL, MODE_CALIBRATED):
        if calibrated:
            return ARM_CALIBRATED, None
        return None, (NOT_CALIBRATED, "the load cell is not calibrated "
                      "(Manual mode) - run the bottle calibration "
                      "(NEBULAOS_LOAD_CELL_CALIBRATE) or select Automatic "
                      "(NEBULAOS_LOAD_CELL_MODE MODE=auto)")
    refusal = raw_motion_refusal(profile_id)
    if refusal is not None:
        return None, refusal
    profile = lookup_profile(profile_id)
    problems = validate_profile(profile)
    if problems:
        return None, (RAW_BAD_PROFILE, "; ".join(problems))
    if auto_check_failed:
        return None, (RAW_AUTO_CHECK_FAILED,
                      "the automatic load cell check failed - run it again "
                      "on an empty bed (NEBULAOS_LOAD_CELL_AUTO_CHECK) or "
                      "use Calibration > Load Cell > Manual")
    if reference_counts is None:
        return None, (RAW_NO_REFERENCE,
                      "the automatic load cell check has not run yet - run "
                      "it on an empty bed (NEBULAOS_LOAD_CELL_AUTO_CHECK) "
                      "or use Calibration > Load Cell > Manual")
    return ARM_RAW, profile


def lookup_profile(profile_id):
    """The qualified profile for this id, or None. Raw contact may only move
    with a profile from this registry - an id named in a config file is not
    a qualification by itself."""
    if not profile_id:
        return None
    return QUALIFIED_PROFILES.get(profile_id)


def raw_motion_refusal(profile_id):
    """(code, message) why raw_contact may not move, or None when allowed."""
    if not profile_id:
        return (RAW_NO_PROFILE,
                "automatic load cell contact has no profile configured - "
                "nozzle contact refused (use Manual: "
                "NEBULAOS_LOAD_CELL_CALIBRATE)")
    if lookup_profile(profile_id) is None:
        return (RAW_UNKNOWN_PROFILE,
                "raw_contact profile %r is not a qualified profile - nozzle "
                "contact refused" % (profile_id,))
    return None


def _median(values):
    s = sorted(values)
    n = len(s)
    if n == 0:
        return None
    mid = n // 2
    return float(s[mid]) if n % 2 else (s[mid - 1] + s[mid]) / 2.


def robust_stats(times, counts):
    """Idle-signal statistics over raw ADC counts.

    median, robust sigma (MAD * 1.4826), peak-to-peak, least-squares drift in
    counts/second, distinct values, measured samples per second."""
    n = len(counts)
    out = {'n': n, 'median': None, 'sigma': None, 'p2p': None,
           'drift_per_s': None, 'unique': len(set(counts)), 'sps': None}
    if n == 0:
        return out
    med = _median(counts)
    mad = _median([abs(c - med) for c in counts])
    out['median'] = med
    out['sigma'] = mad * 1.4826
    out['p2p'] = max(counts) - min(counts)
    if n >= 2 and times[-1] > times[0]:
        out['sps'] = (n - 1) / (times[-1] - times[0])
        t0 = times[0]
        ts = [t - t0 for t in times]
        mt = sum(ts) / n
        mc = sum(counts) / float(n)
        den = sum((t - mt) ** 2 for t in ts)
        if den > 0:
            out['drift_per_s'] = sum((t - mt) * (c - mc)
                                     for t, c in zip(ts, counts)) / den
    return out


def validate_idle(samples, errors=None, sensor_range=None, expected_sps=None,
                  sps_tolerance=0.25, min_samples=20):
    """No-motion health check of an idle window.

    samples: upstream LoadCellSampleCollector tuples [time, grams_or_None,
    raw_counts, tare_counts]. errors: (errors, overflows) or None.
    sensor_range: (min, max) saturation limits. Returns (stats, reasons) -
    an empty reasons list means healthy. A quiet signal is not enough: a
    stuck or saturated sensor is rejected, never reported healthy."""
    reasons = []
    if errors and (errors[0] or errors[1]):
        reasons.append(SENSOR_ERRORS)
    if not samples:
        return robust_stats([], []), reasons + [NO_SAMPLES]
    times = [s[0] for s in samples]
    counts = [s[2] for s in samples]
    if not all(isinstance(c, (int, float)) and math.isfinite(c) for c in counts) \
            or not all(isinstance(t, (int, float)) and math.isfinite(t) for t in times):
        return robust_stats([], []), reasons + [NON_FINITE]
    stats = robust_stats(times, counts)
    if len(samples) < min_samples:
        reasons.append(TOO_FEW_SAMPLES)
    if any(b <= a for a, b in zip(times, times[1:])):
        reasons.append(TIME_NOT_MONOTONIC)
    if sensor_range is not None:
        lo, hi = sensor_range
        if any(c <= lo or c >= hi for c in counts):
            reasons.append(SATURATED)
    if len(samples) >= min_samples and stats['unique'] <= 2:
        reasons.append(STUCK)
    if expected_sps and stats['sps'] is not None:
        if abs(stats['sps'] - expected_sps) > sps_tolerance * expected_sps:
            reasons.append(SAMPLE_RATE_MISMATCH)
    return stats, reasons


def compression_signal(raw_counts, tare_counts, polarity):
    """Counts from tare, sign-normalised so compression is positive.
    polarity: +1 when compression raises the raw ADC value, -1 when it lowers
    it. Only a qualified profile may state the polarity."""
    if polarity not in (1, -1):
        raise ValueError("polarity must be +1 or -1")
    return polarity * (raw_counts - tare_counts)


def fit_magnitude(sample, mode, polarity=None):
    """The amplitude the contact fit uses for one collector sample:
    calibrated grams in calibrated_force mode (unchanged behaviour), the
    sign-normalised count delta in raw_contact mode."""
    if mode == MODE_CALIBRATED:
        return sample[1]
    return compression_signal(sample[2], sample[3], polarity)


_RAW_RE = re.compile(r"raw collector samples \((\d+), ascent_start=([-0-9.e]+)"
                     r" window=([-0-9.e]+)\): (\[.*\])\s*$")
_FIT_RE = re.compile(r"ascent fit window data \((\d+) points, grams/z pairs\):"
                     r" (\[.*\])\s*$")


def parse_contact_traces(log_text):
    """Contact traces from klippy.log lines written by nebulaos_z_offset_probe
    (_fit_contact_z): each 'raw collector samples' line is followed by its
    'ascent fit window data' line. Returns a list of dicts with
    ascent_start, window, samples (time, grams, counts, tare) and
    window_pairs ((grams, z) for the ascent window, in sample order)."""
    traces = []
    pending = None
    for line in log_text.splitlines():
        m = _RAW_RE.search(line)
        if m:
            try:
                samples = ast.literal_eval(m.group(4))
            except (ValueError, SyntaxError):
                pending = None
                continue
            pending = {'ascent_start': float(m.group(2)),
                       'window': float(m.group(3)),
                       'samples': [tuple(s) for s in samples]}
            continue
        m = _FIT_RE.search(line)
        if m and pending is not None:
            try:
                pairs = ast.literal_eval(m.group(2))
            except (ValueError, SyntaxError):
                pending = None
                continue
            pending['window_pairs'] = [tuple(p) for p in pairs]
            traces.append(pending)
            pending = None
    return traces


def replay_fit_pairs(trace, polarity):
    """For one parsed trace: the (grams, z) pairs the calibrated fit used and
    the matching (compression_counts, z) pairs from the SAME samples.

    The probe builds window_pairs from the samples inside
    [ascent_start, ascent_start + window] in order, so the n-th window sample
    and the n-th pair share their Z."""
    lo = trace['ascent_start']
    hi = lo + trace['window']
    window_samples = [s for s in trace['samples'] if lo <= s[0] <= hi]
    pairs = trace.get('window_pairs') or []
    if len(window_samples) != len(pairs):
        raise ValueError("trace window has %d samples but %d fit pairs"
                         % (len(window_samples), len(pairs)))
    grams_pairs = [(p[0], p[1]) for p in pairs]
    count_pairs = [(compression_signal(s[2], s[3], polarity), p[1])
                   for s, p in zip(window_samples, pairs)]
    return grams_pairs, count_pairs
