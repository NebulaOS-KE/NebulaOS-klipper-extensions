# NebulaOS weight-free load-cell contact detection - pure helpers (G83b)
#
# Stages A and B of NebulaOS-firmware/docs/NEBULAOS_RAW_CONTACT_BRIEF.md:
#   A - no-motion evidence: idle-signal statistics and health checks, polarity
#       normalisation, and a replay parser for the contact traces
#       nebulaos_z_offset_probe already writes to klippy.log ("raw collector
#       samples" / "ascent fit window data"), so a raw-count fit can be
#       compared with the calibrated grams fit on the SAME real samples.
#   B - the qualified-profile registry the opt-in raw_contact mode needs
#       before it may move. It is EMPTY: no profile has passed the supervised
#       force qualification (stages C/D), so raw_contact refuses every
#       nozzle-contact motion. Do not add a profile without measured-force
#       evidence; never invent thresholds or a fake counts_per_gram.
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
# G83b-auto (Run 14): the default. Per caller: the unit's own grams
# calibration when there is one (per-unit evidence beats a model profile),
# otherwise a QUALIFIED raw profile - only for the canonical paired Z-offset
# path - otherwise a refusal (the bottle calibration popup).
MODE_AUTO = 'auto'
MODES = (MODE_AUTO, MODE_CALIBRATED, MODE_RAW)

# The only caller that may ever resolve to raw contact: the canonical
# NEBULAOS_Z_OFFSET_CALIBRATE paired path (fixed reference XY, bounded
# floor). Nozzle cleaning, the legacy Z_OFFSET_CALIBRATION and
# NEBULAOS_AUTO_CALIBRATE stay on calibrated force (brief E.2-E.4).
CALLER_Z_OFFSET_PAIR = 'z_offset_pair'
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

# Rejection codes for raw-contact motion (stage B gate).
RAW_NO_PROFILE = 'raw_contact_no_qualified_profile'
RAW_UNKNOWN_PROFILE = 'raw_contact_unknown_profile'
RAW_NO_REFERENCE = 'raw_contact_no_reference'
RAW_REFERENCE_DRIFT = 'raw_contact_reference_drift'
RAW_UNHEALTHY = 'raw_contact_sensor_unhealthy'
RAW_BAD_PROFILE = 'raw_contact_invalid_profile'
NOT_CALIBRATED = 'load_cell_not_calibrated'

# Versioned, hardware-qualified raw-contact profiles, keyed by id. Each entry
# must come from a documented supervised force qualification (brief, PR C/D):
# polarity, trigger band, reference drift window, ADC safety window, noise /
# drift / sample-rate bounds, fit-quality limits. None exist yet.
QUALIFIED_PROFILES = {}


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
                   reference_counts=None):
    """How a touch is armed: (ARM_CALIBRATED, None), (ARM_RAW, profile) or
    (None, (code, message)) for a refusal. Pure - see MODE_AUTO above."""
    if caller != CALLER_Z_OFFSET_PAIR or mode == MODE_CALIBRATED:
        if calibrated:
            return ARM_CALIBRATED, None
        return None, (NOT_CALIBRATED, "the load cell is not calibrated")
    if mode == MODE_AUTO and calibrated:
        return ARM_CALIBRATED, None
    refusal = raw_motion_refusal(profile_id)
    if refusal is not None:
        if mode == MODE_AUTO:
            return None, (NOT_CALIBRATED, "the load cell is not calibrated")
        return None, refusal
    if reference_counts is None:
        return None, (RAW_NO_REFERENCE, "raw contact has no unloaded "
                      "reference for this unit - nozzle contact refused")
    profile = lookup_profile(profile_id)
    problems = validate_profile(profile)
    if problems:
        return None, (RAW_BAD_PROFILE, "; ".join(problems))
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
                "raw_contact mode has no hardware-qualified profile yet - "
                "nozzle contact refused (use the calibrated load cell: "
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
