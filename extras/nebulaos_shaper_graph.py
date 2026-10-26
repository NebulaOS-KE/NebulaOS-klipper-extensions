# NebulaOS input shaper graph (G109, Run 20).
#
# A standalone script, not a Klipper module (no load_config): nebulaos_
# calibration runs it as a low-priority child process after upstream's
# SHAPER_CALIBRATE has written its CSV, so matplotlib never runs inside
# Klippy and a slow or failed graph never blocks or fails a calibration.
#
#   python3 nebulaos_shaper_graph.py CSV AXIS SHAPER FREQ SCREEN_PNG [BIG_PNG]
#
# CSV is upstream's /tmp/calibration_data_<axis>_<name>.csv
# (shaper_calibrate.save_calibration_data: freq, psd_x, psd_y, psd_z,
# psd_xyz, then "shapers:" and one response column per fitted shaper, e.g.
# "mzv(52.4)"). It draws the measured vibration (X+Y+Z power spectral
# density), what is left after the chosen shaper and the shaper's response:
# SCREEN_PNG 480x272 dark for NebulaScreen, BIG_PNG 1200x700 light for
# Mainsail. Each file is written to a temporary name and renamed, so a
# reader never sees half a PNG. Exit code 0 = all written, otherwise a one
# line reason on stderr.
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import os
import sys


def read_csv(path):
    """(freqs, psd_xyz, {shaper_name: (freq, values)}) from upstream's CSV."""
    with open(path) as f:
        header = f.readline().strip().split(',')
        rows = [line.strip().split(',') for line in f if line.strip()]
    if not header or header[0] != 'freq' or 'psd_xyz' not in header:
        raise ValueError("not a SHAPER_CALIBRATE file (header %r)" % (header[:5],))
    i_psd = header.index('psd_xyz')
    shapers = {}
    for i, name in enumerate(header):
        if '(' in name and name.endswith(')'):
            shaper, _, freq = name[:-1].partition('(')
            try:
                shapers[shaper] = (float(freq), i)
            except ValueError:
                continue
    freqs, psd = [], []
    values = dict((s, []) for s in shapers)
    for r in rows:
        if len(r) < len(header):
            continue
        freqs.append(float(r[0]))
        psd.append(float(r[i_psd]))
        for s, (_freq, i) in shapers.items():
            values[s].append(float(r[i]))
    if len(freqs) < 2:
        raise ValueError("no frequency data in %s" % (path,))
    return freqs, psd, dict((s, (shapers[s][0], values[s])) for s in shapers)


THEMES = {
    'dark': {'bg': '#202020', 'fg': '#e8e8e8', 'grid': '#404040',
             'psd': '#5ab0ff', 'after': '#ffb347', 'shaper': '#8bc34a'},
    'light': {'bg': '#ffffff', 'fg': '#202020', 'grid': '#d8d8d8',
              'psd': '#1f6fd0', 'after': '#e07b00', 'shaper': '#4a8f1c'},
}


def render(freqs, psd, shapers, axis, shaper, freq, out, size, dpi, theme):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    t = THEMES[theme]
    small = size[0] <= 600
    fig = plt.figure(figsize=(size[0] / float(dpi), size[1] / float(dpi)), dpi=dpi)
    fig.patch.set_facecolor(t['bg'])
    ax = fig.add_subplot(1, 1, 1)
    ax.set_facecolor(t['bg'])
    fs = 7 if small else 11
    ax.plot(freqs, psd, color=t['psd'], linewidth=1.2, label='Measured')
    resp = shapers.get(shaper)
    if resp is not None:
        after = [p * v for p, v in zip(psd, resp[1])]
        ax.plot(freqs, after, color=t['after'], linewidth=1.2,
                label='After %s' % (shaper,))
    ax.set_xlim(freqs[0], freqs[-1])
    ax.set_ylim(bottom=0)
    ax.set_xlabel('Frequency (Hz)', color=t['fg'], fontsize=fs)
    ax.tick_params(colors=t['fg'], labelsize=fs - 1)
    from matplotlib import ticker
    ax.yaxis.set_major_formatter(ticker.NullFormatter())
    ax.grid(True, color=t['grid'], linewidth=0.5)
    for s in ax.spines.values():
        s.set_color(t['grid'])
    if resp is not None:
        ax2 = ax.twinx()
        ax2.plot(freqs, resp[1], color=t['shaper'], linewidth=0.8, linestyle='--',
                 label='%s response' % (shaper,))
        ax2.set_ylim(0, 1.05)
        ax2.tick_params(colors=t['fg'], labelsize=fs - 1)
        for s in ax2.spines.values():
            s.set_color(t['grid'])
    ax.set_title('%s axis: %s %.1f Hz' % (axis.upper(), shaper, freq),
                 color=t['fg'], fontsize=fs + 2)
    handles, labels = ax.get_legend_handles_labels()
    if resp is not None:
        h2, l2 = ax2.get_legend_handles_labels()
        handles, labels = handles + h2, labels + l2
    leg = ax.legend(handles, labels, loc='upper right', fontsize=fs - 1, frameon=False)
    for txt in leg.get_texts():
        txt.set_color(t['fg'])
    fig.tight_layout(pad=0.4 if small else 1.0)
    tmp = out + '.tmp'
    fig.savefig(tmp, format='png', dpi=dpi, facecolor=t['bg'])
    plt.close(fig)
    os.rename(tmp, out)


def leave_oom_protection():
    """The child starts with Klippy's OOM protection (oom_score_adj -900):
    give it up before matplotlib loads, so on the KE's small RAM the graph -
    not Moonraker or the screen - is what the kernel kills first."""
    try:
        with open('/proc/self/oom_score_adj', 'w') as f:
            f.write('300')
    except (IOError, OSError):
        pass


def main(argv):
    leave_oom_protection()
    if len(argv) not in (6, 7):
        sys.stderr.write("usage: nebulaos_shaper_graph.py CSV AXIS SHAPER FREQ "
                         "SCREEN_PNG [BIG_PNG]\n")
        return 2
    csv, axis, shaper, freq, screen = argv[1:6]
    big = argv[6] if len(argv) == 7 else None
    try:
        freqs, psd, shapers = read_csv(csv)
        for path in (screen, big):
            if path:
                d = os.path.dirname(path)
                if d and not os.path.isdir(d):
                    os.makedirs(d)
        render(freqs, psd, shapers, axis, shaper, float(freq), screen,
               (480, 272), 100, 'dark')
        if big:
            render(freqs, psd, shapers, axis, shaper, float(freq), big,
                   (1200, 700), 100, 'light')
    except Exception as e:
        sys.stderr.write("%s: %s\n" % (type(e).__name__, str(e).splitlines()[0] if str(e) else ''))
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
