"""Make the six review figures for the dark carve-lab page.

Usage: plot_review.py OUT_DIR
"""
import csv
import os
import re
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from mc_summary import envelope  # noqa: E402

MC = os.path.join(HERE, 'mc')
CITY = os.path.join(HERE, '..', 'out', 'city_runs')
BG, TITLE, MUTED, GRID = '#111D27', '#EAF1F1', '#8FA3AA', '#22333F'
PASS, NOSE, TAIL, WARN = '#33C6AC', '#E5604D', '#F2A24A', '#9C8CF0'
EASED = '#CFC8F8'
STRIKE_DEG, MARGIN_LIMIT, BOARD = 18.6, 3.75, 18.4
# X7 nominal build: board 18.4 kg, wheel 4.5 kg, CoM 0.0244 m behind and 0.003 m above the axle, radius 0.146 m
X7 = {'board_kg': BOARD, 'wheel_kg': 4.5, 'com_x': 0.0244, 'com_z': 0.003, 'radius': 0.146}
MARK = {'PASS': 'o', 'TAIL STOP': '^', 'RUNAWAY': 'x', 'EASED STOP': 's', 'DISMOUNT': 'D', 'NOSE STRIKE': 'v'}
COL = {'PASS': PASS, 'TAIL STOP': TAIL, 'RUNAWAY': TAIL, 'EASED STOP': WARN, 'DISMOUNT': WARN, 'NOSE STRIKE': NOSE}
SIZE = 34


def load(name):
    return list(csv.DictReader(open(os.path.join(MC, name))))


def f(x):
    return float(x)


def klass(r):
    if r['status'] == 'FALL':
        return 'NOSE STRIKE' if 'nose' in r['cause'] else 'FALL'
    return r['status']


def style_ax(ax):
    ax.set_facecolor(BG)
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)
    for s in ('left', 'bottom'):
        ax.spines[s].set_color(GRID)
    ax.tick_params(colors=MUTED, labelsize=9)
    ax.set_axisbelow(True)
    ax.grid(True, color=GRID, linewidth=0.6)


def titles(fig, ax, title, subtitle):
    fig.text(0.0, 1.0, title, color=TITLE, fontsize=13, ha='left', va='bottom',
             transform=ax.transAxes + matplotlib.transforms.ScaledTranslation(0, 0.3, fig.dpi_scale_trans))
    ax.text(0.0, 1.03, subtitle, color=MUTED, fontsize=9, ha='left', va='bottom', transform=ax.transAxes)


def new_fig(title, subtitle, size=(8, 5)):
    fig, ax = plt.subplots(figsize=size, dpi=200, facecolor=BG)
    style_ax(ax)
    titles(fig, ax, title, subtitle)
    return fig, ax


def labels(ax, x=None, y=None):
    if x:
        ax.set_xlabel(x, color=MUTED, fontsize=10)
    if y:
        ax.set_ylabel(y, color=MUTED, fontsize=10)


def legend(ax, handles, y=-0.14):
    lg = ax.legend(handles=handles, loc='upper center', bbox_to_anchor=(0.5, y), ncol=len(handles),
                   frameon=False, fontsize=9)
    for t in lg.get_texts():
        t.set_color(TITLE)


def save(fig, out, name):
    fig.savefig(os.path.join(out, name), dpi=200, facecolor=BG, bbox_inches='tight', pad_inches=0.3)
    plt.close(fig)


def handle(k):
    if k == 'RUNAWAY':
        return Line2D([], [], ls='', marker='x', color=COL[k], markersize=6, markeredgewidth=1.5, label=k)
    return Line2D([], [], ls='', marker=MARK[k], color=COL[k], markersize=6,
                  markeredgecolor=BG, markeredgewidth=0.8, label=k)


def scatter_classes(ax, rows, xf, yf, order):
    for k in order:
        s = [r for r in rows if klass(r) == k]
        if not s:
            continue
        xs, ys = [xf(r) for r in s], [yf(r) for r in s]
        if k == 'RUNAWAY':
            ax.scatter(xs, ys, s=SIZE, marker='x', c=COL[k], linewidths=1.5, zorder=3)
        else:
            ax.scatter(xs, ys, s=SIZE, marker=MARK[k], c=COL[k], edgecolors=BG, linewidths=0.8, zorder=3)


def steady_margin(m, g):
    return envelope({**X7, 'rider_kg': m, 'grade_pct': g, 'kt_scale': 1.0, 'amps': 1.0})[0]


def limit_grade(m, target=MARGIN_LIMIT, sign=1):
    lo, hi = 0.0, 100.0
    for _ in range(60):
        mid = (lo + hi) / 2
        if steady_margin(m, sign * mid) > target:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2


CLASSES4 = ['PASS', 'TAIL STOP', 'RUNAWAY', 'NOSE STRIKE']


def fig1(rows, out):
    clr = [STRIKE_DEG - f(r['peak_pitch_deg']) for r in rows]
    bins = [(lambda c: c < 2, NOSE, 'within 2° of a deck strike'),
            (lambda c: 2 <= c < 5, TAIL, '2–5° of clearance left'),
            (lambda c: c >= 5, PASS, '5° or more left')]
    npass = sum(r['status'] == 'PASS' for r in rows)
    fig, ax = new_fig(f"Mike's build: {npass} of {len(rows)} runs ride; how close they came",
                      f'{len(rows)} sim runs · rider {min(f(r["rider_kg"]) for r in rows):.0f}–'
                      f'{max(f(r["rider_kg"]) for r in rows):.0f} kg · grade '
                      f'−{abs(min(f(r["grade_pct"]) for r in rows)):.0f} to +{max(f(r["grade_pct"]) for r in rows):.0f} % · '
                      f'motor {min(f(r["amps"]) for r in rows):.0f}–{max(f(r["amps"]) for r in rows):.0f} A · '
                      'build mass properties dispersed')
    ax.xaxis.grid(True, color=GRID, linewidth=0.6)
    ms = np.linspace(50, 118, 100)
    gc = np.array([limit_grade(m) for m in ms])
    gd = np.array([limit_grade(m, sign=-1) for m in ms])
    ax.plot(gc, ms, ls='--', color=MUTED, lw=1, zorder=2)
    ax.plot(-gd, ms, ls='--', color=MUTED, lw=1, zorder=2)
    ax.text(gc[-1] + 0.6, 112, 'deck strike limit', color=MUTED, fontsize=8, ha='left', va='center')
    gx = [f(r['grade_pct']) for r in rows]
    for test, col, _ in bins:
        s = [r for r, c in zip(rows, clr) if test(c)]
        ax.scatter([f(r['grade_pct']) for r in s], [f(r['rider_kg']) for r in s], s=SIZE, marker='o', c=col,
                   edgecolors=BG, linewidths=0.8, zorder=3)
    ax.set_xlim(min(gx) - 6, max(gx) + 9)
    ax.set_ylim(36, 118)
    close = [r for r, c in zip(rows, clr) if c < 2]
    if close:
        ax.text(max(gx) + 8, 42, 'steepest climbs: under 2° of clearance left', color=NOSE, fontsize=8.5,
                ha='right', va='center')
    labels(ax, 'Grade, % (climb +)', 'Rider mass, kg')
    legend(ax, [Line2D([], [], ls='', marker='o', color=col, markersize=6, markeredgecolor=BG,
                       markeredgewidth=0.8, label=lab) for _, col, lab in bins])
    save(fig, out, 'r01_outcomes_map.png')
    print('fig1: n=%d pass=%d; bins <2: %d, 2-5: %d, >=5: %d; min clearance %.1f; static limit at 110 kg: climb %.1f %%, descent -%.1f %%' %
          (len(rows), npass, sum(c < 2 for c in clr), sum(2 <= c < 5 for c in clr), sum(c >= 5 for c in clr),
           min(clr), limit_grade(110), limit_grade(110, sign=-1)))


def fig2(rows, out):
    fig, ax = new_fig('Every grade costs deck clearance',
                      'Peak deck angle against the road · thin lines: steady lean at 55 and 110 kg · X7 build')
    ax.xaxis.grid(True, color=GRID, linewidth=0.6)
    gx = [f(r['grade_pct']) for r in rows]
    gmax = max(abs(min(gx)), max(gx)) + 3
    ymax = max(f(r['peak_pitch_deg']) for r in rows)
    ytop = max(ymax, STRIKE_DEG) + 3
    ax.set_xlim(-gmax - 4, gmax + 4)
    ax.set_ylim(0, ytop)
    ax.axhspan(0, STRIKE_DEG, color=PASS, alpha=0.06, lw=0, zorder=1)
    ax.axhspan(STRIKE_DEG, ytop, color=NOSE, alpha=0.08, lw=0, zorder=1)
    ax.axhline(STRIKE_DEG, color=NOSE, lw=1, zorder=2)
    ax.text(-gmax + 0.5, STRIKE_DEG + 0.4, 'deck strike, 18.6°', color=NOSE, fontsize=8.5, ha='left', va='bottom')
    g = np.linspace(-gmax - 4, gmax + 4, 600)
    yl = {}
    for m in (55, 110):
        y = np.array([STRIKE_DEG - steady_margin(m, v) for v in g])
        k = y <= ytop
        ax.plot(g[k], y[k], color=MUTED, lw=1, zorder=2)
        yl[m] = (g, y)
    # label each curve on the right slope at 16.5 deg; the outer curve outside, the inner one inside
    ylab = {}
    outer = max(yl, key=lambda m: float(np.interp(16.5, yl[m][1][yl[m][0] > 0], yl[m][0][yl[m][0] > 0])))
    for m in yl:
        ylab[m] = 15.0 if m == outer else 11.0  # both labels sit inside the V, one above the other
    xs = {m: float(np.interp(ylab[m], yl[m][1][yl[m][0] > 0], yl[m][0][yl[m][0] > 0])) for m in yl}
    for m in xs:
        ax.text(xs[m] - 2.2, ylab[m], f'{m} kg steady lean', color=MUTED, fontsize=8,
                ha='right', va='center')
    scatter_classes(ax, rows, lambda r: f(r['grade_pct']), lambda r: f(r['peak_pitch_deg']), CLASSES4)
    present = [k for k in CLASSES4 if any(klass(r) == k for r in rows)]
    # grade at which the 110 kg steady lean leaves 3.75 deg of clearance
    labels(ax, 'Grade, % (climb +)', 'Peak deck angle against road, deg')
    legend(ax, [handle(k) for k in present])
    save(fig, out, 'r02_deck_angle_window.png')
    print('fig2: 110 kg grade at 3.75 deg margin: %.1f %%, 55 kg: %.1f %%' % (limit_grade(110), limit_grade(55)))


def city(name):
    d = np.genfromtxt(os.path.join(CITY, name, 'trace.csv'), delimiter=',', names=True)
    return d


def spans(t, lvl, v):
    on = lvl == v
    out, start = [], None
    for i in range(len(t)):
        if on[i] and start is None:
            start = t[i]
        if not on[i] and start is not None:
            out.append((start, t[i]))
            start = None
    if start is not None:
        out.append((start, t[-1]))
    return out


def fig3(out):
    a = city('climb_heavy_no_warning')
    b = city('climb_heavy_rider_reacts')
    ia = int(np.argmax(a['nose_strike_n'] > 50))
    end_a = a['sim_time_s'][ia] + 0.1
    a = a[a['sim_time_s'] <= end_a]
    host = open(os.path.join(CITY, 'climb_heavy_rider_reacts', 'host.txt')).read()
    t_dis = float(re.search(r'rider dismount at sim_t=([\d.]+)s', host).group(1))
    b = b[b['sim_time_s'] <= t_dis + 1e-6]
    t0 = 1.05
    a = a[a['sim_time_s'] >= t0]
    b = b[b['sim_time_s'] >= t0]
    strike_t = float(a['sim_time_s'][np.argmax(a['nose_strike_n'] > 50)])
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6), dpi=200, facecolor=BG, sharex=True,
                                   gridspec_kw={'hspace': 0.12})
    for ax in (ax1, ax2):
        style_ax(ax)
        ax.xaxis.grid(False)
    titles(fig, ax1, 'The warning comes before the nose drops',
           '110 kg rider, SMALL motor (35 A, Kt 0.88×), 12 % climb — an edge case; the build (90 A) climbs this')
    tb = b['sim_time_s']
    for ax in (ax1, ax2):
        for s, e in spans(tb, b['margin_level'], 1):
            ax.axvspan(s, e, color=WARN, alpha=0.12, lw=0, zorder=1)
        for s, e in spans(tb, b['margin_level'], 2):
            ax.axvspan(s, e, color=WARN, alpha=0.25, lw=0, zorder=1)
    ax1.plot(a['sim_time_s'], a['truth_pitch_deg'], color=NOSE, lw=1.6, zorder=3)
    ax1.plot(tb, b['truth_pitch_deg'], color=WARN, lw=1.6, zorder=3)
    ax1.scatter([a['sim_time_s'][-1]], [a['truth_pitch_deg'][-1]], s=40, marker='v', c=NOSE,
                edgecolors=BG, linewidths=0.8, zorder=4)
    ax1.scatter([tb[-1]], [b['truth_pitch_deg'][-1]], s=40, marker='D', c=WARN,
                edgecolors=BG, linewidths=0.8, zorder=4)
    ax1.annotate('nose strike', (a['sim_time_s'][-1], a['truth_pitch_deg'][-1]), xytext=(-8, 0),
                 textcoords='offset points', color=NOSE, fontsize=8.5, ha='right', va='center')
    ax1.set_ylim(bottom=float(a['truth_pitch_deg'].min()) - 3)
    ax1.annotate('rider steps off', (tb[-1], b['truth_pitch_deg'][-1]), xytext=(-8, 6),
                 textcoords='offset points', color=WARN, fontsize=8.5, ha='right', va='bottom')
    labels(ax1, y='Pitch, deg (nose down negative)')
    ax2.plot(a['sim_time_s'], a['applied_amps'], color=NOSE, lw=1.6, zorder=3)
    ax2.plot(tb, b['applied_amps'], color=WARN, lw=1.6, zorder=3)
    lim = float(max(np.max(np.abs(a['applied_amps'])), np.max(np.abs(b['applied_amps']))))
    ax2.axhline(35, ls='--', color=MUTED, lw=1, zorder=2)
    ax2.text(t0 + 0.05, 35.6, '35 A limit', color=MUTED, fontsize=8, ha='left', va='bottom')
    ax2.set_ylim(top=max(lim, 35) * 1.12)
    labels(ax2, 'Sim time, s', 'Motor current, A')
    ax2.set_xlim(t0, max(a['sim_time_s'][-1], tb[-1]) + 0.15)
    ax2.legend(handles=[Line2D([], [], color=NOSE, lw=2, label='no warning'),
                        Line2D([], [], color=WARN, lw=2, label='warning, rider reacts'),
                        Patch(facecolor=WARN, alpha=0.12, label='pulsed buzz'),
                        Patch(facecolor=WARN, alpha=0.25, label='solid buzz')],
               loc='upper center', bbox_to_anchor=(0.5, -0.28), ncol=4, frameon=False, fontsize=9,
               labelcolor=TITLE)
    save(fig, out, 'r03_warning_timeline.png')
    print('fig3: nose strike at %.2f s, dismount %.3f s, peak current no-warn %.1f, reacts %.1f' %
          (strike_t, t_dis, np.max(np.abs(a['applied_amps'])), np.max(np.abs(b['applied_amps']))))
    for s, e in spans(tb, b['margin_level'], 1) + spans(tb, b['margin_level'], 2):
        print('  warning span %.2f-%.2f' % (s, e))


def fig4(base, warn, noreact, out):
    order = ['PASS', 'TAIL STOP', 'EASED STOP', 'RUNAWAY', 'DISMOUNT', 'NOSE STRIKE']
    order = [k for k in order if any(klass(r) == k for r in base + warn)]
    nwarn = sum(bool(r['t_warn_s']) for r in noreact)
    fc = {'PASS': PASS, 'TAIL STOP': TAIL, 'EASED STOP': EASED, 'RUNAWAY': TAIL, 'DISMOUNT': WARN, 'NOSE STRIKE': NOSE}
    fig = plt.figure(figsize=(8, 5), dpi=200, facecolor=BG)
    gs = fig.add_gridspec(2, 1, height_ratios=[2, 1], hspace=0.75)
    ax = fig.add_subplot(gs[0])
    ax2 = fig.add_subplot(gs[1])
    style_ax(ax)
    style_ax(ax2)
    titles(fig, ax, 'What the rider warning costs on the build',
           f'Same {len(base)} runs · {nwarn} runs warned · 3 rider dismounts on 20–25 % climbs')
    ax.grid(False)
    rowsets = [('no warning', base), ('warning, rider reacts', warn)]
    for yi, (name, rows) in enumerate(rowsets):
        y = 1 - yi
        left = 0
        for k in order:
            n = sum(klass(r) == k for r in rows)
            if n == 0:
                continue
            ax.barh(y, n, left=left, height=0.55, color=fc[k], edgecolor=BG, linewidth=0.8,
                    hatch='//' if k == 'RUNAWAY' else None, zorder=3)
            if n > 8:
                ax.text(left + n / 2, y, str(n), ha='center', va='center', fontsize=9,
                        color=BG, fontweight='bold', zorder=4)
            left += n
    ax.set_yticks([1, 0])
    ax.set_yticklabels([n for n, _ in rowsets], color=MUTED)
    ax.set_xlim(0, len(base))
    ax.tick_params(axis='y', length=0)
    ax.spines['left'].set_visible(False)
    labels(ax, 'Runs')
    plt.rcParams['hatch.linewidth'] = 0.8
    hs = [Patch(facecolor=fc[k], edgecolor=BG, hatch='//' if k == 'RUNAWAY' else None, label=k.lower())
          for k in order]
    lg = ax.legend(handles=hs, loc='upper center', bbox_to_anchor=(0.5, -0.38), ncol=len(order), frameon=False,
                   fontsize=8, handlelength=1.2, columnspacing=1.2)
    for t in lg.get_texts():
        t.set_color(TITLE)
    share = np.array([f(r['peak_amps']) / f(r['amps']) for r in noreact])
    warned = np.array([bool(r['t_warn_s']) for r in noreact])
    rng = np.random.default_rng(3)
    yj = rng.uniform(-0.25, 0.25, len(share))
    ax2.scatter(share[~warned], yj[~warned], s=SIZE, marker='o', c=PASS, edgecolors=BG, linewidths=0.8, zorder=3)
    ax2.scatter(share[warned], yj[warned], s=SIZE, marker='o', c=WARN, edgecolors=BG, linewidths=0.8, zorder=4)
    for x, lab in ((0.70, 'pulsed buzz'), (0.85, 'solid buzz')):
        ax2.axvline(x, color=TITLE, lw=1, ls='--', zorder=2)
        ax2.text(x + 0.008, 0.62, lab, color=TITLE, fontsize=8.5, ha='left', va='top')
    ax2.set_ylim(-0.7, 0.7)
    ax2.set_yticks([])
    ax2.spines['left'].set_visible(False)
    ax2.yaxis.grid(False)
    ax2.set_xlim(0, max(share.max(), 0.95) * 1.05)
    labels(ax2, 'Peak current, share of the motor limit')
    ax2.legend(handles=[Line2D([], [], ls='', marker='o', color=PASS, markersize=6, label='no warning'),
                        Line2D([], [], ls='', marker='o', color=WARN, markersize=6, label='warned')],
               loc='upper center', bbox_to_anchor=(0.5, -0.38), ncol=2, frameon=False, fontsize=8,
               labelcolor=TITLE)
    save(fig, out, 'r04_outcome_shift.png')
    print('fig4: share n=%d median %.2f max %.2f; warned %d, their share min %.2f max %.2f; >=0.70: %d, >=0.85: %d' %
          (len(share), np.median(share), share.max(), warned.sum(), share[warned].min(), share[warned].max(),
           (share >= 0.7).sum(), (share >= 0.85).sum()))
    for name, rows in rowsets:
        print('  ', name, {k: sum(klass(r) == k for r in rows) for k in order})


def fig5(out):
    d = city('tail_brake_descent')
    s = 90 - d['pos_x_m']
    t = d['sim_time_s']
    m = (s >= 0) & (s <= 30)
    s, t, v, tail, lvl = s[m], t[m], d['forward_speed_m_s'][m], d['tail_strike_n'][m], d['margin_level'][m]
    iw = int(np.argmax(lvl > 0))
    moving = np.where(v > 0.05)[0]
    ir = int(moving[-1]) + 1 if len(moving) and moving[-1] + 1 < len(v) else len(v) - 1
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 5.6), dpi=200, facecolor=BG, sharex=True,
                                   gridspec_kw={'hspace': 0.12, 'height_ratios': [1, 1]})
    for ax in (ax1, ax2):
        style_ax(ax)
        ax.xaxis.grid(False)
    titles(fig, ax1, 'At the braking limit the tail taps the road, and the board stops',
           '100 kg rider, SMALL motor (32 A), 15 % descent — an edge case')
    ax1.plot(s, v, color=PASS, lw=1.8, zorder=3)
    # Contact on/off only: the tail pad still touches the heightfield directly,
    # and that contact chatters (force spikes to 10 kN on a 100 kg rider), so
    # the force itself is a sim artefact. The stop distance is not.
    on = (tail > 50.0).astype(float)
    share = np.convolve(on, np.ones(100) / 100.0, mode='same')  # 0.2 s at 500 Hz
    ax2.fill_between(s, 0, share, color=TAIL, alpha=0.6, lw=0, zorder=2)
    ax2.plot(s, share, color=TAIL, lw=1.2, zorder=3)
    ax2.text(0.03, 0.75, 'force not shown: with a passive rider the tail taps the road\n'
             '(about 16 Hz); the peak force depends on the pad model, the stop does not', transform=ax2.transAxes,
             color=MUTED, fontsize=8, ha='left', va='center')
    for ax in (ax1, ax2):
        ax.axvline(s[iw], color=WARN, lw=0.8, ls=':', zorder=1)
        ax.axvline(s[ir], color=MUTED, lw=0.8, ls=':', zorder=1)
    ax1.scatter([s[iw]], [v[iw]], s=34, c=WARN, edgecolors=BG, linewidths=0.8, zorder=4)
    ax1.scatter([s[ir]], [v[ir]], s=34, c=MUTED, edgecolors=BG, linewidths=0.8, zorder=4)
    ax1.text(s[iw] - 0.4, v[iw] + 0.35, f'warning\n{s[iw]:.1f} m, {v[iw]:.1f} m/s', color=WARN, fontsize=8.5,
             ha='right', va='bottom')
    ax1.text(s[ir] + 0.4, max(v) * 0.25, f'at rest\n{s[ir]:.1f} m', color=MUTED, fontsize=8.5, ha='left', va='bottom')
    ax1.set_ylim(0, max(v) * 1.2)
    ax2.set_ylim(0, 1.05)
    ax2.set_yticks([0, 0.5, 1])
    ax2.set_yticklabels(['0', '50 %', '100 %'])
    ax2.set_xlim(0, 30)
    labels(ax1, y='Speed, m/s')
    labels(ax2, 'Distance along the street, m', 'Tail pad on the road,\nshare of time')
    save(fig, out, 'r05_tail_brake_stop.png')
    print('fig5: warning s=%.2f t=%.2f v=%.2f; rest s=%.2f t=%.2f; peak tail %.0f N' %
          (s[iw], t[iw], v[iw], s[ir], t[ir], tail.max()))


def fig6(rows, out):
    fig, ax = new_fig('The old road model shook the wheel; the fixed one does not',
                      '0.145 m wheel, 80 kg frame, MuJoCo 3.10 · fix: a smooth plate under the tyre')
    ax.xaxis.grid(True, color=GRID, linewidth=0.6)
    ser = {}
    for g, col, name in (('plane', PASS, 'smooth plate (fix)'), ('hfield', NOSE, 'heightfield')):
        s = sorted([r for r in rows if r['ground'] == g], key=lambda r: f(r['v_m_s']))
        ser[g] = s
        x, y = [f(r['v_m_s']) for r in s], [f(r['fz_sd']) for r in s]
        ax.plot(x, y, color=col, lw=1.8, marker='o', markersize=4.5, markeredgecolor=BG,
                markeredgewidth=0.8, zorder=3)
        ax.text(x[-1] + 0.12, y[-1], name, color=col, fontsize=9, ha='left', va='center')
    allx = [f(r['v_m_s']) for r in rows]
    ax.set_xlim(min(allx) - 0.1, max(allx) + 1.6)
    ymax = max(f(r['fz_sd']) for r in rows)
    ax.set_ylim(-0.03 * ymax, ymax * 1.12)
    air = [r for r in ser['hfield'] if f(r['fz_min']) <= 0.01]
    if air:
        a = air[0]
        ax.annotate('wheel leaves the ground', (f(a['v_m_s']), f(a['fz_sd'])),
                    xytext=(f(a['v_m_s']) - 0.4, f(a['fz_sd']) + ymax * 0.3), color=TITLE, fontsize=8.5,
                    ha='right', va='bottom', arrowprops=dict(arrowstyle='-', color=MUTED, lw=0.8))
    labels(ax, 'Wheel speed, m/s', 'Vertical specific force, sd (m/s²)')
    save(fig, out, 'r06_contact_chatter.png')
    print('fig6: first air speed', air[0]['v_m_s'] if air else None)


def main():
    out = sys.argv[1]
    os.makedirs(out, exist_ok=True)
    base = load('x7_tail_brake.csv')
    warn = load('x7_warn_rider_reacts.csv')
    noreact = load('x7_warn_no_reaction.csv')
    fig1(base, out)
    fig2(base, out)
    fig3(out)
    fig4(base, warn, noreact, out)
    fig5(out)
    fig6(load('contact_chatter.csv'), out)


if __name__ == '__main__':
    main()
