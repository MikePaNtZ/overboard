"""Make the five static Monte Carlo charts for the dark carve-lab page.

Usage: plot_mc.py OUT_DIR
"""
import csv
import os
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from mc_summary import envelope  # noqa: E402

MC = os.path.join(HERE, 'mc')
BG, TITLE, MUTED, GRID = '#111D27', '#EAF1F1', '#8FA3AA', '#22333F'
PASS, NOSE, TAIL = '#33C6AC', '#E5604D', '#F2A24A'
BASE, CUR = '#9C8CF0', '#F2A24A'
MARK = {'PASS': 'o', 'nose strike': 'v', 'tail strike': '^'}
COL = {'PASS': PASS, 'nose strike': NOSE, 'tail strike': TAIL}
SIZE = 36


def load(name):
    return list(csv.DictReader(open(os.path.join(MC, name))))


def f(x):
    return float(x)


def klass(r):
    if r['status'] == 'PASS':
        return 'PASS'
    return 'nose strike' if 'nose' in r['cause'] else 'tail strike'


def new_fig(title, subtitle):
    fig, ax = plt.subplots(figsize=(8, 5), dpi=200, facecolor=BG)
    ax.set_facecolor(BG)
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)
    for s in ('left', 'bottom'):
        ax.spines[s].set_color(GRID)
    ax.tick_params(colors=MUTED, labelsize=9)
    ax.set_axisbelow(True)
    ax.yaxis.grid(True, color=GRID, linewidth=0.6)
    fig.text(0.0, 1.0, title, color=TITLE, fontsize=13, ha='left', va='bottom',
             transform=ax.transAxes + matplotlib.transforms.ScaledTranslation(0, 0.3, fig.dpi_scale_trans))
    ax.text(0.0, 1.03, subtitle, color=MUTED, fontsize=9, ha='left', va='bottom', transform=ax.transAxes)
    return fig, ax


def labels(ax, x, y):
    ax.set_xlabel(x, color=MUTED, fontsize=10)
    ax.set_ylabel(y, color=MUTED, fontsize=10)


def legend(ax, handles, loc):
    if loc == 'below':
        lg = ax.legend(handles=handles, loc='upper center', bbox_to_anchor=(0.5, -0.14), ncol=len(handles),
                       frameon=False, fontsize=9)
    else:
        lg = ax.legend(handles=handles, loc=loc, frameon=False, fontsize=9)
    for t in lg.get_texts():
        t.set_color(TITLE)


def save(fig, out, name):
    fig.savefig(os.path.join(out, name), dpi=200, facecolor=BG, bbox_inches='tight', pad_inches=0.3)
    plt.close(fig)


def class_handles():
    return [Line2D([], [], ls='', marker=MARK[k], color=COL[k], markersize=6,
                   markeredgecolor=BG, markeredgewidth=0.8, label=k if k != 'PASS' else 'pass')
            for k in MARK]


def scatter_classes(ax, rows, xf, yf):
    for k in MARK:
        s = [r for r in rows if klass(r) == k]
        ax.scatter([xf(r) for r in s], [yf(r) for r in s], s=SIZE, marker=MARK[k], c=COL[k],
                   edgecolors=BG, linewidths=0.8, zorder=3)


def margin_at(m, g):
    a = np.arctan(abs(g) / 100)
    L = (0.75 * m + 0.135) / (13 + m)
    return 18.6 - np.degrees(a + np.arcsin(0.1454 * np.sin(a) / L))


def limit_grade(m, target=3.75):
    lo, hi = 0.0, 100.0
    for _ in range(60):
        mid = (lo + hi) / 2
        if margin_at(m, mid) > target:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2


def chart1(rows, out):
    fig, ax = new_fig('Where the speed hold fails',
                      '200 Monte Carlo runs · 55–110 kg · 2–6 m/s · 30–60 A · Kt ±15 %')
    ax.xaxis.grid(True, color=GRID, linewidth=0.6)
    ms = np.linspace(55, 110, 100)
    gs = np.array([limit_grade(m) for m in ms])
    ax.plot(gs, ms, ls='--', color=MUTED, lw=1, zorder=2)
    ax.plot(-gs, ms, ls='--', color=MUTED, lw=1, zorder=2)
    scatter_classes(ax, rows, lambda r: f(r['grade_pct']), lambda r: f(r['rider_kg']))
    ax.text(-gs[-1] - 0.4, 111.2, 'deck-strike limit (3.75° margin)', color=MUTED, fontsize=8,
            ha='left', va='bottom')
    labels(ax, 'Grade, % (climb +)', 'Rider mass, kg')
    legend(ax, class_handles(), 'below')
    ax.set_ylim(52, 114)
    save(fig, out, 'm01_failure_map.png')


def chart2(rows, out):
    fig, ax = new_fig('Static envelope explains the falls',
                      'Rule: fall if strike margin < 3.75° or current ratio < 1.02 · 194 of 200 runs agree')
    ax.xaxis.grid(True, color=GRID, linewidth=0.6)
    env = {r['run']: envelope(r) for r in rows}
    xs = [env[r['run']][0] for r in rows]
    xmin, xmax = min(xs) - 1, max(xs) + 1
    ax.set_yscale('log')
    ax.set_ylim(0.4, 12)
    ax.set_xlim(xmin, xmax)
    ax.axvspan(xmin, 3.75, color=NOSE, alpha=0.08, lw=0, zorder=1)
    ax.axhspan(0.4, 1.02, color=NOSE, alpha=0.08, lw=0, zorder=1)
    ax.axvline(3.75, ls='--', color=MUTED, lw=1, zorder=2)
    scatter_classes(ax, rows, lambda r: env[r['run']][0], lambda r: min(max(env[r["run"]][1], 0.42), 11.5))
    ticks = [0.5, 1, 1.5, 2, 3, 5, 10]
    ax.set_yticks(ticks)
    ax.set_yticklabels([str(t) for t in ticks])
    ax.minorticks_off()
    miss = [r for r in rows if r['run'] in ('r120', 'r134', 'r170', 'r175')]
    if miss:
        px = np.mean([env[r['run']][0] for r in miss])
        py = np.mean([env[r['run']][1] for r in miss])
        ax.annotate('heavy rider, low Kt', (px, py), xytext=(px, py * 1.9), color=TITLE, fontsize=8,
                    ha='center', arrowprops=dict(arrowstyle='-', color=MUTED, lw=0.8))
    labels(ax, 'Static strike margin, deg', 'Current ratio (limit / steady-state need)')
    legend(ax, class_handles(), 'below')
    save(fig, out, 'm02_envelope.png')
    agree = sum(((env[r['run']][0] < 3.75 or env[r['run']][1] < 1.02) == (r['status'] != 'PASS')) for r in rows)
    print('rule agrees on', agree, 'of', len(rows))
    print('misses:', [(r['run'], r['status']) for r in rows
                      if (env[r['run']][0] < 3.75 or env[r['run']][1] < 1.02) != (r['status'] != 'PASS')])


def ecdf(v):
    v = np.sort(np.array(v))
    return v, np.arange(1, len(v) + 1) / len(v)


def chart3(base, cur, out):
    def ov(rows):
        v = np.array([f(r['overshoot_pct']) for r in rows if r['status'] == 'PASS'])
        return v[np.isfinite(v)]
    b, c = ov(base), ov(cur)
    sub = (f'median {np.median(b):.1f} → {np.median(c):.1f} %, '
           f'p90 {np.percentile(b, 90):.1f} → {np.percentile(c, 90):.1f} %')
    print('chart3 subtitle:', sub, '| n pass base/cur', len(b), len(c))
    fig, ax = new_fig('Grade feedforward cuts speed overshoot', sub)
    xmax = max(b.max(), c.max())
    ax.set_xlim(0, xmax * 1.28)
    for v, col, name in ((b, BASE, 'old law'), (c, CUR, 'grade feedforward')):
        x, y = ecdf(v)
        x = np.append(x, xmax * 1.02)
        y = np.append(y, 1.0)
        ax.step(x, y, where='post', color=col, lw=2, zorder=3)
        md = np.median(v)
        ax.scatter([md], [0.5], s=SIZE, color=col, edgecolors=BG, linewidths=0.8, zorder=4)
        ax.text(xmax * 1.04, 1.0 if name == 'old law' else 0.93, name, color=col, fontsize=9,
                ha='left', va='center')
    labels(ax, 'Speed overshoot, % of target', 'Share of runs')
    ax.set_ylim(0, 1.04)
    save(fig, out, 'm03_overshoot.png')


def chart4(rows, out):
    fig, ax = new_fig('Energy per km against grade', 'Constant-grade section · 20S2P pack model (battery.py)')
    ax.xaxis.grid(True, color=GRID, linewidth=0.6)
    p = [r for r in rows if r['status'] == 'PASS' and np.isfinite(f(r['wh_per_km']))]
    cmap = matplotlib.colors.LinearSegmentedColormap.from_list('mint', ['#1E5F55', '#7FF0DA'])
    ax.axhline(0, color=MUTED, lw=1, zorder=2)
    sc = ax.scatter([f(r['grade_pct']) for r in p], [f(r['wh_per_km']) for r in p],
                    c=[f(r['v_target']) for r in p], cmap=cmap, vmin=2, vmax=6, s=SIZE,
                    edgecolors=BG, linewidths=0.8, zorder=3)
    cb = fig.colorbar(sc, ax=ax, pad=0.02, fraction=0.04)
    cb.set_label('Target speed, m/s', color=MUTED, fontsize=10)
    cb.ax.tick_params(colors=MUTED, labelsize=9)
    cb.outline.set_edgecolor(GRID)
    labels(ax, 'Grade, % (climb +)', 'Battery energy, Wh/km (negative = regeneration)')
    save(fig, out, 'm04_energy.png')


def chart5(rows, out):
    fig, ax = new_fig('Heightfield contact chatters; the plane does not',
                      '0.145 m sphere wheel, 80 kg frame, MuJoCo 3.10')
    ax.xaxis.grid(True, color=GRID, linewidth=0.6)
    series = {}
    for g, col, name in (('plane', PASS, 'flat plane'), ('hfield', NOSE, 'heightfield')):
        s = [r for r in rows if r['ground'].startswith(g[:5]) or r['ground'] == g]
        s.sort(key=lambda r: f(r['v_m_s']))
        series[g] = s
        x = [f(r['v_m_s']) for r in s]
        y = [f(r['fz_sd']) for r in s]
        ax.plot(x, y, color=col, lw=2, marker='o', markersize=5, markeredgecolor=BG,
                markeredgewidth=0.8, zorder=3)
        ax.text(x[-1] + 0.08, y[-1], name, color=col, fontsize=9, ha='left', va='center')
    allx = [f(r['v_m_s']) for r in rows]
    ax.set_xlim(min(allx) - 0.1, max(allx) + 1.4)
    air = [r for r in series.get('hfield', []) if f(r['fz_min']) <= 0.01]
    print('hfield air points:', [(r['v_m_s'], r['fz_sd']) for r in air], 'ground names:', {r['ground'] for r in rows})
    if air:
        a = air[0]
        ys = [f(r['fz_sd']) for r in air]
        ax.annotate('wheel leaves the ground', (f(a['v_m_s']), f(a['fz_sd'])),
                    xytext=(f(a['v_m_s']) + 0.4, f(a['fz_sd']) * 0.45), color=TITLE, fontsize=8,
                    ha='left', va='top', arrowprops=dict(arrowstyle='-', color=MUTED, lw=0.8))
    labels(ax, 'Wheel speed, m/s', 'Vertical specific force, sd (m/s²)')
    ax.set_ylim(bottom=-0.02 * max(f(r['fz_sd']) for r in rows))
    legend(ax, [Line2D([], [], color=PASS, lw=2, marker='o', label='flat plane'),
                Line2D([], [], color=NOSE, lw=2, marker='o', label='heightfield')], 'upper left')
    save(fig, out, 'm05_contact_chatter.png')


def main():
    out = sys.argv[1]
    os.makedirs(out, exist_ok=True)
    cur, base = load('grade_ff.csv'), load('baseline.csv')
    chart1(cur, out)
    chart2(cur, out)
    chart3(base, cur, out)
    chart4(cur, out)
    chart5(load('contact_chatter.csv'), out)


if __name__ == '__main__':
    main()
