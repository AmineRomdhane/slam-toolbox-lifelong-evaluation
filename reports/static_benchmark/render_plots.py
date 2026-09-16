"""Render presentation-only figures from frozen snapshots; never run experiments.

Usage: python3 reports/static_benchmark/render_plots.py (NumPy + Matplotlib).
Every panel shows ten run values, linear-quantile IQR and median, without density
estimation. Repeatability uses relative deviations and sample CV, not new trials.
"""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

OUT = Path(__file__).resolve().parent
COHORTS = ['r1', 'r2', 'r3']
COLORS = ['#0072B2', '#D55E00', '#009E73']
plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 10,
                     'axes.spines.top': False, 'axes.spines.right': False,
                     'svg.fonttype': 'none', 'svg.hashsalt': 'static-benchmark'})


def read_values(kind, key, scale=1):
    values = []
    for route in COHORTS:
        report = json.loads((OUT / 'data' / f'{route}_{kind}.json').read_text())
        raw = report['metrics'][key]['individual_values']
        array = np.asarray([x['value'] if isinstance(x, dict) else x for x in raw], dtype=float) * scale
        assert len(array) == 10 and np.isfinite(array).all()
        values.append(array)
    return values


def plot(name, items, title, relative=False):
    fig, axes = plt.subplots(2, 2, figsize=(10, 7))
    fig.subplots_adjust(top=.88, bottom=.12, wspace=.36, hspace=.57)
    for ax, (kind, key, label, unit, scale) in zip(axes.flat, items):
        values = read_values(kind, key, scale)
        labels = ['R1', 'R2', 'R3 valid']
        if relative:
            assert all(np.all(v > 0) for v in values)
            labels = [f'{label}\nCV {100*np.std(v, ddof=1)/np.mean(v):.3f}%'
                      for label, v in zip(labels, values)]
            values = [100 * (v / np.mean(v) - 1) for v in values]
            unit = 'Deviation from route mean (%)'
            ax.axhline(0, color='#858b91', linewidth=.8, linestyle='--')
        for index, (v, color) in enumerate(zip(values, COLORS), 1):
            q1, median, q3 = np.quantile(v, [.25, .5, .75], method='linear')
            # Separate the summary horizontally from points so no run is hidden.
            ax.plot([index-.23, index-.23], [q1, q3], color=color,
                    linewidth=6, solid_capstyle='butt', zorder=3)
            ax.plot([index-.31, index-.15], [median, median], color='#24292f',
                    linewidth=2, zorder=4)
            ax.scatter(index + np.linspace(-.07, .27, 10), v, s=27,
                       color=color, edgecolors='white', linewidth=.45, zorder=3)
        lo = min(v.min() for v in values)
        hi = max(v.max() for v in values)
        pad = max((hi-lo)*.18, abs(hi)*.008, 1e-6)
        ax.set_ylim(lo-pad if relative else max(0, lo-pad), hi+pad)
        ax.set_xlim(.5, 3.5)
        ax.set_xticks([1, 2, 3], labels)
        ax.set_title(label, loc='left', fontweight='bold')
        ax.set_ylabel(unit)
        ax.grid(axis='y', alpha=.18)
        ax.set_axisbelow(True)
        ax.ticklabel_format(axis='y', style='plain', useOffset=False)
    fig.suptitle(title, fontweight='bold', fontsize=15)
    fig.text(.5, .026, '10 points per route · thick segment: IQR · black tick: median · independently zoomed panels',
             ha='center', fontsize=9, color='#57606a')
    for ext in ['png', 'svg']:
        path = OUT / 'assets' / f'{name}.{ext}'
        fig.savefig(path, dpi=180, facecolor='white', metadata={'Date': None} if ext == 'svg' else None)
        if ext == 'svg':
            path.write_text('\n'.join(line.rstrip() for line in path.read_text().splitlines())+'\n')
    plt.close(fig)


if __name__ == '__main__':
    pose = [('pose_system', 'ate_translation_rmse', 'Translational ATE RMSE ↓', 'mm', 1000),
            ('pose_system', 'ape_yaw_rmse', 'Yaw APE RMSE ↓', 'mrad', 1000),
            ('pose_system', 'rpe_1m_translation_rmse', '1 m translational RPE RMSE ↓', 'mm', 1000),
            ('pose_system', 'rpe_1m_rotation_rmse', '1 m rotational RPE RMSE ↓', 'mrad', 1000)]
    system = [('pose_system', 'active_cpu_mean', 'Active CPU mean', '% of one logical CPU', 1),
              ('pose_system', 'active_cpu_peak', 'Active CPU peak', '% of one logical CPU', 1),
              ('pose_system', 'active_rss_mean', 'Active RSS mean', 'MiB', 1),
              ('pose_system', 'active_rss_peak', 'Active RSS peak', 'MiB', 1)]
    maps = [('map', 'occupied_precision', 'Occupied precision ↑', 'fraction', 1),
            ('map', 'occupied_recall', 'Occupied recall ↑', 'fraction', 1),
            ('map', 'occupied_f1', 'Occupied F1 ↑', 'fraction', 1),
            ('map', 'boundary_distance_mean_m', 'Symmetric boundary mean ↓', 'mm', 1000)]
    plot('pose_distributions', pose, 'Trajectory accuracy · ten valid trials per route')
    plot('system_distributions', system, 'Active SLAM-process resources')
    plot('map_distributions', maps, 'Map quality · route-specific observed domains')
    plot('repeatability', [pose[0], pose[2], system[0], system[2]],
         'Fixed-input repeatability · relative run-level spread', relative=True)
