#!/usr/bin/env python3
"""Analyze multi-robot exploration runs.

Usage:
    python3 analyze_metrics.py ~/run_random.csv ~/run_nearest.csv \\
                                ~/run_furthest.csv ~/run_spread.csv \\
                                ~/run_ucb1.csv

Each CSV must come from multirobot_bringup's metrics_logger and have:
    sim_time, cells_known, area_m2, robotN_dist_m, ...

Outputs (written to current directory):
    coverage_vs_time.png   ← headline plot, all strategies overlaid
    time_to_milestones.png ← bar chart of time-to-X% coverage
    rate_summary.png       ← bar chart of avg mapping rate (m²/s)
    summary.txt            ← table of final stats per run
"""

import os
import sys

import matplotlib.pyplot as plt
import pandas as pd

COLORS = {
    'random':   '#888888',
    'nearest':  '#1f77b4',
    'furthest': '#ff7f0e',
    'spread':   '#2ca02c',
    'ucb1':     '#d62728',
}


def load_run(path):
    """Read CSV; derive a label from the filename."""
    name = os.path.basename(path).replace('.csv', '')
    label = name[4:] if name.startswith('run_') else name
    df = pd.read_csv(os.path.expanduser(path))
    return label, df


def time_to_fraction(df, target_area):
    """Return first sim_time at which area_m2 >= target_area, else None."""
    hits = df[df['area_m2'] >= target_area]
    return float(hits['sim_time'].iloc[0]) if len(hits) else None


def color_for(label):
    return COLORS.get(label, '#444444')


def total_distance(df):
    """Sum the final values of all robotN_dist_m columns."""
    cols = [c for c in df.columns if c.startswith('robot') and c.endswith('_dist_m')]
    return float(sum(df[c].iloc[-1] for c in cols))


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    runs = [load_run(p) for p in sys.argv[1:]]

    # --- Plot 1: coverage over time ---
    fig, ax = plt.subplots(figsize=(10, 6))
    for label, df in runs:
        ax.plot(df['sim_time'], df['area_m2'],
                label=label, linewidth=2, color=color_for(label))
    ax.set_xlabel('Simulation time (s)')
    ax.set_ylabel('Mapped area (m²)')
    ax.set_title('Multi-robot exploration: coverage over time')
    ax.legend()
    ax.grid(alpha=0.3)
    plt.tight_layout()
    plt.savefig('coverage_vs_time.png', dpi=150)
    plt.close()
    print('saved: coverage_vs_time.png')

    # Use the global peak as the reference for "X% coverage"
    overall_peak = max(df['area_m2'].max() for _, df in runs)

    # --- Summary table ---
    summary_rows = []
    for label, df in runs:
        final = float(df['area_m2'].iloc[-1])
        runtime = float(df['sim_time'].iloc[-1])
        rate = final / runtime if runtime > 0 else 0.0
        t25 = time_to_fraction(df, 0.25 * overall_peak)
        t50 = time_to_fraction(df, 0.50 * overall_peak)
        t75 = time_to_fraction(df, 0.75 * overall_peak)
        total_dist = total_distance(df)
        summary_rows.append({
            'strategy': label,
            'runtime_s': runtime,
            'final_area_m2': final,
            'mapping_rate_m2_per_s': rate,
            'time_to_25pct_s': t25,
            'time_to_50pct_s': t50,
            'time_to_75pct_s': t75,
            'total_robot_distance_m': total_dist,
        })

    summary = pd.DataFrame(summary_rows)
    print('\n=== summary ===')
    print(summary.to_string(index=False, float_format=lambda v: f'{v:.2f}'))
    summary.to_csv('summary.csv', index=False)
    with open('summary.txt', 'w') as f:
        f.write(f'overall_peak_area_m2: {overall_peak:.2f}\n\n')
        f.write(summary.to_string(index=False, float_format=lambda v: f'{v:.2f}'))
        f.write('\n')
    print('saved: summary.csv, summary.txt')

    # --- Plot 2: time to X% milestones ---
    fig, ax = plt.subplots(figsize=(10, 6))
    labels = [r['strategy'] for r in summary_rows]
    width = 0.27
    x = range(len(labels))
    t25 = [r['time_to_25pct_s'] or 0 for r in summary_rows]
    t50 = [r['time_to_50pct_s'] or 0 for r in summary_rows]
    t75 = [r['time_to_75pct_s'] or 0 for r in summary_rows]
    ax.bar([i - width for i in x], t25, width, label='25%', color='#a8d5a3')
    ax.bar([i for i in x], t50, width, label='50%', color='#5cb85c')
    ax.bar([i + width for i in x], t75, width, label='75%', color='#2c7d2c')
    ax.set_xticks(list(x))
    ax.set_xticklabels(labels)
    ax.set_ylabel('Sim time (s)')
    ax.set_title(f'Time to reach % of overall peak ({overall_peak:.0f} m²) — lower is better')
    ax.legend()
    ax.grid(alpha=0.3, axis='y')
    plt.tight_layout()
    plt.savefig('time_to_milestones.png', dpi=150)
    plt.close()
    print('saved: time_to_milestones.png')

    # --- Plot 3: avg mapping rate ---
    fig, ax = plt.subplots(figsize=(8, 5))
    rates = [r['mapping_rate_m2_per_s'] for r in summary_rows]
    ax.bar(labels, rates, color=[color_for(l) for l in labels])
    ax.set_ylabel('Avg mapping rate (m²/s)')
    ax.set_title('Mapping efficiency — higher is better')
    ax.grid(alpha=0.3, axis='y')
    plt.tight_layout()
    plt.savefig('rate_summary.png', dpi=150)
    plt.close()
    print('saved: rate_summary.png')

    # --- Plot 4: per-robot trajectories per strategy ---
    # One PNG per strategy showing all robots' paths in different colors.
    for label, df in runs:
        # Find robotN_x / robotN_y column pairs.
        x_cols = sorted(c for c in df.columns if c.endswith('_x'))
        if not x_cols:
            continue  # CSV from older logger version, no positions
        robot_palette = plt.get_cmap('tab10')
        fig, ax = plt.subplots(figsize=(8, 8))
        for i, xc in enumerate(x_cols):
            yc = xc[:-2] + '_y'
            robot = xc[:-2]
            x = pd.to_numeric(df[xc], errors='coerce')
            y = pd.to_numeric(df[yc], errors='coerce')
            valid = x.notna() & y.notna()
            if not valid.any():
                continue
            color = robot_palette(i % 10)
            ax.plot(x[valid], y[valid], color=color, label=robot,
                    linewidth=1.8, alpha=0.85)
            # Start (circle) and end (square) markers
            ax.scatter(x[valid].iloc[0], y[valid].iloc[0],
                       marker='o', s=120, color=color,
                       edgecolor='white', linewidth=2, zorder=5)
            ax.scatter(x[valid].iloc[-1], y[valid].iloc[-1],
                       marker='s', s=120, color=color,
                       edgecolor='white', linewidth=2, zorder=5)
        ax.set_xlabel('x (m, map_merged frame)')
        ax.set_ylabel('y (m, map_merged frame)')
        ax.set_title(f'Robot trajectories — {label}\n'
                     f'(○ = start, ■ = end)')
        ax.set_aspect('equal')
        ax.legend(loc='best')
        ax.grid(alpha=0.3)
        plt.tight_layout()
        out = f'trajectory_{label}.png'
        plt.savefig(out, dpi=150)
        plt.close()
        print(f'saved: {out}')


if __name__ == '__main__':
    main()
