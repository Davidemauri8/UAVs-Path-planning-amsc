"""
plot_results.py - benchmark plots for the parallel DRSTASA UAV path planner.

Produces (into ../output/):
  1. plot_strong_scaling.png     speedup + parallel efficiency vs #threads, with ideal lines
  2. plot_obstacles.png          mean SA vs DRSTASA fitness vs obstacle count
  3. plot_cluster_granularity.png runtime vs K  (sensitivity study -- NOT a scaling curve)
  4. plot_weak_scaling.png       OPTIONAL: appears only if bench_weak.csv exists

Expected CSVs in ../output/:
  bench_parallel.csv    : threads, run, wall_time
  bench_obstacles.csv   : n_obstacles, run, sa_fit, drstasa_fit
  bench_scaling.csv     : k, run, wall_time
  bench_weak.csv  (opt) : threads, run, wall_time      # problem size scaled WITH the thread count
"""

import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_DIR    = os.path.join(SCRIPT_DIR, '..', 'output')

# A run whose returned cost exceeds this is treated as INFEASIBLE.
# The planner returns a 1e12 sentinel on constraint violation, while feasible
# mission costs are ~1e7, so any threshold in between cleanly separates the two.
FEASIBLE_THRESHOLD = 1e11

plt.rcParams.update({
    'font.size': 11,
    'axes.titlesize': 12,
    'axes.labelsize': 11,
    'legend.fontsize': 10,
    'axes.spines.top': False,
    'axes.spines.right': False,
})


# ----------------------------------------------------------------------------- helpers
def _read(name):
    path = os.path.join(OUT_DIR, name)
    return pd.read_csv(path) if os.path.exists(path) else None


def _save(fig, name):
    path = os.path.join(OUT_DIR, name)
    fig.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved {path}')


# ----------------------------------------------------------- 1. STRONG SCALING
# Fixed problem, vary threads. Ideal speedup is linear (S = p); ideal efficiency is 100%.
def plot_strong_scaling():
    df = _read('bench_parallel.csv')
    if df is None:
        print('skip strong scaling: bench_parallel.csv not found')
        return

    m = df.groupby('threads')['wall_time'].mean()
    p = m.index.to_numpy(dtype=float)
    t = m.to_numpy()
    n_runs = df['run'].nunique()

    speedup    = t[0] / t
    efficiency = speedup / p                      # E(p) = S(p) / p

    fig, (axS, axE) = plt.subplots(1, 2, figsize=(11, 4.5))

    # --- speedup ---
    axS.plot(p, p, '--', color='gray', linewidth=1.5, label='ideal (linear)')
    axS.plot(p, speedup, marker='o', markersize=7, linewidth=2,
             color='steelblue', label='measured')
    for pi, si in zip(p, speedup):
        axS.annotate(f'{si:.2f}x', xy=(pi, si), xytext=(6, -12),
                     textcoords='offset points', fontsize=9, color='steelblue')
    axS.set_xlabel('Number of threads')
    axS.set_ylabel('Speedup ')
    axS.set_title('Strong scaling - speedup')
    axS.set_xticks(p)
    axS.legend()
    axS.grid(True, alpha=0.3, linestyle='--')

    # --- efficiency ---
    axE.axhline(1.0, linestyle='--', color='gray', linewidth=1.5, label='ideal (100%)')
    axE.plot(p, efficiency, marker='o', markersize=7, linewidth=2,
             color='indianred', label='measured')
    for pi, ei in zip(p, efficiency):
        axE.annotate(f'{ei*100:.0f}%', xy=(pi, ei), xytext=(6, -12),
                     textcoords='offset points', fontsize=9, color='indianred')
    axE.set_xlabel('Number of threads')
    axE.set_ylabel('Parallel efficiency')
    axE.set_title('Strong scaling - efficiency')
    axE.set_xticks(p)
    axE.set_ylim(0, 1.1)
    axE.legend()
    axE.grid(True, alpha=0.3, linestyle='--')

    fig.suptitle(f'Strong scaling - fixed problem',
                 fontsize=13)
    fig.tight_layout()
    _save(fig, 'plot_strong_scaling.png')


# ------------------------------------------------- 2. FITNESS VS OBSTACLES
def plot_obstacles():
    df = _read('bench_obstacles.csv')
    if df is None:
        print('skip obstacles: bench_obstacles.csv not found')
        return

    x = np.sort(df['n_obstacles'].unique())
    n_runs = df['run'].nunique()

    def stats(col):
        g = df.groupby('n_obstacles')[col]
        return g.mean().reindex(x).to_numpy(), g.std().reindex(x).to_numpy()

    m_sa, s_sa = stats('sa_fit')
    m_dr, s_dr = stats('drstasa_fit')

    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(x, m_sa, marker='s', markersize=7, linewidth=2, color='tomato', label='SA')
    ax.fill_between(x, m_sa - s_sa, m_sa + s_sa, color='tomato', alpha=0.18)
    ax.plot(x, m_dr, marker='o', markersize=7, linewidth=2, color='steelblue', label='DRSTASA')
    ax.fill_between(x, m_dr - s_dr, m_dr + s_dr, color='steelblue', alpha=0.18)
    ax.set_xlabel('Number of obstacles')
    ax.set_ylabel('Mean fitness')
    ax.set_title(f'Path cost vs obstacle density')
    ax.set_xticks(x)
    ax.legend()
    ax.grid(True, alpha=0.3, linestyle='--')
    fig.tight_layout()
    _save(fig, 'plot_obstacles.png')


# ---------------------------------------------- 3. CLUSTER GRANULARITY (sensitivity)
# Varying K on a FIXED dataset changes the WORKLOAD (segments per drone ~ N - K),
# so this is NOT a scaling curve. No speedup "x" labels: they would be meaningless here.
def plot_cluster_granularity():
    df = _read('bench_scaling.csv')
    if df is None:
        print('skip cluster granularity: bench_scaling.csv not found')
        return

    g = df.groupby('k')['wall_time']
    k = g.mean().index.to_numpy()
    t = g.mean().to_numpy()
    s = g.std().to_numpy()
    n_runs = df['run'].nunique()

    fig, ax = plt.subplots(figsize=(7, 4.5))
    ax.plot(k, t, marker='o', markersize=7, linewidth=2, color='mediumseagreen')
    ax.fill_between(k, t - s, t + s, color='mediumseagreen', alpha=0.18)
    ax.set_xlabel('Number of clusters K  (fixed 21-waypoint dataset)')
    ax.set_ylabel('Execution time (s)')
    ax.set_title(f'Runtime vs clustering granularity  (4 threads, {n_runs} runs/config)')
    ax.set_xticks(k)
    ax.grid(True, alpha=0.3, linestyle='--')
    fig.tight_layout()
    _save(fig, 'plot_cluster_granularity.png')


# ----------------------------------------------------------- 4. WEAK SCALING (optional)
# Problem size grows proportionally to the thread count, so work-per-thread is constant.
# Ideal runtime is FLAT; ideal weak-scaling efficiency is 100%.
def plot_weak_scaling():
    df = _read('bench_weak.csv')
    if df is None:
        print('skip weak scaling: bench_weak.csv not found '
              '(generate it -- e.g. p=1/K=1, p=2/K=2, p=4/K=4 -- to enable this plot)')
        return

    g = df.groupby('threads')['wall_time']
    p = g.mean().index.to_numpy(dtype=float)
    t = g.mean().to_numpy()
    s = g.std().to_numpy()
    n_runs = df['run'].nunique()

    weak_eff = t[0] / t                           # ideal = 1 (flat)

    fig, (axT, axE) = plt.subplots(1, 2, figsize=(11, 4.5))

    axT.axhline(t[0], linestyle='--', color='gray', linewidth=1.5, label='ideal (constant)')
    axT.plot(p, t, marker='o', markersize=7, linewidth=2, color='darkorange', label='measured')
    axT.fill_between(p, t - s, t + s, color='darkorange', alpha=0.18)
    axT.set_xlabel('Number of threads  (work per thread fixed)')
    axT.set_ylabel('Execution time (s)')
    axT.set_title('Weak scaling - runtime')
    axT.set_xticks(p)
    axT.legend()
    axT.grid(True, alpha=0.3, linestyle='--')

    axE.axhline(1.0, linestyle='--', color='gray', linewidth=1.5, label='ideal (100%)')
    axE.plot(p, weak_eff, marker='o', markersize=7, linewidth=2,
             color='darkorange', label='measured')
    for pi, ei in zip(p, weak_eff):
        axE.annotate(f'{ei*100:.0f}%', xy=(pi, ei), xytext=(6, -12),
                     textcoords='offset points', fontsize=9, color='darkorange')
    axE.set_xlabel('Number of threads  (work per thread fixed)')
    axE.set_ylabel('Weak-scaling efficiency  T(1) / T(p)')
    axE.set_title('Weak scaling - efficiency')
    axE.set_xticks(p)
    axE.set_ylim(0, 1.1)
    axE.legend()
    axE.grid(True, alpha=0.3, linestyle='--')

    fig.suptitle(f'Weak scaling - constant work per thread  ({n_runs} runs/config)', fontsize=13)
    fig.tight_layout()
    _save(fig, 'plot_weak_scaling.png')


if __name__ == '__main__':
    plot_strong_scaling()
    plot_obstacles()
    plot_cluster_granularity()
    plot_weak_scaling()