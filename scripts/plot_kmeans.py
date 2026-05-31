"""
K-scaling plot
--------------
Reads output/bench_kmeans.csv and produces two panels:

  Left  — wall time vs K (mean ± 1 std across runs)
  Right — speedup (T_K1 / T_K) vs K, with ideal linear line and
          a horizontal ceiling at NUM_THREADS

Usage:
    python scripts/plot_kmeans.py
"""

import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
OUT_DIR     = os.path.join(SCRIPT_DIR, '..', 'output')
CSV_PATH    = os.path.join(OUT_DIR, 'bench_kmeans.csv')
OUT_PNG     = os.path.join(OUT_DIR, 'plot_kmeans.png')

plt.rcParams.update({
    'font.size': 11,
    'axes.titlesize': 12,
    'axes.labelsize': 11,
    'legend.fontsize': 10,
    'axes.spines.top': False,
    'axes.spines.right': False,
})

df = pd.read_csv(CSV_PATH)

# Infer number of fixed threads from the CSV (constant column)
num_threads = int(df['threads'].iloc[0])

g       = df.groupby('k')['wall_time']
mean_t  = g.mean()
std_t   = g.std().fillna(0)
k_vals  = mean_t.index.to_numpy()
n_runs  = df['run'].nunique()

# Speedup relative to K=1 (serial baseline)
t_serial = mean_t.loc[1] if 1 in mean_t.index else mean_t.iloc[0]
speedup  = t_serial / mean_t.values
k_ideal  = np.linspace(k_vals[0], k_vals[-1], 200)

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))
fig.suptitle(
    f'K-scaling benchmark  (fixed threads={num_threads}, {n_runs} runs/config)',
    fontsize=13, y=1.01)

# ── Left panel: wall time vs K ──────────────────────────────────────────────
ax1.plot(k_vals, mean_t.values, marker='o', markersize=7, linewidth=2,
         color='steelblue', label='measured')
ax1.fill_between(k_vals,
                 mean_t.values - std_t.values,
                 mean_t.values + std_t.values,
                 color='steelblue', alpha=0.18, label='±1 std')

# Annotate each point with its value
for k, t in zip(k_vals, mean_t.values):
    ax1.annotate(f'{t:.1f}s',
                 xy=(k, t), xytext=(4, 6), textcoords='offset points',
                 fontsize=8.5, color='steelblue')

ax1.set_xlabel('K  (number of clusters)')
ax1.set_ylabel('Wall time (s)')
ax1.set_title('Wall time vs K')
ax1.set_xticks(k_vals)
ax1.tick_params(axis='x', rotation=45)
ax1.grid(True, alpha=0.3, linestyle='--')
ax1.legend()

# ── Right panel: speedup vs K ───────────────────────────────────────────────
# Ideal linear speedup (capped at num_threads)
ideal   = np.minimum(k_ideal / k_vals[0], num_threads)
ceiling = np.full_like(k_ideal, num_threads)

ax2.plot(k_ideal, ideal, linestyle='--', color='gray', linewidth=1.4,
         label='ideal linear')
ax2.plot(k_ideal, ceiling, linestyle=':', color='tomato', linewidth=1.6,
         label=f'thread ceiling ({num_threads})')
ax2.plot(k_vals, speedup, marker='s', markersize=7, linewidth=2,
         color='steelblue', label='measured speedup')

# Annotate each measured point
for k, s in zip(k_vals, speedup):
    ax2.annotate(f'{s:.2f}×',
                 xy=(k, s), xytext=(4, 6), textcoords='offset points',
                 fontsize=8.5, color='steelblue')

ax2.set_xlabel('K  (number of clusters)')
ax2.set_ylabel('Speedup  (T$_{K=1}$ / T$_K$)')
ax2.set_title('Speedup vs K')
ax2.set_xticks(k_vals)
ax2.tick_params(axis='x', rotation=45)
ax2.yaxis.set_minor_locator(ticker.AutoMinorLocator())
ax2.grid(True, alpha=0.3, linestyle='--')
ax2.legend()

fig.tight_layout()
fig.savefig(OUT_PNG, dpi=150, bbox_inches='tight')
plt.close(fig)
print(f'Saved {OUT_PNG}')
