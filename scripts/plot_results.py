import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_DIR    = os.path.join(SCRIPT_DIR, '..', 'output')

plt.rcParams.update({
    'font.size': 11,
    'axes.titlesize': 12,
    'axes.labelsize': 11,
    'legend.fontsize': 10,
    'axes.spines.top': False,
    'axes.spines.right': False,
})

# Plot 1: Execution time vs number of threads
df_par = pd.read_csv(os.path.join(OUT_DIR, 'bench_parallel.csv'))
g      = df_par.groupby('threads')['wall_time']
mean_t = g.mean()
std_t  = g.std()
threads = mean_t.index.to_numpy()
n_runs  = df_par['run'].nunique()

fig, ax = plt.subplots(figsize=(7, 4.5))

t1 = mean_t.iloc[0]

ax.plot(threads, mean_t.values, marker='o', markersize=7, linewidth=2,
        color='steelblue')

# Annotate actual speedup above each point
for t, m in zip(threads, mean_t.values):
    ax.annotate(f'{t1/m:.2f}×',
                xy=(t, m), xytext=(5, 8), textcoords='offset points',
                fontsize=9, color='steelblue')

ax.set_xlabel('Number of threads')
ax.set_ylabel('Execution time (s)')
ax.set_title(f'Parallel scaling')
ax.set_xticks(threads)
ax.grid(True, alpha=0.3, linestyle='--')
fig.tight_layout()

out1 = os.path.join(OUT_DIR, 'plot_parallel.png')
fig.savefig(out1, dpi=150, bbox_inches='tight')
plt.close(fig)
print(f'Saved {out1}')

# Plot 2: Fitness vs N obstacles
df_obs  = pd.read_csv(os.path.join(OUT_DIR, 'bench_obstacles.csv'))
g_sa    = df_obs.groupby('n_obstacles')['sa_fit']
g_dr    = df_obs.groupby('n_obstacles')['drstasa_fit']

sa_mean = g_sa.mean();  sa_std = g_sa.std()
dr_mean = g_dr.mean();  dr_std = g_dr.std()
x       = sa_mean.index.to_numpy()
n_runs2 = df_obs['run'].nunique()

fig, ax1 = plt.subplots(figsize=(7, 4.5))

ax1.plot(x, sa_mean.values, marker='s', markersize=7, linewidth=2,
         color='tomato', label='SA')
ax1.fill_between(x, sa_mean - sa_std, sa_mean + sa_std,
                 color='tomato', alpha=0.18)

ax1.plot(x, dr_mean.values, marker='o', markersize=7, linewidth=2,
         color='steelblue', label='DRSTASA')
ax1.fill_between(x, dr_mean - dr_std, dr_mean + dr_std,
                 color='steelblue', alpha=0.18)

ax1.set_xlabel('Number of obstacles')
ax1.set_ylabel('Fitness function')
ax1.set_title(f'SA vs DRSTASA')
ax1.set_xticks(x)
ax1.legend()
ax1.grid(True, alpha=0.3, linestyle='--')

fig.tight_layout()

out2 = os.path.join(OUT_DIR, 'plot_obstacles.png')
fig.savefig(out2, dpi=150, bbox_inches='tight')
plt.close(fig)
print(f'Saved {out2}')
