"""
Benchmark visualization for UAV path-planning parallelisation.

Reads:
  output/benchmark16.csv  -- strong-scaling data (K fixed, threads 1..maxThreads)
  output/benchmark14.csv  -- speedup-vs-K data   (K varies, threads 1 vs max)

Produces:
  output/benchmark.png    -- 2x2 paper-style figure (or 1x3 fallback)
"""

import os
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT    = os.path.join(os.path.dirname(__file__), "..")
CSV14   = os.path.join(ROOT, "output", "benchmark14.csv")
CSV16   = os.path.join(ROOT, "output", "benchmark16.csv")
OUT_PNG = os.path.join(ROOT, "output", "benchmark.png")

REAL_COLOR  = "#19a3c8"   # teal  (real measurements)
IDEAL_COLOR = "#e8342a"   # red   (ideal scaling)
SA_COLOR    = "#f07d2a"   # orange
DRS_COLOR   = "#3dab4f"   # green

# ── load benchmark14 (speedup vs K) ──────────────────────────────────────────
df14     = pd.read_csv(CSV14)
serial14 = df14[df14["num_threads"] == 1].set_index("K")
par14    = df14[df14["num_threads"] >  1].set_index("K")
common_K = serial14.index.intersection(par14.index)
speedup_K = serial14.loc[common_K, "wall_time"] / par14.loc[common_K, "wall_time"]
n_threads_14 = int(par14["num_threads"].iloc[0])
N_pts = int(df14["N_points"].iloc[0])

has16 = os.path.exists(CSV16)

# ── figure layout ─────────────────────────────────────────────────────────────
if has16:
    df16    = pd.read_csv(CSV16)
    K_fixed = int(df16["K"].iloc[0])
    threads = df16["num_threads"].values.astype(float)
    times   = df16["wall_time"].values
    T0      = times[0]
    ideal_t = T0 / threads
    speedup_p = T0 / times

    fig, axes = plt.subplots(2, 2, figsize=(13, 10))
    fig.suptitle(
        f"UAV Path Planning — Parallelisation Analysis\n"
        f"L'Aquila dataset  |  N = {N_pts} waypoints  |  OMP threads",
        fontsize=13, fontweight='bold'
    )
    ax_sc, ax_su = axes[0, 0], axes[0, 1]
    ax_sk, ax_q  = axes[1, 0], axes[1, 1]

    # ── top-left: strong scalability (time vs threads) ────────────────────────
    ax_sc.plot(threads, times,    marker='o', color=REAL_COLOR,  lw=2,
               label="Real")
    ax_sc.plot(threads, ideal_t,  marker='o', color=IDEAL_COLOR, lw=1.5,
               ls='--', label=r"Ideal $T_0 / p$")
    ax_sc.set_xlabel("Number of threads $p$")
    ax_sc.set_ylabel("Wall time (s)")
    ax_sc.set_title(f"Strong Scalability  (K = {K_fixed}, fixed)")
    ax_sc.legend(); ax_sc.grid(True, ls=':')

    # ── top-right: speedup vs threads ─────────────────────────────────────────
    ax_su.plot(threads, speedup_p, marker='o', color=REAL_COLOR,  lw=2,
               label="Real speedup")
    ax_su.plot(threads, threads,   color=IDEAL_COLOR, lw=1.5, ls='--',
               label="Ideal speedup")
    ax_su.set_xlabel("Number of threads $p$")
    ax_su.set_ylabel(r"Speedup  $S(p) = T_0 / T(p)$")
    ax_su.set_title(f"Speedup vs Threads  (K = {K_fixed}, fixed)")
    ax_su.legend(); ax_su.grid(True, ls=':')

else:
    # fallback: 1×3 when benchmark16 doesn't exist yet
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    fig.suptitle(
        f"UAV Path Planning — {n_threads_14}-thread benchmark  |  N = {N_pts} pts",
        fontsize=13, fontweight='bold'
    )
    ax_wt, ax_sk, ax_q = axes[0], axes[1], axes[2]

    ax_wt.plot(serial14.index, serial14["wall_time"], marker='s',
               color=IDEAL_COLOR, lw=2, label="Serial (1 thread)")
    ax_wt.plot(par14.index,    par14["wall_time"],    marker='o',
               color=REAL_COLOR,  lw=2, label=f"Parallel ({n_threads_14} threads)")
    ax_wt.set_xlabel("K (clusters)"); ax_wt.set_ylabel("Wall time (s)")
    ax_wt.set_title("Execution Time vs K"); ax_wt.legend(); ax_wt.grid(True, ls=':')

# ── speedup vs K ──────────────────────────────────────────────────────────────
ax_sk.plot(common_K, speedup_K.values, marker='o', color=REAL_COLOR,  lw=2,
           label=f"Real speedup ({n_threads_14} threads)")
ax_sk.axhline(n_threads_14, color=IDEAL_COLOR, lw=1.5, ls='--',
              label=f"Ideal speedup = {n_threads_14}")
ax_sk.axhline(1, color='gray', lw=0.8, ls=':', label="Speedup = 1")
ax_sk.set_xlabel("K (clusters)"); ax_sk.set_ylabel("Speedup")
ax_sk.set_title(f"Speedup vs K  ({n_threads_14} threads)")
ax_sk.legend(); ax_sk.grid(True, ls=':')

# ── quality: SA vs DRSTASA (log scale to handle penalty outliers) ─────────────
ax_q.plot(serial14.index, serial14["sa_fit"],      marker='s', color=SA_COLOR,  lw=2,
          label="SA fitness")
ax_q.plot(serial14.index, serial14["drstasa_fit"], marker='o', color=DRS_COLOR, lw=2,
          label="DRSTASA fitness")
ax_q.set_yscale("log")
ax_q.set_xlabel("K (clusters)"); ax_q.set_ylabel("Total fitness  (lower = better, log scale)")
ax_q.set_title("Quality: SA vs DRSTASA")
ax_q.legend(); ax_q.grid(True, ls=':', which='both')

plt.tight_layout()
plt.savefig(OUT_PNG, dpi=150)
print(f"Plot saved -> {OUT_PNG}")
