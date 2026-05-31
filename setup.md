# Algorithm Parameter Reference

This document lists **all configurable parameters** in the project, with the file, function, and meaning for each.

---

## 1. Global Scenario Parameters

**File:** [src/testiceland.cpp](src/testiceland.cpp) — `main()`

| Parameter | Current value | Description |
|-----------|--------------|-------------|
| `NWaypoints` | `4` | Number of intermediate waypoints per path segment |
| `K` | `4` | Number of UAVs / K-Means clusters |
| `zMin` | `800.0` | Minimum flight altitude (metres, local coordinate frame) |
| `zMax` | `950.0` | Maximum flight altitude (metres, local coordinate frame) |

`NWaypoints` is a template parameter (`constexpr int`): it propagates to `runPipelineOptimization<NWaypoints>` and from there to all sub-algorithms (SA, DRSTASA, LHS).

---

## 2. Geographic Origin (Iceland scenario)

**File:** [src/testiceland.cpp](src/testiceland.cpp) — `main()`

| Parameter | Value | Description |
|-----------|-------|-------------|
| `lat0` | `63.985` | GPS origin latitude (near Keflavik Airport) |
| `lon0` | `-22.605` | GPS origin longitude |

All CSV points and obstacle coordinates are expressed in metres relative to this origin. Conversion between GPS and metres is handled by `GeoUtils::toMeters` / `GeoUtils::toGPS`.

Conversion factors:
- `metersPerDegLat = 111320.0`
- `metersPerDegLon = 111320.0 × cos(lat0 × π/180) ≈ 48840 m/deg`

---

## 3. Fitness Function — Weights and Constraints

**File:** [src/structures/functions/fitnessUtilities.cpp](src/structures/functions/fitnessUtilities.cpp) — `sampleFitnessWeights(zMin, zMax)`

These weights are shared by **both** optimisers (SA and DRSTASA). The `FitnessWeights` struct is declared in [src/structures/fitness/fitnessFunction.hpp](src/structures/fitness/fitnessFunction.hpp) with all fields defaulting to `0.0`; the operative values live **only** in `fitnessUtilities.cpp`.

| Parameter | Value | Description |
|-----------|-------|-------------|
| `b1` | `12.0` | Weight for **path length** cost (F1) |
| `b2` | `20.0` | Weight for **obstacle proximity** cost (F2) |
| `b3` | `2.0` | Weight for **altitude deviation** cost (F3) |
| `b4` | `10.0` | Weight for **path smoothness** cost (F4) |
| `a1` | `2.5` | Sub-weight for **horizontal turn angle** (inside F4) |
| `a2` | `1.5` | Sub-weight for **climb angle variation** (inside F4) |
| `hMin` | `= zMin` | Minimum allowed altitude (taken from scenario) |
| `hMax` | `= zMax` | Maximum allowed altitude (taken from scenario) |
| `droneRadius` | `3.0` | Physical drone radius in metres (inflates obstacle radii) |

Total fitness: `F = b1·F1 + b2·F2 + b3·F3 + b4·F4`.  
Altitude violations or collisions return `+∞`, discarding the solution.

---

## 4. Cylindrical Obstacles — Iceland Scenario

**File:** [src/structures/functions/fitnessUtilities.cpp](src/structures/functions/fitnessUtilities.cpp) — `buildDefaultObstacles()`

Each obstacle is `CylinderObstacle(Point(x, y, 0, -1), radius, height, buffer)`.

**Field meanings:**
- `radius` — hard collision radius (drone cannot enter; penalty = ∞)
- `height` — cylinder height; above this altitude the obstacle has no effect
- `buffer` — soft-penalty zone thickness around the hard radius

| # | Name | x (m) | y (m) | radius (m) | height (m) | buffer (m) | Cluster |
|---|------|--------|--------|-----------|-----------|-----------|---------|
| 0 | Reykjanes Ridge | −25922.9 | 37805.7 | 8000 | 2000 | 100 | C1 |
| 1 | Snæfellsjökull | −31531.5 | 97295.0 | 5000 | 1500 | 90 | C1 |
| 2 | Vestfjarðahæðir | −75108.3 | 179579.3 | 5000 | 1500 | 80 | C2 |
| 3 | Ísafjarðarfjöll | −19021.8 | 177911.3 | 5000 | 1500 | 80 | C2 |
| 4 | Drangajökull | 26578.9 | 201262.3 | 6000 | 1500 | 90 | C2 |
| 5 | Langjökull | 127290.6 | −14455.9 | 18000 | 1500 | 120 | C3 |
| 6 | Hrafntinnusker | 174793.3 | −11898.4 | 3500 | 1500 | 70 | C3 |
| 7 | Katla / Mýrdalsjökull | 187522.5 | −56821.2 | 10000 | 1500 | 140 | C3 |
| 8 | Hverfjall | 246779.0 | 182359.1 | 2500 | 1500 | 60 | C0 |
| 9 | Krafla | 285795.7 | 190698.7 | 5000 | 1500 | 80 | C0 |
| 10 | Herðubreið | 319935.2 | 195146.5 | 3500 | 1500 | 100 | C0 |
| 11 | Askja / Dyngjufjöll | 371388.4 | 195813.7 | 8000 | 1500 | 120 | C0 |
| 12 | Snæfell | 388458.2 | 185806.2 | 5000 | 1500 | 110 | C0 |
| 13 | Kverkfjöll | 408942.0 | 145108.8 | 6000 | 1500 | 130 | C0 |
| 14 | Bárðarbunga | 430888.8 | 113418.3 | 9000 | 1500 | 150 | C0 |
| 15 | Öræfajökull | 401870.2 | 95071.1 | 5000 | 1500 | 120 | C0 |
| 16 | Hofsjökull | 372607.7 | 72832.1 | 14000 | 1500 | 130 | C0 |
| 17 | Eiríksjökull | 107050.7 | 86620.3 | 3500 | 1500 | 80 | visual |
| 18 | Langjökull (visual) | 122413.5 | 73944.1 | 17000 | 1500 | 120 | visual |
| 19 | Þórisjökull | 152163.7 | 85397.1 | 4000 | 1500 | 80 | visual |
| 20 | Hofsjökull (visual) | 179963.0 | 93848.0 | 13000 | 1500 | 130 | visual |
| 21 | Vatnajökull (NW edge) | 234586.3 | 68384.3 | 22000 | 1500 | 150 | visual |

Total: **22 obstacles**. The benchmark sweeps obstacle counts by calling `makeDefaultFitness(zMin, zMax, nObstacles)`, which trims this list via `buildDefaultObstacles()`.

---

## 5. K-Means

**File:** [src/testiceland.cpp](src/testiceland.cpp) — `KMeans(K, 100).run(allPoints)`

Class declared in [src/structures/kmeans.hpp](src/structures/kmeans.hpp).

| Parameter | Value | Description |
|-----------|-------|-------------|
| `K` | `4` (from `testiceland.cpp`) | Number of clusters (= number of UAVs) |
| `max_iterations` | `100` | Maximum iterations for centroid convergence |
| `seed` | `42` (constructor default) | Seed for random centroid initialisation |

---

## 6. TSP-SA (waypoint ordering within a cluster)

**File:** [src/structures/pipeline/pipelineRunner.hpp](src/structures/pipeline/pipelineRunner.hpp) — inside `runPipelineOptimization()`

```cpp
TspSA(fitness,
      std::make_shared<MetropolisCriterion>(),
      std::make_shared<ExponentialScheduler>(100.0, 0.01, 200, 0.95),
      500).run(cluster);
```

| Parameter | Value | Description |
|-----------|-------|-------------|
| `maxIter` | `500` | Maximum SA iterations for waypoint ordering |
| Criterion | `MetropolisCriterion` | Standard probabilistic acceptance criterion |
| Scheduler | `ExponentialScheduler` | Exponential cooling |
| T0 | `100.0` | Initial temperature |
| Tmin | `0.01` | Minimum temperature |
| `stab_it` | `200` | Stabilisation iterations per temperature level |
| `alpha` | `0.95` | Cooling factor: T(k+1) = T(k) · α |

---

## 7. Classic SA — Per-Segment Optimisation

**File:** [src/structures/segment/segmentOptimizer.hpp](src/structures/segment/segmentOptimizer.hpp)

### 7a. `runSegmentSA()` — single SA run

```cpp
SegmentSAResult<NWaypoints> runSegmentSA(
    ...,
    long maxIter = 5000)
```

| Parameter | Default | Description |
|-----------|---------|-------------|
| `maxIter` | `5000` | Maximum iterations of a single SA run |
| Scheduler T0 | `100.0` | Initial temperature |
| Scheduler Tmin | `0.01` | Minimum temperature |
| Scheduler `stab_it` | `15` | Stabilisation iterations per temperature level |
| Scheduler `alpha` | `0.95` | Cooling factor |
| `stepRadius` | computed | `0.1 × xyDiag / √Dim`; XY diagonal of segment bounds divided by dimension |
| `LocalSamplerFixed k` | `0.1` | Locality parameter: σ = k · stepRadius |

Budget: `stab_it × maxIter = 15 × 5000 = 75,000` fitness evaluations — matched to DRSTASA budget.

The search domain is an **axis-aligned hyperrectangle** spanning the segment's bounding box in XY and `[zMin, zMax]` in altitude.

### 7b. `runSegmentSAMultiStart()` — multi-start SA

```cpp
SegmentSAResult<NWaypoints> runSegmentSAMultiStart(
    ...,
    int  nRestarts        = 4,
    long maxIterPerRestart = 6000)
```

| Parameter | Default | Description |
|-----------|---------|-------------|
| `nRestarts` | `4` | Number of independent SA runs; best result is kept |
| `maxIterPerRestart` | `6000` | Iterations per restart (passed to `runSegmentSA`) |

### 7c. Segment bounds — `computeBounds()`

```cpp
SegmentBounds computeBounds(const PointsList& ordered,
                            double marginFactor = 1.0,
                            double marginMin    = 10.0);
```

| Parameter | Default | Description |
|-----------|---------|-------------|
| `marginFactor` | `1.0` | The XY bounding box is expanded by 100% on each side |
| `marginMin` | `10.0` m | Minimum absolute margin (prevents degenerate domains) |

---

## 8. LHS (Latin Hypercube Sampling)

**File:** used in [src/structures/segment/segmentOptimizer.hpp](src/structures/segment/segmentOptimizer.hpp) and [src/structures/drstasa/drstasa.cpp](src/structures/drstasa/drstasa.cpp)

```cpp
Lhs lhs(xMin, xMax, yMin, yMax, zMin, zMax, NWaypoints, idCluster);
```

Class declared in [src/structures/lhs/lhs.hpp](src/structures/lhs/lhs.hpp).

| Parameter | Source | Description |
|-----------|--------|-------------|
| `xMin, xMax` | segment bounds / DRSTASA::Config | X-axis sampling limits |
| `yMin, yMax` | segment bounds / DRSTASA::Config | Y-axis sampling limits |
| `zMin, zMax` | scenario (800–950 m for Iceland) | Altitude sampling limits |
| `n` | `= NWaypoints` | Number of samples generated (one per waypoint) |
| `idCluster` | `-1` (neutral) | Cluster ID assigned to generated points |

LHS is used **only** to generate the starting point (`startPoint`) of each SA run and the initial population of DRSTASA.

---

## 9. DRSTASA

**File:** [src/structures/functions/fitnessUtilities.cpp](src/structures/functions/fitnessUtilities.cpp) — `GetConfigurationDRST(NWaypoints)`

The `DRSTASA::Config` struct is declared in [src/structures/drstasa/drstasa.hpp](src/structures/drstasa/drstasa.hpp) with all fields defaulting to `0.0`. The operative values live **only** in `fitnessUtilities.cpp`.

```cpp
DRSTASA::Config GetConfigurationDRST(int NWaypoints)
```

| Parameter | Value | Description |
|-----------|-------|-------------|
| `popSize` | `30` | Population size |
| `maxIter` | `500` | Maximum DRSTASA iterations |
| `T0` | `100.0` | Initial SA temperature |
| `alpha` | `0.96` | Cooling factor: T(k+1) = T(k) · α |
| `p` | `0.4` | Reverse-learning threshold: applied when `rand > p` (60% of iterations) |
| `C0` | `3.0` | Initial disruption strength: C(iter) = C0 · (1 − iter/maxIter) |
| `eps_rot` | `180.0` | Step for the **rotation** operator |
| `eps_trans` | `150.0` | Step for the **translation** operator |
| `eps_scale` | `0.08` | Perturbation for the **scaling** operator |
| `eps_axis` | `0.08` | Perturbation for the **axis-transform** operator |
| `nWaypoints` | `= NWaypoints` | Number of intermediate waypoints (passed by caller) |
| `xMin, xMax` | segment bounds | X search domain limits (set in `pipelineRunner.hpp`) |
| `yMin, yMax` | segment bounds | Y search domain limits (set in `pipelineRunner.hpp`) |
| `zMin, zMax` | scenario | Altitude limits (set in `pipelineRunner.hpp`) |

Budget: `popSize × 5 operators × maxIter = 30 × 5 × 500 = 75,000` fitness evaluations — matched to SA budget.

The XY bounds are written in `pipelineRunner.hpp` just before creating the DRSTASA instance:
```cpp
DRSTASA::Config localCfg = drsCfg;   // copy base config
localCfg.xMin = b.xMin; localCfg.xMax = b.xMax;
localCfg.yMin = b.yMin; localCfg.yMax = b.yMax;
DRSTASA drstasa(fitness, localCfg);
```

### 9a. Reverse Learning Strategy

**File:** [src/structures/drstasa/reverseLearnStrategy.hpp](src/structures/drstasa/reverseLearnStrategy.hpp)

The reverse learning point is computed as:
- `xBar[d] = r5 · (dimMin[d] + dimMax[d] − x[d])`
- `xNew[d] = (1 − r6) · x[d] + r6 · xBar[d]`

Limits used are from `DRSTASA::Config` (xMin/xMax, yMin/yMax, zMin/zMax). Parameter `p` controls how often reverse learning is applied.

### 9b. Controller (Disruption Operator)

**File:** [src/simulatedAnnealing/controller/controller.hpp](src/simulatedAnnealing/controller/controller.hpp)

| Parameter | Source | Description |
|-----------|--------|-------------|
| `C0` | `DRSTASA::Config.C0 = 3.0` | Initial disruption strength |
| `total_iters` | `DRSTASA::Config.maxIter = 500` | Total iterations used to scale C over time |

Formula: `C(iter) = C0 · (1.0 − current_iter / total_iters)`

---

## 10. Temperature Schedulers (classic SA and DRSTASA)

**File:** [src/simulatedAnnealing/scheduler/scheduler.hpp](src/simulatedAnnealing/scheduler/scheduler.hpp)

### LinearScheduler
```cpp
LinearScheduler(double T0, double delta, double Tmin, long stab_it)
```
T(k+1) = T(k) − delta

| Parameter | Description |
|-----------|-------------|
| `T0` | Initial temperature |
| `delta` | Fixed decrement per step |
| `Tmin` | Minimum temperature (floor) |
| `stab_it` | Stabilisation iterations per temperature level |

### ExponentialScheduler *(used throughout the project)*
```cpp
ExponentialScheduler(double T0, double Tmin, long stab_it, double alpha = 0.95)
```
T(k+1) = T(k) · alpha

| Parameter | Description |
|-----------|-------------|
| `T0` | Initial temperature |
| `Tmin` | Minimum temperature |
| `stab_it` | Stabilisation iterations per temperature level |
| `alpha` | Cooling factor ∈ (0, 1) |

---

## 11. Benchmark Runner

**File:** [src/testbenchmark.cpp](src/testbenchmark.cpp)

Runs two experiments and writes CSV output files for plotting.

| Parameter | Value | Description |
|-----------|-------|-------------|
| `NWaypoints` | `4` | Intermediate waypoints per segment (template) |
| `K` | `4` | Number of clusters |
| `zMin / zMax` | `800.0 / 950.0` | Altitude band |
| `N_RUNS` | `5` | Repetitions per configuration |

**Experiment 1 — Parallelisation:**  
Threads ∈ {1, 2, 3, 4}, all 22 obstacles. Output: `output/bench_parallel.csv`  
Columns: `threads, run, sa_fit, drstasa_fit, wall_time`

**Experiment 2 — Fitness vs obstacle count:**  
Threads = K = 4, obstacle count swept over 7 evenly-spaced values from 0 to `maxObs` (22). Output: `output/bench_obstacles.csv`  
Columns: `n_obstacles, run, sa_fit, drstasa_fit, wall_time`

K-Means is run **once** at startup; the same cluster assignment is reused across all N_RUNS repetitions.

---

## 12. Plot Generation

**File:** [scripts/plot_results.py](scripts/plot_results.py)

Reads the two benchmark CSVs and generates:

| Output file | Content |
|-------------|---------|
| `output/plot_parallel.png` | Execution time vs number of threads (speedup annotated) |
| `output/plot_obstacles.png` | SA vs DRSTASA total fitness vs number of obstacles (±1σ bands) |

Run from any directory:
```bash
python scripts/plot_results.py
```

---

## 13. Quick Reference — What to Change and Where

| What to change | File | Function / line |
|----------------|------|-----------------|
| Number of UAVs K | [src/testiceland.cpp](src/testiceland.cpp) | `constexpr int K` |
| Waypoints per segment | [src/testiceland.cpp](src/testiceland.cpp) | `constexpr int NWaypoints` |
| Flight altitude band | [src/testiceland.cpp](src/testiceland.cpp) | `const double zMin, zMax` |
| Fitness weights (b1–b4, a1–a2) | [src/structures/functions/fitnessUtilities.cpp](src/structures/functions/fitnessUtilities.cpp) | `sampleFitnessWeights()` |
| Drone radius | [src/structures/functions/fitnessUtilities.cpp](src/structures/functions/fitnessUtilities.cpp) | `sampleFitnessWeights()` → `droneRadius` |
| Obstacles | [src/structures/functions/fitnessUtilities.cpp](src/structures/functions/fitnessUtilities.cpp) | `buildDefaultObstacles()` |
| DRSTASA parameters | [src/structures/functions/fitnessUtilities.cpp](src/structures/functions/fitnessUtilities.cpp) | `GetConfigurationDRST()` |
| SA iterations per segment | [src/structures/segment/segmentOptimizer.hpp](src/structures/segment/segmentOptimizer.hpp) | `runSegmentSAMultiStart()` → `maxIterPerRestart`, `nRestarts` |
| SA scheduler (per segment) | [src/structures/segment/segmentOptimizer.hpp](src/structures/segment/segmentOptimizer.hpp) | `runSegmentSA()` → `ExponentialScheduler(...)` |
| TSP-SA iterations | [src/structures/pipeline/pipelineRunner.hpp](src/structures/pipeline/pipelineRunner.hpp) | `TspSA(..., 500)` |
| TSP-SA scheduler | [src/structures/pipeline/pipelineRunner.hpp](src/structures/pipeline/pipelineRunner.hpp) | `ExponentialScheduler(100.0, 0.01, 200, 0.95)` |
| K-Means max iterations | [src/testiceland.cpp](src/testiceland.cpp) | `KMeans(K, 100)` |
| Segment bounding box margins | [src/structures/segment/segmentOptimizer.hpp](src/structures/segment/segmentOptimizer.hpp) | `computeBounds()` → `marginFactor`, `marginMin` |
| Number of OMP threads | [src/testiceland.cpp](src/testiceland.cpp) | `runPipelineOptimization<NWaypoints>(..., /*numThreads=*/K)` |
| Benchmark repetitions | [src/testbenchmark.cpp](src/testbenchmark.cpp) | `N_RUNS` |
