# UAV Path Planning — AMSC Project

A C++17 framework for multi-objective UAV path planning in obstacle-laden environments. The system optimizes intermediate waypoints between a set of survey targets by combining K-means clustering, TSP-based ordering, and two competing segment-level metaheuristics: classic multi-start Simulated Annealing (SA) and a novel population-based hybrid, **DRSTASA**.

---

## Problem Statement

Given a set of GPS waypoints (e.g., earthquake survey targets), the system computes a flyable UAV path that minimizes total length, collision risk with cylindrical obstacles, altitude deviation, and trajectory roughness — all subject to hard altitude band constraints.

---

## Pipeline

The optimization follows a three-stage hierarchical decomposition:

```
GPS Waypoints
     │
     ▼
K-Means Clustering          — partitions points into K spatial groups
     │
     ▼  (for each cluster)
TSP-SA Ordering             — finds a good visiting sequence via 2-opt SA
     │
     ▼  (for each consecutive pair)
Segment Optimization        — inserts NWaypoints intermediate waypoints
   ├── Multi-start SA
   └── DRSTASA
     │
     ▼
GPS Export (KML / DAE)
```

All optimization runs in local metric coordinates (meters); GPS↔metric conversion uses a first-order equirectangular projection anchored at a user-specified origin.

---

## Fitness Function

Every candidate path is evaluated by a weighted sum of four sub-objectives:

**F = b₁·F₁ + b₂·F₂ + b₃·F₃ + b₄·F₄**

| Term | Meaning | Hard constraint |
|------|---------|-----------------|
| F₁ | Total Euclidean path length | — |
| F₂ | Cumulative obstacle proximity penalty | Returns ∞ on collision |
| F₃ | Deviation from mid-altitude band | Returns ∞ outside [hMin, hMax] |
| F₄ | Path roughness (turning angles + climb-angle variation) | — |

F₂ and F₃ are evaluated first; any violation short-circuits to ∞ before the more expensive F₁ and F₄ are computed. All weights and altitude bounds are configured in `src/structures/functions/fitnessUtilities.cpp`.

### Obstacle Model

The only implemented obstacle type is `CylinderObstacle`: a vertical cylinder defined by a center (x, y), collision radius r, height h, and a soft buffer zone b. For each path segment, the closest point to the cylinder axis is computed analytically. The cost is:

- **0** if the segment passes beyond the buffer zone, or entirely above height h
- **∞** if the segment penetrates the hard radius r
- **(b + r) − d** (linear) if the segment falls within the buffer zone

---

## Algorithms

### TSP-SA
A standard SA with a 2-opt neighbourhood (random swap + reversal) used to order the waypoints within each cluster before segment optimization. The fitness is the total path length through the cluster evaluated by the full fitness function.

### Multi-start SA
A classic SA engine with a circular neighbourhood sampler. Four independent restarts are run per segment; the best result is retained. The search domain is a sphere centered at the cluster bounding-box midpoint, expanded by a 20 % safety margin (minimum 50 m).

### DRSTASA
A population-based SA variant that extends the STASA operator set with a disruption mechanism and a reverse-learning strategy.

**Population:** initialized via Latin Hypercube Sampling (LHS) over the segment's spatial domain.

**Per iteration, each individual undergoes three operations:**

1. **STASA Neighbourhood** — four geometric perturbation operators applied in sequence:
   - *Rotation* — hypersphere exploration scaled by `eps_rot`
   - *Translation* — step along the previous displacement direction, scaled by `eps_trans`
   - *Scaling* — global multiplicative Gaussian perturbation with magnitude `eps_scale`
   - *Axis Transform* — single-coordinate Gaussian perturbation with magnitude `eps_axis`

2. **Disruption Operator** — blends the individual with the global best and a random neighbour; perturbation strength decays from `C0` over iterations to maintain late-stage exploitation.

3. **Reverse Learning** — applied with probability `1 − p`; computes a dynamic mirror point from the current population bounds and accepts it if it improves fitness, providing a population-level escape from local optima.

Acceptance of new solutions follows the Metropolis criterion with an exponential cooling schedule (rate `alpha`, initial temperature `T0`).

All operative parameters are set in `src/structures/functions/fitnessUtilities.cpp`; the struct fields in `drstasa.hpp` are intentionally zero-initialized.

---

## Repository Structure

```
src/
├── structures/
│   ├── fitness/            — FitnessFunction, FitnessWeights
│   ├── obstacles/          — Obstacle (abstract), CylinderObstacle
│   ├── drstasa/            — DRSTASA, STASANeighbourhood, ReverseLearnStrategy
│   ├── lhs/                — Latin Hypercube Sampling
│   ├── tsp/                — TSP-SA
│   ├── segment/            — SegmentOptimizer, SegmentBounds
│   ├── pipeline/           — pipelineRunner (serial + OpenMP)
│   ├── geo/                — GPS ↔ metric conversion
│   ├── exporters/          — KML, DAE, CSV I/O
│   └── functions/          — fitnessUtilities (all operative parameters)
├── simulatedAnnealing/     — generic template SA engine
├── test11.cpp              — serial pipeline on 200-point dataset
├── test12.cpp              — OpenMP-parallel pipeline
└── test13.cpp              — benchmark suite (K × threads grid)
```

---

## Build

```bash
mkdir build && cd build
cmake ..
cmake --build .
```

Three executables are produced: `test11` (serial), `test12` (parallel), `test13` (benchmark).

---

## Configuration

All tunable parameters live in a single file: **`src/structures/functions/fitnessUtilities.cpp`**.

**Fitness weights** (`sampleFitnessWeights`):
```
b1 = 5.0    path length
b2 = 10.0   obstacle cost
b3 = 1.0    altitude deviation
b4 = 5.0    smoothness
a1 = 1.0    horizontal turning-angle coefficient (inside F4)
a2 = 1.0    vertical climb-angle coefficient (inside F4)
hMin, hMax  altitude band (passed at runtime)
```

**DRSTASA** (`GetConfigurationDRST`):
```
popSize   = 20      population size
maxIter   = 300     maximum iterations
T0        = 100.0   initial temperature
alpha     = 0.93    cooling rate
p         = 0.5     reverse-learning probability threshold
C0        = 2.0     disruption operator initial strength
eps_rot   = 150.0   rotation step size
eps_trans = 100.0   translation step size
eps_scale = 0.05    scaling perturbation magnitude
eps_axis  = 0.05    axis-transform perturbation magnitude
```

Spatial bounds (`xMin`, `xMax`, `yMin`, `yMax`) are computed at runtime from each cluster's bounding box and are never set in the configuration file.

---

## Output

Each test exports two optimized paths — one from SA, one from DRSTASA — in:
- **KML** (`sa_path.kml`, `drstasa_path.kml`) — viewable in Google Earth
- **DAE** (`drone.dae`) — 3D mesh for visualization

`test13` additionally writes a CSV benchmark table (`output/benchmark.csv`) with columns: `K`, `N_points`, `num_threads`, `wall_time`, `sa_fit`, `drstasa_fit`.

---

## Authors

Davide Mauri and Tommaso Roncaglio 
