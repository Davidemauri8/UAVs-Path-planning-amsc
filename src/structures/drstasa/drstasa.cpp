#include "structures/drstasa/drstasa.hpp"
#include "structures/lhs/lhs.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <omp.h>

DRSTASA::Config::Config(int nWaypointsVal, double zMinVal, double zMaxVal)
    : nWaypoints(nWaypointsVal), zMin(zMinVal), zMax(zMaxVal) {}

DRSTASA::DRSTASA(const FitnessFunction& fitness, const Config& cfg, unsigned seed)
    : fitness_(fitness), cfg_(cfg), rng_(seed),
      neighbourhood_(cfg.eps_rot, cfg.eps_trans, cfg.eps_scale, cfg.eps_axis,
                     cfg.xMin, cfg.xMax, cfg.yMin, cfg.yMax, cfg.zMin, cfg.zMax,
                     seed),
      reverseLearn_(cfg.xMin, cfg.xMax, cfg.yMin, cfg.yMax, cfg.zMin, cfg.zMax,
                    seed + 1) {}

// Assembles the full path and evaluates it
double DRSTASA::evalWaypoints(const PointsList& waypoints,
                               const Point& start, const Point& end) const {
    PointsList fullPath;
    fullPath.addPoint(start);
    for (int i = 0; i < waypoints.size(); ++i)
        fullPath.addPoint(Point(waypoints.getX(i), waypoints.getY(i),
                                waypoints.getZ(i), -1));
    fullPath.addPoint(end);
    double f = fitness_.evaluate(fullPath);
    if (std::isinf(f) || std::isnan(f)) return 1e12;
    return f;
}

// Main optimisation loop.
PointsList DRSTASA::run(const Point& start, const Point& end) {
    int popSize = cfg_.popSize;

    // Initialise the population with Latin Hypercube Sampling
    std::vector<PointsList> pop;
    pop.reserve(popSize);
    for (int i = 0; i < popSize; ++i) {
        Lhs lhs(cfg_.xMin, cfg_.xMax, cfg_.yMin, cfg_.yMax,
                cfg_.zMin, cfg_.zMax, cfg_.nWaypoints, -1);
        pop.push_back(lhs.generatePopulation());
    }
    std::vector<PointsList> prevPop = pop;

    // Initial fitness evaluation
    std::vector<double> fitVals(popSize);
    for (int i = 0; i < popSize; ++i)
        fitVals[i] = evalWaypoints(pop[i], start, end);

    int bestIdx = static_cast<int>(
        std::min_element(fitVals.begin(), fitVals.end()) - fitVals.begin());
    PointsList bestPath = pop[bestIdx];
    bestFit_ = fitVals[bestIdx];

    auto scheduler = std::make_shared<ExponentialScheduler>(
        cfg_.T0, 1e-6, 1, cfg_.alpha);

    std::uniform_real_distribution<> uni(0.0, 1.0);

    for (int iter = 0; iter < cfg_.maxIter; ++iter) {

        // ── STASA operators ──────────────────────────────────────────────────
        // Each individual is independent: parallelise over the population.
        // The guard prevents nesting when called from the outer cluster loop.
        #pragma omp parallel if(!omp_in_parallel())
        {
            const int    tid      = omp_get_thread_num();
            const double T        = scheduler->temp();
            const unsigned thrSeedA = static_cast<unsigned>(iter * 1009u + tid * 9973u);
            const unsigned thrSeedB = thrSeedA ^ 0xdeadbeef;

            // Per-thread neighbourhood (avoids races on the shared rng_)
            STASANeighbourhoodDyn thrNbhd(
                cfg_.eps_rot, cfg_.eps_trans, cfg_.eps_scale, cfg_.eps_axis,
                cfg_.xMin, cfg_.xMax, cfg_.yMin, cfg_.yMax, cfg_.zMin, cfg_.zMax,
                thrSeedA);

            std::mt19937 thrRng(thrSeedB);
            std::uniform_real_distribution<double> thrUni(0.0, 1.0);

            double     localBestFit = std::numeric_limits<double>::max();
            int        localBestIdx = -1;

            #pragma omp for schedule(static)
            for (int i = 0; i < popSize; ++i) {
                thrNbhd.from(prevPop[i], pop[i]);
                std::array<PointsList, 4> candidates = thrNbhd.generateAll();

                int    bestK    = 0;
                double bestNF   = evalWaypoints(candidates[0], start, end);
                for (int k = 1; k < 4; ++k) {
                    double f = evalWaypoints(candidates[k], start, end);
                    if (f < bestNF) { bestNF = f; bestK = k; }
                }

                // Inline Metropolis — avoids sharing MetropolisCriterion::gen
                const double delta    = bestNF - fitVals[i];
                const bool   accepted = (delta < 0.0) ||
                    (std::exp(-delta / T) >= thrUni(thrRng));

                if (accepted) {
                    prevPop[i] = pop[i];
                    pop[i]     = candidates[bestK];
                    fitVals[i] = bestNF;
                    if (bestNF < localBestFit) {
                        localBestFit = bestNF;
                        localBestIdx = i;
                    }
                }
            }

            // Merge thread-local best into the global best
            if (localBestIdx >= 0) {
                #pragma omp critical
                {
                    if (localBestFit < bestFit_) {
                        bestFit_ = localBestFit;
                        bestPath = pop[localBestIdx];
                    }
                }
            }
        } // end parallel — implicit barrier guarantees bestPath is up to date

        // ── Disruption operator ───────────────────────────────────────────────
        // Snapshot pop so each thread can safely read a random neighbour while
        // writing its own index (Jacobi-style update, standard in parallel MH).
        const std::vector<PointsList> popSnap = pop;

        #pragma omp parallel if(!omp_in_parallel())
        {
            const int    tid      = omp_get_thread_num();
            const unsigned thrSeed = static_cast<unsigned>(iter * 2003u + tid * 8191u);

            // Per-thread controller keeps its own rng
            Controller thrCtrl(cfg_.C0, cfg_.maxIter, thrSeed);

            std::mt19937 thrRng(thrSeed ^ 0xcafebabe);
            std::uniform_int_distribution<int> jDist(0, popSize - 2);

            double localBestFit = std::numeric_limits<double>::max();
            int    localBestIdx = -1;

            #pragma omp for schedule(static)
            for (int i = 0; i < popSize; ++i) {
                int j = jDist(thrRng);
                if (j >= i) ++j;

                // write pop[i], read popSnap[j] (immutable) and bestPath (immutable)
                thrCtrl.applyDisruption(pop[i], bestPath, popSnap[j], iter);
                fitVals[i] = evalWaypoints(pop[i], start, end);

                if (fitVals[i] < localBestFit) {
                    localBestFit = fitVals[i];
                    localBestIdx = i;
                }
            }

            if (localBestIdx >= 0) {
                #pragma omp critical
                {
                    if (localBestFit < bestFit_) {
                        bestFit_ = localBestFit;
                        bestPath = pop[localBestIdx];
                    }
                }
            }
        } // end parallel

        // ── Reverse learning ─────────────────────────────────────────────────
        if (uni(rng_) > cfg_.p)
            reverseLearn_.apply(pop, fitVals,
                [&](const PointsList& wp) { return evalWaypoints(wp, start, end); },
                cfg_.nWaypoints);

        // Sync global best after all operators
        for (int i = 0; i < popSize; ++i) {
            if (fitVals[i] < bestFit_) {
                bestFit_ = fitVals[i];
                bestPath = pop[i];
            }
        }

        scheduler->update(iter + 1);
    }

    PointsList result;
    result.addPoint(start);
    for (int i = 0; i < bestPath.size(); ++i)
        result.addPoint(Point(bestPath.getX(i), bestPath.getY(i),
                              bestPath.getZ(i), -1));
    result.addPoint(end);
    return result;
}
