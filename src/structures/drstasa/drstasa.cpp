#include "structures/drstasa/drstasa.hpp"
#include "structures/lhs/lhs.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

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

    auto criterion = std::make_shared<MetropolisCriterion>();
    auto scheduler = std::make_shared<ExponentialScheduler>(
        cfg_.T0, 1e-6, 1, cfg_.alpha);
    Controller controller(cfg_.C0, cfg_.maxIter);

    std::uniform_real_distribution<> uni(0.0, 1.0);
    std::uniform_int_distribution<> neighborDist(0, popSize - 2);

    for (int iter = 0; iter < cfg_.maxIter; ++iter) {

        // Apply all four STASA operators to each individual,
        // keep the best candidate, and accept via Metropolis criterion
        for (int i = 0; i < popSize; ++i) {
            neighbourhood_.from(prevPop[i], pop[i]);
            std::array<PointsList, 4> candidates = neighbourhood_.generateAll();

            int    bestK      = 0;
            double bestNewFit = evalWaypoints(candidates[0], start, end);
            for (int k = 1; k < 4; ++k) {
                double f = evalWaypoints(candidates[k], start, end);
                if (f < bestNewFit) { bestNewFit = f; bestK = k; }
            }

            double delta = bestNewFit - fitVals[i];
            if (criterion->accept(delta, iter, scheduler->temp())) {
                prevPop[i] = pop[i];
                pop[i]     = candidates[bestK];
                fitVals[i] = bestNewFit;
                if (bestNewFit < bestFit_) {
                    bestFit_ = bestNewFit;
                    bestPath = candidates[bestK];
                }
            }
        }

        // Disruption operator perturbs each individual
        // using the global best and a random neighbour to maintain diversity
        for (int i = 0; i < popSize; ++i) {
            int j = neighborDist(rng_);
            if (j >= i) ++j;
            controller.applyDisruption(pop[i], bestPath, pop[j], iter);
            fitVals[i] = evalWaypoints(pop[i], start, end);
            if (fitVals[i] < bestFit_) {
                bestFit_ = fitVals[i];
                bestPath = pop[i];
            }
        }

        // Reverse learning is applied
        if (uni(rng_) > cfg_.p)
            reverseLearn_.apply(pop, fitVals,
                [&](const PointsList& wp) { return evalWaypoints(wp, start, end); },
                cfg_.nWaypoints);

        // Update global best after all operators
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
