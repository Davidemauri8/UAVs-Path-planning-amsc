#include "structures/drstasa/drstasa.hpp"
#include "structures/lhs/lhs.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <future>
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

// ─── Valutazione fitness ──────────────────────────────────────────────────────

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

// ─── Loop principale ──────────────────────────────────────────────────────────

PointsList DRSTASA::run(const Point& start, const Point& end) {
    int popSize = cfg_.popSize;

    // Inizializzazione con LHS (Section 4.1)
    std::vector<PointsList> pop;
    pop.reserve(popSize);
    for (int i = 0; i < popSize; ++i) {
        Lhs lhs(cfg_.xMin, cfg_.xMax, cfg_.yMin, cfg_.yMax,
                cfg_.zMin, cfg_.zMax, cfg_.nWaypoints, -1);
        pop.push_back(lhs.generatePopulation());
    }
    std::vector<PointsList> prevPop = pop;

    // Valutazione iniziale
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

        // Passo 4 (Algorithm 1): tutti e 4 gli operatori STASA, valutati in parallelo
        for (int i = 0; i < popSize; ++i) {
            neighbourhood_.from(prevPop[i], pop[i]);
            std::array<PointsList, 4> candidates = neighbourhood_.generateAll();

            std::array<std::future<double>, 4> futures;
            for (int k = 0; k < 4; ++k)
                futures[k] = std::async(std::launch::async,
                    [&, k]() { return evalWaypoints(candidates[k], start, end); });

            int    bestK      = 0;
            double bestNewFit = futures[0].get();
            for (int k = 1; k < 4; ++k) {
                double f = futures[k].get();
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

        // Passo 7-8 (Algorithm 1): Disruption Operator
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

        // Passo 9 (Algorithm 1): Reverse Learning (se rand > p)
        if (uni(rng_) > cfg_.p)
            reverseLearn_.apply(pop, fitVals,
                [&](const PointsList& wp) { return evalWaypoints(wp, start, end); },
                cfg_.nWaypoints);

        // Aggiorna best globale dopo tutti gli operatori
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
