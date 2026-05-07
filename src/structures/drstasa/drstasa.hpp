#ifndef DRSTASA_HPP
#define DRSTASA_HPP

#include "structures/fitness/fitnessFunction.hpp"
#include "structures/pointsList.hpp"
#include "structures/point.hpp"
#include "structures/drstasa/stasaNeighbourhood.hpp"
#include "structures/drstasa/reverseLearnStrategy.hpp"
#include "controller/controller.hpp"
#include "criterion/criterion.hpp"
#include "scheduler/scheduler.hpp"
#include <vector>
#include <random>
#include <memory>

// DRSTASA — Orchestratore del loop principale.
// Paper: Liu et al., Appl. Sci. 2025, 15, 6064.
//
// Delega:
//   STASANeighbourhoodDyn → 4 operatori STASA (eq. 10-13)
//   ReverseLearnStrategy  → reverse learning (eq. 19-21)
//   Controller            → disruption operator (eq. 15-18)
//   MetropolisCriterion   → accettazione Metropolis
//   ExponentialScheduler  → raffreddamento
class DRSTASA {
public:
    struct Config {
        int    popSize    = 20;
        int    maxIter    = 300;
        double T0         = 100.0;
        double alpha      = 0.93;
        double p          = 0.5;
        double eps_rot    = 150.0;
        double eps_trans  = 100.0;
        double eps_scale  = 0.05;
        double eps_axis   = 0.05;
        double C0         = 2.0;
        double xMin, xMax;
        double yMin, yMax;
        double zMin, zMax;
        int    nWaypoints = 4;

        Config() = default;
        Config(int nWaypoints, double zMin, double zMax);
    };

    DRSTASA(const FitnessFunction& fitness, const Config& cfg, unsigned seed = 42);

    PointsList run(const Point& start, const Point& end);

    double lastBestFit() const { return bestFit_; }

private:
    double evalWaypoints(const PointsList& waypoints,
                         const Point& start, const Point& end) const;

    const FitnessFunction& fitness_;
    Config                 cfg_;
    std::mt19937           rng_;
    double                 bestFit_ = 1e18;

    STASANeighbourhoodDyn neighbourhood_;
    ReverseLearnStrategy  reverseLearn_;
};

#endif
