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

// DRSTASA — main orchestrator of the DR-STASA algorithm.
// Reference: Liu et al., Appl. Sci. 2025, 15, 6064.
//
// Each iteration of the main loop delegates to:
//   STASANeighbourhoodDyn — four STASA perturbation operators (eq. 10-13)
//   ReverseLearnStrategy  — population-level reverse learning (eq. 19-21)
//   Controller            — disruption operator for diversity (eq. 15-18)
//   MetropolisCriterion   — Metropolis acceptance rule
//   ExponentialScheduler  — geometric cooling schedule
class DRSTASA {
public:
    struct Config {
        int    popSize    = 20;    // population size
        int    maxIter    = 300;   // maximum number of iterations
        double T0         = 100.0; // initial temperature
        double alpha      = 0.93;  // cooling rate
        double p          = 0.5;   // probability threshold for applying reverse learning
        double eps_rot    = 150.0; // rotation operator step size
        double eps_trans  = 100.0; // translation operator step size
        double eps_scale  = 0.05;  // scaling operator perturbation magnitude
        double eps_axis   = 0.05;  // axis-transform operator perturbation magnitude
        double C0         = 2.0;   // disruption operator initial strength
        double xMin = 0.0, xMax = 0.0;
        double yMin = 0.0, yMax = 0.0;
        double zMin = 0.0, zMax = 0.0;
        int    nWaypoints = 4;     // number of intermediate waypoints per path

        Config() = default;
        Config(int nWaypoints, double zMin, double zMax);
    };

    DRSTASA(const FitnessFunction& fitness, const Config& cfg, unsigned seed = 42);

    // Optimises the sequence of intermediate waypoints between start and end.
    // Returns a PointsList containing start + best waypoints + end.
    PointsList run(const Point& start, const Point& end);

    // Returns the best fitness value found in the last call to run().
    double lastBestFit() const { return bestFit_; }

private:
    // Builds the full path start+waypoints+end and evaluates its fitness.
    // Returns 1e12 for infinite or NaN values to keep the optimiser stable.
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
