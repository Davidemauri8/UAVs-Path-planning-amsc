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

class DRSTASA {
public:
    // All values are intentionally left at 0.0. 
    // Configure them in fitnessUtilities.cpp.
    struct Config {
        int    popSize    = 0;   // population size
        int    maxIter    = 0;   // maximum number of iterations
        double T0         = 0.0; // initial temperature
        double alpha      = 0.0; // cooling rate
        double p          = 0.0; // probability threshold for applying reverse learning
        double eps_rot    = 0.0; // rotation operator step size
        double eps_trans  = 0.0; // translation operator step size
        double eps_scale  = 0.0; // scaling operator perturbation magnitude
        double eps_axis   = 0.0; // axis-transform operator perturbation magnitude
        double C0         = 0.0; // disruption operator initial strength
        double xMin = 0.0, xMax = 0.0;
        double yMin = 0.0, yMax = 0.0;
        double zMin = 0.0, zMax = 0.0;
        int    nWaypoints = 0;   // number of intermediate waypoints per path

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
