#ifndef REVERSE_LEARN_STRATEGY_HPP
#define REVERSE_LEARN_STRATEGY_HPP

#include "structures/pointsList.hpp"
#include <functional>
#include <random>
#include <vector>

// Reverse Learning Strategy applied to the entire population.
// For each individual, a dynamic reverse point is computed from the current population
// bounds and blended with the original solution; the reverse is accepted only if it
// improves fitness, providing a population-level escape from local optima.
class ReverseLearnStrategy {
public:
    ReverseLearnStrategy(
        double xMin, double xMax,
        double yMin, double yMax,
        double zMin, double zMax,
        unsigned seed = 42
    );

    // Applies the reverse learning update to every individual in pop.
    // evalFn: fitness function for a waypoint-only PointsList (no start/end).
    // nWaypoints: number of intermediate waypoints per individual.
    void apply(std::vector<PointsList>& pop,
               std::vector<double>&    fitVals,
               std::function<double(const PointsList&)> evalFn,
               int nWaypoints);

private:
    std::vector<double> flatten  (const PointsList& path) const;
    PointsList          unflatten(const std::vector<double>& v) const;
    PointsList          clampToBounds(const PointsList& path) const;

    double xMin_, xMax_, yMin_, yMax_, zMin_, zMax_;
    std::mt19937 rng_;
};

#endif
