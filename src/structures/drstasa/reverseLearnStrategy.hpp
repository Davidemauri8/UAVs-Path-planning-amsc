#ifndef REVERSE_LEARN_STRATEGY_HPP
#define REVERSE_LEARN_STRATEGY_HPP

#include "structures/pointsList.hpp"
#include <functional>
#include <random>
#include <vector>

// Reverse Learning Strategy (eq. 19-21, Liu et al. 2025).
// Applicata all'intera popolazione per sfuggire a ottimi locali.
// evalFn: lambda che calcola la fitness di un percorso di soli waypoint intermedi.
class ReverseLearnStrategy {
public:
    ReverseLearnStrategy(
        double xMin, double xMax,
        double yMin, double yMax,
        double zMin, double zMax,
        unsigned seed = 42
    );

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
