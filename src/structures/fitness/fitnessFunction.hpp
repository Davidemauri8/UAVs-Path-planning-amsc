#ifndef FITNESS_FUNCTION_HPP
#define FITNESS_FUNCTION_HPP

#include "../pointsList.hpp"
#include "../obstacles/obstacle.hpp"
#include <vector>
#include <memory>
#include <limits>

// Weights and altitude bounds used to combine the four fitness sub-functions.
struct FitnessWeights {
    double b1;    // weight for path length (F1)
    double b2;    // weight for obstacle/threat cost (F2)
    double b3;    // weight for altitude deviation (F3)
    double b4;    // weight for path smoothness (F4)
    double a1;    // weight for horizontal turning angle inside F4
    double a2;    // weight for vertical climb-angle variation inside F4
    double hMin;  // minimum allowed flight altitude
    double hMax;  // maximum allowed flight altitude
};

// Multi-objective fitness function for UAV path evaluation.
// Combines four criteria: path length, obstacle proximity, altitude, and smoothness.
// Returns infinity for any path that collides with an obstacle or violates altitude limits.
class FitnessFunction {
public:
    FitnessFunction(const std::vector<std::shared_ptr<Obstacle>>& obstacles,
                    const FitnessWeights& weights = FitnessWeights{});

    // Computes the weighted sum b1*F1 + b2*F2 + b3*F3 + b4*F4.
    // Evaluates F2 and F3 first and short-circuits to infinity on constraint violation.
    double evaluate(const PointsList& path) const;

private:
    double f1_pathLength  (const PointsList& path) const; // total Euclidean path length
    double f2_threatCost  (const PointsList& path) const; // cumulative obstacle penalty
    double f3_altitudeCost(const PointsList& path) const; // deviation from mid-altitude band
    double f4_smoothness  (const PointsList& path) const; // turning and climb-angle variation

    std::vector<std::shared_ptr<Obstacle>> obstacles_;
    FitnessWeights w_;
};

#endif
