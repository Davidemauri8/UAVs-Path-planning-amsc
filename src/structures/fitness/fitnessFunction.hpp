#ifndef FITNESS_FUNCTION_HPP
#define FITNESS_FUNCTION_HPP

#include "../pointsList.hpp"
#include "../obstacles/obstacle.hpp"
#include <vector>
#include <memory>
#include <limits>

// Weights and altitude bounds used to combine the four fitness sub-functions.
// All values are intentionally left at 0.0.
// Configure them in fitnessUtilities.cpp.
struct FitnessWeights {
    double b1   = 0.0; // weight for path length (F1)
    double b2   = 0.0; // weight for obstacle/threat cost (F2)
    double b3   = 0.0; // weight for altitude deviation (F3)
    double b4   = 0.0; // weight for path smoothness (F4)
    double a1   = 0.0; // weight for horizontal turning angle inside F4
    double a2   = 0.0; // weight for vertical climb-angle variation inside F4
    double hMin        = 0.0; // minimum allowed flight altitude
    double hMax        = 0.0; // maximum allowed flight altitude
    double droneRadius = 0.0; // physical footprint radius of the UAV (inflates obstacle boundaries)
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

    const std::vector<std::shared_ptr<Obstacle>>& getObstacles() const { return obstacles_; }

private:
    double f1_pathLength  (const PointsList& path) const; // total Euclidean path length
    double f2_threatCost  (const PointsList& path) const; // cumulative obstacle penalty
    double f3_altitudeCost(const PointsList& path) const; // deviation from mid-altitude band
    double f4_smoothness  (const PointsList& path) const; // turning and climb-angle variation

    std::vector<std::shared_ptr<Obstacle>> obstacles_;
    FitnessWeights w_;
};

#endif
