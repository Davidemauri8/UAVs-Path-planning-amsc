#ifndef FITNESS_FUNCTION_HPP
#define FITNESS_FUNCTION_HPP

#include "../pointsList.hpp"
#include "../obstacles/obstacle.hpp"
#include <vector>
#include <memory>
#include <limits>


struct FitnessWeights {
    double b1;    // peso distanza
    double b2;    // peso ostacoli
    double b3;    // peso altitudine
    double b4;    // peso smoothness
    double a1;    // peso angolo orizzontale in F4
    double a2;    // peso angolo verticale in F4
    double hMin;  // altitudine minima consentita
    double hMax;  // altitudine massima consentita
};

class FitnessFunction {
public:
    FitnessFunction(const std::vector<std::shared_ptr<Obstacle>>& obstacles,
                    const FitnessWeights& weights = FitnessWeights{});

    double evaluate(const PointsList& path) const;

private:
    double f1_pathLength  (const PointsList& path) const;
    double f2_threatCost  (const PointsList& path) const;
    double f3_altitudeCost(const PointsList& path) const;
    double f4_smoothness  (const PointsList& path) const;

    std::vector<std::shared_ptr<Obstacle>> obstacles_;
    FitnessWeights w_;
};

#endif