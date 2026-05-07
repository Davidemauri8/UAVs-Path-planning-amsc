#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include "structures/pointsList.hpp"
#include <cmath>
#include <random>

class Controller {
private:
    double C0;          // initial threshold
    int total_iters;    // Total iteration
    
    std::mt19937 gen;

    // calculate Euclidean distance 
    double calculateDistance(const PointsList& a, const PointsList& b);

public:
    Controller(double initial_C, int iterations);

    /**
     * aplly Disruption Operator
     * @param current actual solution (Xi)
     * @param best best solution (Xbest)
     * @param neighbor close solution to (Xj)
     * @param current_iter 
     */
    void applyDisruption(PointsList& current, const PointsList& best, const PointsList& neighbor, int current_iter);
};

#endif