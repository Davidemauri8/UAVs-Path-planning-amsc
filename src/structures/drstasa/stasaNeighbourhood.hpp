#ifndef STASA_NEIGHBOURHOOD_HPP
#define STASA_NEIGHBOURHOOD_HPP

#include "structures/pointsList.hpp"
#include <array>
#include <random>
#include <vector>

// Runtime-sized implementation of the four STASA neighbourhood operators
// applied to a PointsList (path of intermediate waypoints).
// Mirrors the interface of STASANeighbourhood<Dim> from the SA library:
//   from(prev, curr)  — updates the neighbourhood centre
//   generateNext()    — picks one operator at random and returns a candidate
//   generateAll()     — evaluates all four operators and returns the results
class STASANeighbourhoodDyn {
public:
    STASANeighbourhoodDyn(
        double eps_rot, double eps_trans, double eps_scale, double eps_axis,
        double xMin, double xMax,
        double yMin, double yMax,
        double zMin, double zMax,
        unsigned seed = 42
    );

    void      from(const PointsList& prev, const PointsList& curr);
    PointsList generateNext();
    std::array<PointsList, 4> generateAll();

    // Clamps each point in path to the allowed bounding box.
    PointsList clampToBounds(const PointsList& path) const;

private:
    PointsList applyRotation();    // eq. (10): hypersphere exploration around current point
    PointsList applyTranslation(); // eq. (11): local search along the previous step direction
    PointsList applyScaling();     // eq. (12): global multiplicative Gaussian perturbation
    PointsList applyAxisTransf();  // eq. (13): single-coordinate Gaussian perturbation

    // Helpers: flatten PointsList to a flat double vector and back.
    std::vector<double> flatten  (const PointsList& path) const;
    PointsList          unflatten(const std::vector<double>& v) const;

    double eps_rot_, eps_trans_, eps_scale_, eps_axis_;
    double xMin_, xMax_, yMin_, yMax_, zMin_, zMax_;
    PointsList x_k_, x_prev_;
    std::mt19937 rng_;
};

#endif
