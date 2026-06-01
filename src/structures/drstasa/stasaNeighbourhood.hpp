#ifndef STASA_NEIGHBOURHOOD_HPP
#define STASA_NEIGHBOURHOOD_HPP

#include "structures/pointsList.hpp"
#include <array>
#include <random>
#include <vector>

// Runtime-sized implementation of the four STASA neighbourhood operators
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
    // Each operator accepts an explicit rng so generateAll() can call them
    // with independent per-operator rngs in the parallel path.
    PointsList applyRotation   (std::mt19937& rng);
    PointsList applyTranslation(std::mt19937& rng);
    PointsList applyScaling    (std::mt19937& rng);
    PointsList applyAxisTransf (std::mt19937& rng);

    // Helpers: flatten PointsList to a flat double vector and back
    std::vector<double> flatten  (const PointsList& path) const;
    PointsList          unflatten(const std::vector<double>& v) const;

    double eps_rot_, eps_trans_, eps_scale_, eps_axis_;
    double xMin_, xMax_, yMin_, yMax_, zMin_, zMax_;
    PointsList x_k_, x_prev_;
    std::mt19937 rng_;
};

#endif
