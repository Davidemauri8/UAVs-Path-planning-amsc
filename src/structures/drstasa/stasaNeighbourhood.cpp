#include "structures/drstasa/stasaNeighbourhood.hpp"
#include <cmath>

STASANeighbourhoodDyn::STASANeighbourhoodDyn(
    double eps_rot, double eps_trans, double eps_scale, double eps_axis,
    double xMin, double xMax, double yMin, double yMax, double zMin, double zMax,
    unsigned seed)
    : eps_rot_(eps_rot), eps_trans_(eps_trans), eps_scale_(eps_scale), eps_axis_(eps_axis),
      xMin_(xMin), xMax_(xMax), yMin_(yMin), yMax_(yMax), zMin_(zMin), zMax_(zMax),
      rng_(seed) {}

// Helpers
std::vector<double> STASANeighbourhoodDyn::flatten(const PointsList& path) const {
    int N = path.size();
    std::vector<double> v(N * 3);
    for (int i = 0; i < N; ++i) {
        v[i*3+0] = path.getX(i);
        v[i*3+1] = path.getY(i);
        v[i*3+2] = path.getZ(i);
    }
    return v;
}

PointsList STASANeighbourhoodDyn::unflatten(const std::vector<double>& v) const {
    int N = static_cast<int>(v.size()) / 3;
    PointsList path;
    for (int i = 0; i < N; ++i)
        path.addPoint(Point(v[i*3+0], v[i*3+1], v[i*3+2], -1));
    return path;
}

PointsList STASANeighbourhoodDyn::clampToBounds(const PointsList& path) const {
    int N = path.size();
    PointsList result;
    for (int i = 0; i < N; ++i) {
        double x = std::max(xMin_, std::min(xMax_, path.getX(i)));
        double y = std::max(yMin_, std::min(yMax_, path.getY(i)));
        double z = std::max(zMin_, std::min(zMax_, path.getZ(i)));
        result.addPoint(Point(x, y, z, -1));
    }
    return result;
}

// Public interface 
void STASANeighbourhoodDyn::from(const PointsList& prev, const PointsList& curr) {
    x_prev_ = prev;
    x_k_    = curr;
}

PointsList STASANeighbourhoodDyn::generateNext() {
    std::uniform_int_distribution<> opDist(0, 3);
    switch (opDist(rng_)) {
        case 0:  return applyRotation();
        case 1:  return applyTranslation();
        case 2:  return applyScaling();
        default: return applyAxisTransf();
    }
}

std::array<PointsList, 4> STASANeighbourhoodDyn::generateAll() {
    return { applyRotation(), applyTranslation(), applyScaling(), applyAxisTransf() };
}

// Four STASA operators

// explores a hypersphere centred on the current point
PointsList STASANeighbourhoodDyn::applyRotation() {
    std::vector<double> x = flatten(x_k_);
    int D = static_cast<int>(x.size());

    double norm = 0.0;
    for (double v : x) norm += v * v;
    norm = std::sqrt(norm) + 1e-9;

    std::uniform_real_distribution<> r(-1.0, 1.0);
    std::vector<double> result(D);
    for (int i = 0; i < D; ++i) {
        double dot = 0.0;
        for (int j = 0; j < D; ++j) dot += r(rng_) * x[j];
        result[i] = x[i] + eps_rot_ * dot / (D * norm);
    }
    return clampToBounds(unflatten(result));
}

// local search along the direction of the previous step
PointsList STASANeighbourhoodDyn::applyTranslation() {
    std::vector<double> x    = flatten(x_k_);
    std::vector<double> xPre = flatten(x_prev_);
    int D = static_cast<int>(x.size());

    double norm = 0.0;
    for (int i = 0; i < D; ++i) {
        double d = x[i] - xPre[i];
        norm += d * d;
    }
    norm = std::sqrt(norm) + 1e-9;

    std::uniform_real_distribution<> r(0.0, 1.0);
    double Rt = r(rng_);
    std::vector<double> result(D);
    for (int i = 0; i < D; ++i)
        result[i] = x[i] + eps_trans_ * Rt * (x[i] - xPre[i]) / norm;

    return clampToBounds(unflatten(result));
}

// global multiplicative Gaussian perturbation
PointsList STASANeighbourhoodDyn::applyScaling() {
    std::vector<double> x = flatten(x_k_);
    int D = static_cast<int>(x.size());

    std::normal_distribution<> gauss(0.0, 1.0);
    std::vector<double> result(D);
    for (int i = 0; i < D; ++i)
        result[i] = x[i] * (1.0 + eps_scale_ * gauss(rng_));

    return clampToBounds(unflatten(result));
}

// perturbs a single randomly chosen coordinate
PointsList STASANeighbourhoodDyn::applyAxisTransf() {
    std::vector<double> x = flatten(x_k_);
    int D = static_cast<int>(x.size());

    std::uniform_int_distribution<> idx(0, D - 1);
    std::normal_distribution<> gauss(0.0, 1.0);
    int k = idx(rng_);
    x[k] *= (1.0 + eps_axis_ * gauss(rng_));

    return clampToBounds(unflatten(x));
}
