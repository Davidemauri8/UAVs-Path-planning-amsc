#include "structures/drstasa/reverseLearnStrategy.hpp"
#include <algorithm>
#include <cmath>

ReverseLearnStrategy::ReverseLearnStrategy(
    double xMin, double xMax, double yMin, double yMax, double zMin, double zMax,
    unsigned seed)
    : xMin_(xMin), xMax_(xMax), yMin_(yMin), yMax_(yMax), zMin_(zMin), zMax_(zMax),
      rng_(seed) {}

// ─── Utility ─────────────────────────────────────────────────────────────────

std::vector<double> ReverseLearnStrategy::flatten(const PointsList& path) const {
    int N = path.size();
    std::vector<double> v(N * 3);
    for (int i = 0; i < N; ++i) {
        v[i*3+0] = path.getX(i);
        v[i*3+1] = path.getY(i);
        v[i*3+2] = path.getZ(i);
    }
    return v;
}

PointsList ReverseLearnStrategy::unflatten(const std::vector<double>& v) const {
    int N = static_cast<int>(v.size()) / 3;
    PointsList path;
    for (int i = 0; i < N; ++i)
        path.addPoint(Point(v[i*3+0], v[i*3+1], v[i*3+2], -1));
    return path;
}

PointsList ReverseLearnStrategy::clampToBounds(const PointsList& path) const {
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

// ─── Algoritmo ───────────────────────────────────────────────────────────────

void ReverseLearnStrategy::apply(std::vector<PointsList>& pop,
                                  std::vector<double>&    fitVals,
                                  std::function<double(const PointsList&)> evalFn,
                                  int nWaypoints) {
    int N = static_cast<int>(pop.size());
    int D = nWaypoints * 3;

    // Min e max per dimensione sull'intera popolazione (eq. 20: a_i(t), b_i(t))
    std::vector<double> dimMin(D,  1e18);
    std::vector<double> dimMax(D, -1e18);
    for (int i = 0; i < N; ++i) {
        std::vector<double> x = flatten(pop[i]);
        for (int d = 0; d < D; ++d) {
            dimMin[d] = std::min(dimMin[d], x[d]);
            dimMax[d] = std::max(dimMax[d], x[d]);
        }
    }

    std::uniform_real_distribution<> uni(0.0, 1.0);
    for (int i = 0; i < N; ++i) {
        std::vector<double> x  = flatten(pop[i]);
        double r5 = uni(rng_);
        double r6 = uni(rng_);

        // Punto reverse dinamico (eq. 20)
        std::vector<double> xBar(D);
        for (int d = 0; d < D; ++d)
            xBar[d] = r5 * (dimMin[d] + dimMax[d] - x[d]);

        // Blending con soluzione originale (eq. 21)
        std::vector<double> xNew(D);
        for (int d = 0; d < D; ++d)
            xNew[d] = (1.0 - r6) * x[d] + r6 * xBar[d];

        PointsList candidate = clampToBounds(unflatten(xNew));
        double newFit = evalFn(candidate);
        if (newFit < fitVals[i]) {
            pop[i]     = candidate;
            fitVals[i] = newFit;
        }
    }
}
