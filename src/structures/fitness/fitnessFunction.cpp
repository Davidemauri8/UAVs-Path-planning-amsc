#include "fitnessFunction.hpp"
#include <cmath>

static const double INF = std::numeric_limits<double>::infinity();

FitnessFunction::FitnessFunction(const std::vector<std::shared_ptr<Obstacle>>& obstacles,
                                 const FitnessWeights& weights)
    : obstacles_(obstacles), w_(weights) {}

// F1: total Euclidean length of the path.
double FitnessFunction::f1_pathLength(const PointsList& path) const {
    return path.totalDistance();
}

// F2: cumulative obstacle penalty across all segments and all obstacles.
// Returns infinity immediately if any segment causes a hard collision.
double FitnessFunction::f2_threatCost(const PointsList& path) const {
    double total = 0.0;

    for (int i = 0; i + 1 < path.size(); ++i) {
        Point A(path.getX(i),   path.getY(i),   path.getZ(i),   -1);
        Point B(path.getX(i+1), path.getY(i+1), path.getZ(i+1), -1);

        for (const auto& obs : obstacles_) {
            double cost = obs->segmentCost(A, B, w_.droneRadius);
            if (cost == INF) return INF;
            total += cost;
        }
    }
    return total;
}

// F3: penalises altitude deviation from the midpoint of [hMin, hMax].
// Returns infinity if any waypoint falls outside the allowed altitude band.
double FitnessFunction::f3_altitudeCost(const PointsList& path) const {
    double total = 0.0;
    double hMid  = (w_.hMin + w_.hMax) / 2.0;

    for (int i = 0; i < path.size(); ++i) {
        double h = path.getZ(i);

        if (h < w_.hMin || h > w_.hMax)
            return INF; // out of range: path is illegal

        total += std::abs(h - hMid);
    }
    return total;
}

// F4: path smoothness measured as weighted sum of horizontal turning angles (phi)
// and changes in vertical climb angle (psi) across consecutive segment triplets.
double FitnessFunction::f4_smoothness(const PointsList& path) const {
    if (path.size() < 3) return 0.0;

    double sumPhi  = 0.0;
    double sumPsi  = 0.0;
    double prevPsi = 0.0;

    for (int i = 0; i + 2 < path.size(); ++i) {
        Point p0(path.getX(i),   path.getY(i),   path.getZ(i),   -1);
        Point p1(path.getX(i+1), path.getY(i+1), path.getZ(i+1), -1);
        Point p2(path.getX(i+2), path.getY(i+2), path.getZ(i+2), -1);

        Point v1 = p1 - p0; // direction vector of the first segment
        Point v2 = p2 - p1; // direction vector of the second segment

        double n1 = v1.norm();
        double n2 = v2.norm();
        if (n1 < 1e-9 || n2 < 1e-9) continue; // skip overlapping waypoints

        // phi: horizontal turning angle between v1 and v2.
        double cross = v1.cross2D(v2);
        double phi   = std::atan(std::abs(cross) / (n1 * n2));
        sumPhi += phi;

        // psi: climb angle of the second segment.
        double dz  = p2.getZ() - p1.getZ();
        double dxy = std::sqrt(v2.getX() * v2.getX() + v2.getY() * v2.getY());
        double psi;
        if (dxy > 1e-9)
            psi = std::atan(dz / dxy);
        else
            psi = 0.0;

        // Accumulate the change in climb angle, not the absolute angle.
        if (i > 0) sumPsi += std::abs(psi - prevPsi);
        prevPsi = psi;
    }

    return w_.a1 * sumPhi + w_.a2 * sumPsi;
}

// evaluate: assembles the total fitness score.
// F2 and F3 are evaluated first to short-circuit on illegal paths before
// computing the more expensive F1 and F4.
double FitnessFunction::evaluate(const PointsList& path) const {
    if (path.size() < 2) return INF;

    double f2 = f2_threatCost(path);
    if (f2 == INF) return INF;

    double f3 = f3_altitudeCost(path);
    if (f3 == INF) return INF;

    double f1 = f1_pathLength(path);
    double f4 = f4_smoothness(path);

    return w_.b1 * f1 + w_.b2 * f2 + w_.b3 * f3 + w_.b4 * f4;
}
