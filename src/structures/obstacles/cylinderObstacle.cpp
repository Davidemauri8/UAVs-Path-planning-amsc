#include "cylinderObstacle.hpp"
#include <cmath>
#include <algorithm>
#include <limits>

CylinderObstacle::CylinderObstacle(Point c, double r, double h, double b)
    : center(c), radius(r), height(h), buffer(b) {}

double CylinderObstacle::distance(Drone* drone) const {
    if (drone->getPosition().getZ() > height)
        return 0.0;
    double dX = center.getX() - drone->getPosition().getX();
    double dY = center.getY() - drone->getPosition().getY();
    return std::sqrt(dX*dX + dY*dY);
}

double CylinderObstacle::segmentCost(const Point& A, const Point& B) const {
    Point v = B.diff(A);
    Point w = center.diff(A);

    double dot_vv = v * v;
    double t = 0.0;
    if (dot_vv > 1e-9)
        t = std::clamp((w * v) / dot_vv, 0.0, 1.0);

    Point close = A + (v * t);
    double d_k  = close.diff(center).norm();

    if (d_k > buffer + radius)
        return 0.0;
    else if (d_k <= radius)
        return std::numeric_limits<double>::infinity();
    else
        return (buffer + radius) - d_k;
}
