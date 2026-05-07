#ifndef CYLINDER_OBSTACLE_HPP
#define CYLINDER_OBSTACLE_HPP

#include "obstacle.hpp"

// Upright infinite cylinder obstacle defined by a center (z must be 0),
// a hard radius (collision zone), a height, and a soft buffer zone around the radius.
class CylinderObstacle: public Obstacle{
    Point center; // z of the center must be zero (cylinder axis is vertical)
    double radius;
    double height;
    double buffer; // soft penalty zone thickness outside the hard radius

public:
    // c: center point, r: collision radius, h: obstacle height, b: buffer zone thickness.
    CylinderObstacle(Point c, double r, double h, double b);

    // Returns the horizontal distance from the drone to the cylinder center.
    // Returns 0 if the drone is above the obstacle height.
    double distance(Drone* drone) const;

    // Computes the penalty cost for segment A→B.
    // Returns infinity if the closest point on the segment falls inside the hard radius,
    // a linear penalty proportional to penetration depth in the buffer zone, or 0 if clear.
    double segmentCost(const Point& A, const Point& B) const override;
};

#endif
