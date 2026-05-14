#ifndef DRONE_HPP
#define DRONE_HPP

#include "point.hpp"

// Represents a UAV with a current 3D position and a physical footprint radius.
// The radius is used by obstacle classes to inflate collision boundaries (Minkowski sum).
class Drone{
    Point position;
    double radius;
public:
    Drone(Point position, double radius = 0.0);
    Point getPosition();
    double getRadius() const;

};

#endif
