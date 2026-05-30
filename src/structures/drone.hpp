#ifndef DRONE_HPP
#define DRONE_HPP

#include "point.hpp"

// Represents a UAV with a current 3D position and a physical footprint radius
class Drone{
    Point position;
    double radius;
public:
    Drone(Point position, double radius = 0.0);
    Point getPosition();
    double getRadius() const;

};

#endif
