#ifndef DRONE_HPP
#define DRONE_HPP

#include "point.hpp"

// Represents a UAV with a current 3D position.
// Used by obstacle classes to query the drone's location for collision checks.
class Drone{
    Point position;
public:
    Drone(Point position);
    Point getPosition();

};

#endif
