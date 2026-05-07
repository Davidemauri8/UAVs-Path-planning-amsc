#ifndef DRONE_HPP
#define DRONE_HPP

#include "point.hpp"

class Drone{
    Point position;
public:
    Drone(Point position);
    Point getPosition();
    
};

#endif