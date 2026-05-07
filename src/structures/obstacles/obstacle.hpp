#ifndef OBSTACLE_HPP
#define OBSTACLE_HPP

#include "../point.hpp"
#include "../drone.hpp"

class Obstacle {
public:
    Obstacle() {}
    virtual ~Obstacle() {}

    virtual double distance(Drone* drone) const = 0;
    virtual double segmentCost(const Point& A, const Point& B) const = 0;
};

#endif