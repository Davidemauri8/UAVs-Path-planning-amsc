#ifndef OBSTACLE_HPP
#define OBSTACLE_HPP

#include "../point.hpp"
#include "../drone.hpp"

// Abstract base class for all obstacles in the environment
class Obstacle {
public:
    Obstacle() {}
    virtual ~Obstacle() {}

    // Returns the distance from the drone's current position to the obstacle surface.
    virtual double distance(Drone* drone) const = 0;

    // Returns the penalty cost for the path segment passing through or near this obstacle
    // Returns infinity if the segment intersects the obstacle body.
    virtual double segmentCost(const Point& A, const Point& B) const = 0;
};

#endif
