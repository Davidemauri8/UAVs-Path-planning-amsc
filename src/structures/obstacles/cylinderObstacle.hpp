#ifndef CYLINDER_OBSTACLE_HPP
#define CYLINDER_OBSTACLE_HPP

#include "obstacle.hpp"

class CylinderObstacle: public Obstacle{
    Point center; // z of the center must be zero 
    double radius;
    double height;
    double buffer; // soft penalty zone thickness outside the hard radius

public:
    // c: center point, r: collision radius, h: obstacle height, b: buffer zone thickness.
    CylinderObstacle(Point c, double r, double h, double b);

    // Returns the horizontal distance from the drone to the cylinder center.
    // Returns 0 if the drone is above the obstacle height.
    double distance(Drone* drone) const;

    // Computes the penalty cost for segment 
    double segmentCost(const Point& A, const Point& B) const override;
};

#endif
