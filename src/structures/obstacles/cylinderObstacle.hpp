#ifndef CYLINDER_OBSTACLE_HPP
#define CYLINDER_OBSTACLE_HPP

#include "obstacle.hpp"

class CylinderObstacle: public Obstacle{
    Point center;   // z of the center must be zero 
    double radius;
    double height;
    double buffer;  // soft penalty zone thickness outside the hard radius

public:

    CylinderObstacle(Point c, double r, double h, double b);

    double getX()      const { return center.getX(); }
    double getY()      const { return center.getY(); }
    double getRadius() const { return radius; }
    double getHeight() const { return height; }
    double getBuffer() const { return buffer; }

    // Returns the horizontal distance from the drone to the cylinder center
    double distance(Drone* drone) const;

    // Computes the penalty cost for segment AB, inflating the collision radius by droneRadius
    double segmentCost(const Point& A, const Point& B, double droneRadius = 0.0) const override;
};

#endif
