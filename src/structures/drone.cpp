#include "drone.hpp"

// can be define the dimension of the drone
Drone::Drone(Point position, double radius)
    : position(position), radius(radius) {}

Point Drone::getPosition(){
    return this->position;
}

double Drone::getRadius() const {
    return radius;
}
// review
