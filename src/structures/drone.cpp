#include "drone.hpp"

Drone::Drone(Point position, double radius)
    : position(position), radius(radius) {}

Point Drone::getPosition(){
    return this->position;
}

double Drone::getRadius() const {
    return radius;
}
