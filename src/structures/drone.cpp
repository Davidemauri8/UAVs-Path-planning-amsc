#include "drone.hpp"

Drone::Drone(Point position){
    this->position=position; 
}
Point Drone::getPosition(){
    return this->position;
}
