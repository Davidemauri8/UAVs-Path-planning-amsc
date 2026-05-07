#ifndef CYLINDER_OBSTACLE_HPP
#define CYLINDER_OBSTACLE_HPP

#include "obstacle.hpp"
class CylinderObstacle: public Obstacle{
    Point center; //z of the center must be equal to zero
    double radius;
    double height;
    double buffer; //???

public:
    
    // Costruttore per inizializzare l'ostacolo
    CylinderObstacle(Point c, double r, double h, double b);
     
    //distanza drone(punto)-ostacolo(centro)
    double distance(Drone* drone) const;

    double segmentCost(const Point& A, const Point& B) const override;


};

#endif
