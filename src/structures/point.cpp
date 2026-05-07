#include "point.hpp"
#include <cmath>

//Creo e inizializzo direttamete risparmiando un'operazione
Point::Point() : x(0), y(0), z(0), idCluster(-1) {}

Point::Point(double x, double y, double z, int idCluster)
    : x(x), y(y), z(z), idCluster(idCluster) {}

double Point::getX()       const { return x; }
double Point::getY()       const { return y; }
double Point::getZ()       const { return z; }
int    Point::getCluster() const { return idCluster; }

double Point::distance(const Point& other) const {
    double dX = x - other.x;
    double dY = y - other.y;
    double dZ = z - other.z;
    return std::sqrt(dX*dX + dY*dY + dZ*dZ);
}

Point Point::diff(const Point& other) const {
    return Point(x - other.x, y - other.y, z - other.z, 0);
}

double Point::norm() const {
    return std::sqrt(x*x + y*y + z*z);
}

double Point::cross2D(const Point& other) const {
    return x * other.y - y * other.x;
}

void Point::setX(double xNew)    { x = xNew; }
void Point::setY(double yNew)    { y = yNew; }
void Point::setZ(double zNew)    { z = zNew; }
void Point::setCluster(int id)   { idCluster = id; }

double Point::operator*(const Point& other) const {
    return x*other.x + y*other.y + z*other.z;
}

Point Point::operator*(double scalar) const {
    return Point(x*scalar, y*scalar, z*scalar, idCluster);
}

Point Point::operator+(const Point& other) const {
    return Point(x+other.x, y+other.y, z+other.z, idCluster);
}

Point Point::operator-(const Point& other) const {
    return Point(x-other.x, y-other.y, z-other.z, idCluster);
}