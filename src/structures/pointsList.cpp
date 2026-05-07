#include "pointsList.hpp"
#include <cmath>

PointsList::PointsList() {}

//Creo e inizializzo in unica operazine con :
PointsList::PointsList(std::vector<Point> points)
    : points(points) {}

int PointsList::size() const {
    return points.size();
}

double PointsList::getX(int i) const { return points[i].getX(); }
double PointsList::getY(int i) const { return points[i].getY(); }
double PointsList::getZ(int i) const { return points[i].getZ(); }

double PointsList::totalDistance() const {
    double sum = 0.0;
    for (int i = 0; i < (int)points.size() - 1; ++i)
        sum += points[i].distance(points[i + 1]);
    return sum;
}

void PointsList::addPoint(Point p) {
    points.push_back(p);
}

Point& PointsList::extractPoint(int i) {
    return points[i];
}

void PointsList::replacePoint(int i, Point p) {
    if (i >= 0 && i < (int)points.size())
        points[i] = p;
}

PointsList PointsList::extractCluster(int numberCluster) const {
    PointsList cluster;
    for (const Point& p : points)
        if (p.getCluster() == numberCluster)
            cluster.addPoint(p);
    return cluster;
}

double PointsList::getXMin() const {
    if (points.empty()) return 0.0;
    double xMin = points[0].getX();
    for (int i = 1; i < (int)points.size(); ++i)
        if (points[i].getX() < xMin) xMin = points[i].getX();
    return xMin;
}

double PointsList::getYMin() const {
    if (points.empty()) return 0.0;
    double yMin = points[0].getY();
    for (int i = 1; i < (int)points.size(); ++i)
        if (points[i].getY() < yMin) yMin = points[i].getY();
    return yMin;
}

double PointsList::getXMax() const {
    if (points.empty()) return 0.0;
    double xMax = points[0].getX();
    for (int i = 1; i < (int)points.size(); ++i)
        if (points[i].getX() > xMax) xMax = points[i].getX();
    return xMax;
}

double PointsList::getYMax() const {
    if (points.empty()) return 0.0;
    double yMax = points[0].getY();
    for (int i = 1; i < (int)points.size(); ++i)
        if (points[i].getY() > yMax) yMax = points[i].getY();
    return yMax;
}

