#ifndef POINTLIST_HPP
#define POINTLIST_HPP

#include <vector>
#include "point.hpp"

// Ordered sequence of 3D points 
class PointsList {
    std::vector<Point> points;

public:
    PointsList();
    PointsList(std::vector<Point> points);

    int    size()          const;
    double getX(int i)     const;
    double getY(int i)     const;
    double getZ(int i)     const;

    // Sum of Euclidean distances between consecutive points
    double totalDistance() const;

    void   addPoint(Point p);

    // Returns a mutable reference so callers can update the point in-place
    Point& extractPoint(int i);

    void   replacePoint(int i, Point p);

    // Returns a new PointsList containing only points assigned to the given cluster id
    PointsList extractCluster(int numberCluster) const;

    double getYMin() const;
    double getXMin() const;
    double getYMax() const;
    double getXMax() const;

};

#endif
