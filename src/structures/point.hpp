#ifndef POINT_HPP
#define POINT_HPP

// 3D point with an optional cluster label, used as the basic spatial unit
// throughout the path-planning pipeline.
class Point {
    double x;
    double y;
    double z;
    int idCluster;

public:
    Point();
    Point(double x, double y, double z, int idCluster);

    // Read-only accessors; declared const to allow use on const Point references.
    double getX() const;
    double getY() const;
    double getZ() const;
    int    getCluster() const;

    // Euclidean distance to another point.
    double distance(const Point& other) const;

    // Returns the vector from other to this (component-wise subtraction).
    Point  diff(const Point& other) const;

    // Euclidean norm of the point interpreted as a vector from the origin.
    double norm() const;

    // 2D cross product (x*other.y - y*other.x), used in the F4 smoothness cost
    // to measure the horizontal turning angle between two path segments.
    double cross2D(const Point& other) const;

    void setX(double xNew);
    void setY(double yNew);
    void setZ(double zNew);
    void setCluster(int id);

    double operator*(const Point& other) const; // 3D dot product
    Point  operator*(double scalar) const;       // scalar multiplication
    Point  operator+(const Point& other) const;
    Point  operator-(const Point& other) const;
};

#endif
