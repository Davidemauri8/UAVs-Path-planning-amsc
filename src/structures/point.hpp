#ifndef POINT_HPP
#define POINT_HPP

// 3D point with an optional cluster label
class Point {
    double x;
    double y;
    double z;
    int idCluster;

public:
    Point();
    Point(double x, double y, double z, int idCluster);

    double getX() const;
    double getY() const;
    double getZ() const;
    int    getCluster() const;

    // Euclidean distance to another point.
    double distance(const Point& other) const;

    // Returns the vector from other to this 
    Point  diff(const Point& other) const;

    // Euclidean norm of the point interpreted as a vector from the origin.
    double norm() const;

    // 2D cross product, used in the F4 smoothness cost
    double cross2D(const Point& other) const;

    void setX(double xNew);
    void setY(double yNew);
    void setZ(double zNew);
    void setCluster(int id);

    double operator*(const Point& other) const;  // 3D dot product
    Point  operator*(double scalar) const;       // scalar multiplication
    Point  operator+(const Point& other) const;
    Point  operator-(const Point& other) const;
};

#endif
