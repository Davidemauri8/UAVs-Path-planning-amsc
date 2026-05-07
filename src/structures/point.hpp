#ifndef POINT_HPP
#define POINT_HPP

class Point {
    double x;
    double y;
    double z;
    int idCluster;

public:
    Point();
    Point(double x, double y, double z, int idCluster);

    //Per selezionare le singole cordinate o il cluster di un punto
    //utilizziamo const per rendere queste variabili non modificabili
    double getX() const;
    double getY() const;
    double getZ() const;
    int    getCluster() const;

    //Distanza tra due punti
    double distance(const Point& other) const;

    //Differenza tra due punti
    Point  diff(const Point& other) const;

    //Norma
    double norm() const;

    //Moltiplicazione solo tra cordinate x e y di due punti(utile per F4)
    double cross2D(const Point& other) const;

    //Inserire nuove cordinate e cluster
    void setX(double xNew);
    void setY(double yNew);
    void setZ(double zNew);
    void setCluster(int id);

    //Operatori utili
    double operator*(const Point& other) const;
    Point  operator*(double scalar) const;
    Point  operator+(const Point& other) const;
    Point  operator-(const Point& other) const;
};

#endif
