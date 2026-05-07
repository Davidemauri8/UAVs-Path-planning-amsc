#ifndef POINTLIST_HPP
#define POINTLIST_HPP

#include <vector>
#include "point.hpp"

class PointsList {
    std::vector<Point> points;

public:
    PointsList();
    PointsList(std::vector<Point> points);

    int    size()          const;
    double getX(int i)     const;
    double getY(int i)     const;
    double getZ(int i)     const;

    //Calcola la distanza totale tra tutti i punti della lista
    double totalDistance() const;

    //Aggiunge un punto alla lista
    void   addPoint(Point p);

    //Estrae un punto dalla lista
    Point& extractPoint(int i);

    //Sostituisce l'i-esimo punto nella lista
    void   replacePoint(int i, Point p);

    //Estrarre tutti i punti appartenenti ad un cluster
    PointsList extractCluster(int numberCluster) const;

    double getYMin() const;
    double getXMin() const;
    double getYMax() const;
    double getXMax() const;

};

#endif
