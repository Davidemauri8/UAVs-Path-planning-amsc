#ifndef LHS_HPP
#define LHS_HPP

#include <vector>
#include <random>
#include "../point.hpp"
#include "../pointsList.hpp"
#include "geometry/point.hpp"

class Lhs {
private:
    // Dati membri (ora chiaramente privati)
    double xMin, xMax, yMin, yMax, zMin, zMax;
    int n;
    int idCluster;
    std::vector<double> xSamples;
    std::vector<double> ySamples;
    std::vector<double> zSamples;
    std::mt19937 gen;

    //genera partizione per x y e z
    void generatePartitions();
    //faccio shuffle
    void shuffleSamples();
    //genero popolazione iniziale
    PointsList getSample();

public:
    // Costruttore
    Lhs(double xmin, double xmax, double ymin, double ymax, double zmin, double zmax, int n, int idCluster);

    PointsList generatePopulation();

    // converte il primo punto della popolazione in point_nd<NW*3>
    template <int NWaypoints>
    point_nd<NWaypoints * 3> toPointNd() {
        PointsList pop = generatePopulation();
        point_nd<NWaypoints * 3> result = 0.0;
        int count = std::min(NWaypoints, pop.size());
        for (int i = 0; i < count; ++i) {
            result[i * 3 + 0] = pop.getX(i);
            result[i * 3 + 1] = pop.getY(i);
            result[i * 3 + 2] = pop.getZ(i);
        }
        return result;
    }
};

#endif