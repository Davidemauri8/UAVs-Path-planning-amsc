#ifndef LHS_HPP
#define LHS_HPP

#include <vector>
#include <random>
#include "../point.hpp"
#include "../pointsList.hpp"
#include "geometry/point.hpp"

// Latin Hypercube Sampling for generating an initial population of waypoints
class Lhs {
private:
    double xMin, xMax, yMin, yMax, zMin, zMax;
    int n;
    int idCluster;
    std::vector<double> xSamples;
    std::vector<double> ySamples;
    std::vector<double> zSamples;
    std::mt19937 gen;

    // Divides each axis into n equal intervals and draws one uniform random sample per interval
    void generatePartitions();

    // Independently shuffles the three sample vectors to pair axes randomly,
   void shuffleSamples();

    // Combines the three sample vectors into a PointsList.
    PointsList getSample();

public:
    Lhs(double xmin, double xmax, double ymin, double ymax, double zmin, double zmax, int n, int idCluster);

    // Clears internal buffers, generates partitions, shuffles, and returns a fresh population.
    PointsList generatePopulation();

    // Generates one population and packs the first NWaypoints points into a vector 
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
