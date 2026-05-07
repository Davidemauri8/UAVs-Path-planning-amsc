#ifndef LHS_HPP
#define LHS_HPP

#include <vector>
#include <random>
#include "../point.hpp"
#include "../pointsList.hpp"
#include "geometry/point.hpp"

// Latin Hypercube Sampling (LHS) for generating an initial population of waypoints.
// The 3D search space [xMin,xMax] x [yMin,yMax] x [zMin,zMax] is divided into n equal
// intervals per axis; one sample is drawn from each interval and the three axes are
// independently shuffled to ensure uniform coverage.
class Lhs {
private:
    double xMin, xMax, yMin, yMax, zMin, zMax;
    int n;
    int idCluster;
    std::vector<double> xSamples;
    std::vector<double> ySamples;
    std::vector<double> zSamples;
    std::mt19937 gen;

    // Divides each axis into n equal intervals and draws one uniform random
    // sample per interval, storing results in xSamples/ySamples/zSamples.
    void generatePartitions();

    // Independently shuffles the three sample vectors to pair axes randomly,
    // satisfying the Latin Hypercube property.
    void shuffleSamples();

    // Combines the three sample vectors into a PointsList.
    PointsList getSample();

public:
    Lhs(double xmin, double xmax, double ymin, double ymax, double zmin, double zmax, int n, int idCluster);

    // Clears internal buffers, generates partitions, shuffles, and returns a fresh population.
    PointsList generatePopulation();

    // Generates one population and packs the first NWaypoints points into a
    // flat point_nd<NWaypoints*3> vector for use with the SA library.
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
