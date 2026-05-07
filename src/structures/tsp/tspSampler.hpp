#ifndef TSP_SAMPLER_HPP
#define TSP_SAMPLER_HPP

#include "tspNeighbourhood.hpp"

// Sampler for TspNeighbourhood
class TspSampler {
public:
    using Neighbourhood = TspNeighbourhood;

    PointsList sample(const TspNeighbourhood& n, int max_it = 1000) {
        return n.generateNext();
    }

    // Seeding is handled internally by TspNeighbourhood's own RNG
    void seed(long s) {}
};

#endif
