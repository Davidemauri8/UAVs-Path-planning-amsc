#ifndef TSP_SAMPLER_HPP
#define TSP_SAMPLER_HPP

#include "tspNeighbourhood.hpp"

class TspSampler {
public:
    using Neighbourhood = TspNeighbourhood;

    // max_it: numero massimo di tentativi per trovare soluzione valida
    PointsList sample(const TspNeighbourhood& n, int max_it = 1000) {
        return n.generateNext();  // ogni permutazione è valida → sempre accettata
    }

    void seed(long s) {
        // non usiamo seed esterno — il gen è dentro TspNeighbourhood
    }
};

#endif