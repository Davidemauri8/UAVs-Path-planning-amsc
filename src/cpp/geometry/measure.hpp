#pragma once
#ifndef GEOMETRY_DISTANCE
#define GEOMETRY_DISTANCE

#include "combination.hpp"
#include "point.hpp"
#include "lattice_point.hpp"

typedef unsigned int domain_dim;


namespace Distance {

    template <domain_dim Dim>
    long hamming_distance(const combination_nd<Dim>& a, const combination_nd<Dim>& b) {
        // From https://dev.to/ggorantala/hamming-distance-kcm we get the
        // single-byte algorithm, apply it to the entire combination
        long distance = 0;
        // Notice that the function get_byte() already handles the edge case
        // by zeroing out non-used bits 
        for (int byte = 0; byte < a.byte_size() - 1; ++byte) {
            unsigned char xor_val = a.get_byte(byte) ^ b.get_byte(byte);
            while (xor_val ^ 0) {
                if (xor_val % 2 == 1)
                    distance += 1;
                xor_val >>= 1;
            }
        }
        return distance; 
    }

    template <domain_dim Dim>
    double euclidean_distance(const point_nd<Dim>& p1, const point_nd<Dim>& p2) {
        return (p1 - p2).norm();
    }

    template <domain_dim Dim>
    double manhattan_distance(const lattice_nd<Dim>& l1, const lattice_nd<Dim>& l2) {
        // The manhattan distance computes the distance in the lattice grid component-wise as in
        // https://en.wikipedia.org/wiki/Taxicab_geometry
        return (l1 - l2).l1_norm();
    }

}

#endif