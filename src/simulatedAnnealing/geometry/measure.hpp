#pragma once
#ifndef GEOMETRY_DISTANCE_HPP
#define GEOMETRY_DISTANCE_HPP

#include "point_base.hpp"
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

    // Overload for discrete grid of points (the euclidena measure still makes sense)
    template <domain_dim Dim>
    double euclidean_distance(const lattice_nd<Dim>& p1, const lattice_nd<Dim>& p2) {
        // While the lattice point should allow for an euclidean (non-standard) norm over the 
        // grid; we use the neat .l2_norm() interface.
        return (p1 - p2).l2_norm();
    }

    template <domain_dim Dim>
    double manhattan_distance(const lattice_nd<Dim>& l1, const lattice_nd<Dim>& l2) {
        // The manhattan distance computes the distance in the lattice grid component-wise as in
        // https://en.wikipedia.org/wiki/Taxicab_geometry
        return (l1 - l2).l1_norm();
    }

}

namespace Conversion {

    template <typename PointType>
    auto make_continuous(const PointType& pt) -> point_nd<PointType::Dim>{
        constexpr const domain_dim Dim = PointType::Dim;
        point_nd<Dim> ret_val = 0.0;

        for (unsigned long i = 0; i < Dim; ++i) {
            // The pointType must implement the operator[] and must
            // be convertible to double
            ret_val[i] = (double) pt[i];
        }
        return ret_val;
    }

}

#endif