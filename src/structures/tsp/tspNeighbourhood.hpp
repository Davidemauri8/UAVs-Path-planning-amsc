#ifndef TSP_NEIGHBOURHOOD_HPP
#define TSP_NEIGHBOURHOOD_HPP

#include "../pointsList.hpp"
#include <random>

// Neighbourhood operator for TSP-style ordering of waypoints.
// PointsList is not a point_nd, so this class does not inherit from the AMSC
// library's templatised NeighbourhoodExtractor; it mirrors the same interface instead.
class TspNeighbourhood {
public:
    using PointType = PointsList;

    PointsList current;
    PointsList prev;

    mutable std::mt19937 gen{std::random_device{}()};

    TspNeighbourhood(const PointsList& path)
        : current(path), prev(path) {}

    // Updates the neighbourhood centre to a new current solution.
    void from(const PointsList& p, const PointsList& c) {
        prev    = p;
        current = c;
    }

    // Generates a neighbour by applying a random 2-opt swap:
    // two indices i < j are chosen and the sub-sequence between them is reversed.
    PointsList generateNext() const {
        PointsList next = current;
        int size = next.size();
        if (size < 2) return next;

        std::uniform_int_distribution<int> dist(0, size - 1);
        int i = dist(gen);
        int j = dist(gen);
        while (j == i) j = dist(gen);
        if (i > j) std::swap(i, j);

        int lo = i, hi = j;
        while (lo < hi) {
            Point tmp = next.extractPoint(lo);
            next.replacePoint(lo, next.extractPoint(hi));
            next.replacePoint(hi, tmp);
            ++lo; --hi;
        }
        return next;
    }

    // Every permutation of waypoints is a valid TSP solution.
    bool contains(const PointsList&) const {
        return true;
    }
};

#endif
