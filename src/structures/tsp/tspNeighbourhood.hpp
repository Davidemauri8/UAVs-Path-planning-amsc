#ifndef TSP_NEIGHBOURHOOD_HPP
#define TSP_NEIGHBOURHOOD_HPP

#include "../pointsList.hpp"
#include <random>

// PointsList non è un point_nd — definiamo il nostro NeighbourhoodExtractor
// senza ereditare da quello del repo AMSC che è templatizzato su point_nd
class TspNeighbourhood {
public:
    using PointType = PointsList;

    PointsList current;
    PointsList prev;

    mutable std::mt19937 gen{std::random_device{}()};

    TspNeighbourhood(const PointsList& path)
        : current(path), prev(path) {}

    // aggiorna il centro del neighbourhood
    void from(const PointsList& p, const PointsList& c) {
        prev    = p;
        current = c;
    }

    // genera una nuova soluzione con 2-opt swap
    PointsList generateNext() const {
        PointsList next = current;
        int size = next.size();
        if (size < 2) return next;

        std::uniform_int_distribution<int> dist(0, size - 1);
        int i = dist(gen);
        int j = dist(gen);
        while (j == i) j = dist(gen);
        if (i > j) std::swap(i, j);

        // inverti il segmento tra i e j
        int lo = i, hi = j;
        while (lo < hi) {
            Point tmp = next.extractPoint(lo);
            next.replacePoint(lo, next.extractPoint(hi));
            next.replacePoint(hi, tmp);
            ++lo; --hi;
        }
        return next;
    }

    bool contains(const PointsList&) const {
        return true; // ogni permutazione è valida
    }
};

#endif