#ifndef STASA_NEIGHBOURHOOD_HPP
#define STASA_NEIGHBOURHOOD_HPP

#include "structures/pointsList.hpp"
#include <array>
#include <random>
#include <vector>

// I 4 operatori STASA su PointsList a dimensione runtime.
// Interfaccia analoga a STASANeighbourhood<Dim> della libreria SA:
//   from(prev, curr)  → aggiorna il centro del neighbourhood
//   generateNext()    → sceglie un operatore a caso e genera il candidato
class STASANeighbourhoodDyn {
public:
    STASANeighbourhoodDyn(
        double eps_rot, double eps_trans, double eps_scale, double eps_axis,
        double xMin, double xMax,
        double yMin, double yMax,
        double zMin, double zMax,
        unsigned seed = 42
    );

    void      from(const PointsList& prev, const PointsList& curr);
    PointsList generateNext();
    std::array<PointsList, 4> generateAll();
    PointsList clampToBounds(const PointsList& path) const;

private:
    PointsList applyRotation();      // eq. (10)
    PointsList applyTranslation();   // eq. (11)
    PointsList applyScaling();       // eq. (12)
    PointsList applyAxisTransf();    // eq. (13)

    std::vector<double> flatten  (const PointsList& path) const;
    PointsList          unflatten(const std::vector<double>& v) const;

    double eps_rot_, eps_trans_, eps_scale_, eps_axis_;
    double xMin_, xMax_, yMin_, yMax_, zMin_, zMax_;
    PointsList x_k_, x_prev_;
    std::mt19937 rng_;
};

#endif
