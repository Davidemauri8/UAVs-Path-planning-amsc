#pragma once

#include "structures/fitness/fitnessFunction.hpp"
#include "structures/obstacles/cylinderObstacle.hpp"
#include "structures/functions/fitnessUtilities.hpp"
#include "structures/geo/geoUtils.hpp"
#include "structures/point.hpp"
#include <memory>
#include <vector>

// Reference origin: Keflavik Airport (63.985°N, 22.605°W).
// All obstacle metric coordinates below are referenced to this origin.
inline GeoOrigin icelandOrigin() { return {63.985, -22.605}; }

// Four volcanic no-fly zones.
// CylinderObstacle args: centre (x,y), radius_m, height_m, zBase_m
inline FitnessFunction makeIcelandFitness(double zMin, double zMax)
{
    std::vector<std::shared_ptr<Obstacle>> obs;

    auto add = [&](double x, double y, double r, double height, double zBase) {
        obs.push_back(std::make_shared<CylinderObstacle>(
            Point(x, y, 0.0, -1), r, height, zBase));
    };

    add( 16212,  -8880,  5000,  385, 2000); // Fagradalsfjall (eruzioni 2021-23)
    add(141480,   -555, 15000, 1491, 5000); // Hekla (più attivo d'Islanda)
    add(145715, -39405, 12000, 1651, 4000); // Eyjafjallajökull (eruzione 2010)
    add(284906, 192252, 10000,  818, 3000); // Krafla (adiacente Mývatn)

    return FitnessFunction(obs, sampleFitnessWeights(zMin, zMax));
}
