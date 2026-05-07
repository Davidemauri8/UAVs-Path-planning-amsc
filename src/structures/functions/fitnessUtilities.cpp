#include "fitnessUtilities.hpp"
#include <algorithm>
#include <cmath>
#include <memory>


FitnessWeights sampleFitnessWeights(double zMin, double zMax) {
    FitnessWeights weights;
    weights.b1   = 5.0;  // Ostacoli
    weights.b2   = 10.0; // Lunghezza
    weights.b3   = 1.0;  // Curvatura
    weights.b4   = 5.0;  // Smoothness
    weights.a1   = 1.0;
    weights.a2   = 1.0;
    weights.hMin = zMin;
    weights.hMax = zMax;
    return weights;
}

FitnessFunction sampleFitnessFunction(double zMin, double zMax) {
    std::vector<std::shared_ptr<Obstacle>> obstacles;
    obstacles.push_back(std::make_shared<CylinderObstacle>(
        Point(250.0, 350.0, 0.0, -1), 40.0, 200.0, 50.0));
    obstacles.push_back(std::make_shared<CylinderObstacle>(
        Point(350.0, 450.0, 0.0, -1), 40.0, 200.0, 50.0));

    FitnessWeights weights = sampleFitnessWeights(zMin, zMax);

    // Restituiamo l'oggetto FitnessFunction
    return FitnessFunction(obstacles, weights);
}

DRSTASA::Config GetConfigurationDRST(int NWaypoints) {
    DRSTASA::Config drsCfg;
    
    // Inizializzazione basata sui campi definiti in drstasa.hpp
    drsCfg.popSize    = 20;
    drsCfg.maxIter    = 300;
    drsCfg.T0         = 100.0;
    drsCfg.alpha      = 0.93;
    drsCfg.p          = 0.5;
    drsCfg.C0         = 2.0;
    drsCfg.eps_rot    = 150.0;
    drsCfg.eps_trans  = 100.0;
    drsCfg.eps_scale  = 0.05;
    drsCfg.eps_axis   = 0.05;
      
    drsCfg.nWaypoints = NWaypoints;

    return drsCfg;
}


FitnessFunction makeDefaultFitness(double zMin, double zMax) {
    std::vector<std::shared_ptr<Obstacle>> obstacles;
    obstacles.push_back(std::make_shared<CylinderObstacle>(
        Point(200.0, 300.0, 0.0, -1), 50.0, 200.0, 60.0));
    obstacles.push_back(std::make_shared<CylinderObstacle>(
        Point(-300.0, -200.0, 0.0, -1), 50.0, 200.0, 60.0));
    return FitnessFunction(obstacles, sampleFitnessWeights(zMin, zMax));
}

void convertToGPS(PointsList& path, double lon0, double lat0, double metersPerDegLon, double metersPerDegLat) {
    for (int i = 0; i < path.size(); ++i) {
        // Supponendo che extractPoint restituisca un riferimento a un oggetto Point
        Point& p = path.extractPoint(i); 
        p.setX(lon0 + p.getX() / metersPerDegLon);
        p.setY(lat0 + p.getY() / metersPerDegLat);
    }
}