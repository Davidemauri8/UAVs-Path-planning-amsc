#include "fitnessUtilities.hpp"
#include <algorithm>
#include <cmath>
#include <memory>

// Builds a FitnessWeights with all operative values. Weight fields (b1..a2) are
// defined here and left at zero in the struct; only altitude bounds come from outside.
FitnessWeights sampleFitnessWeights(double zMin, double zMax) {
    FitnessWeights weights;
    // b1: length penalty doubled to create strong tension with obstacle avoidance
    weights.b1          = 12.0;

    // b2: obstacle weight doubled
    weights.b2          = 20.0;

    // b3: small increase; altitude constraint is a secondary selective pressure
    weights.b3          = 2.0;

    // b4: smoothness weight doubled
    weights.b4          = 10.0;

    // a1: stronger horizontal turn penalty
    weights.a1          = 2.5;

    // a2: moderate increase for vertical climb-angle variation
    weights.a2          = 1.5;
    weights.hMin        = zMin;
    weights.hMax        = zMax;

    // Larger footprint narrows the feasible corridors between obstacles
    weights.droneRadius = 3.0;
    return weights;
}


// All DRSTASA operative parameters are defined here
DRSTASA::Config GetConfigurationDRST(int NWaypoints) {
    DRSTASA::Config drsCfg;

    // popSize: larger population covers more corridors simultaneously
    drsCfg.popSize    = 30;
    drsCfg.maxIter    = 500;
    drsCfg.T0         = 100.0;

    // Slower cooling: each temperature level is exploited longer by the full population 
    drsCfg.alpha      = 0.96;

    // p: giving more chances to exploit opposite-side solutions around obstacles.
    drsCfg.p          = 0.4;

    // Stronger disruption to escape corridor-trapping local minima.
    drsCfg.C0         = 3.0;

    // Larger geometric perturbations so rotations and translations can reach
    // alternative corridors that are far from the current solution.
    drsCfg.eps_rot    = 180.0;
    drsCfg.eps_trans  = 150.0;
    drsCfg.eps_scale  = 0.08;
    drsCfg.eps_axis   = 0.08;

    drsCfg.nWaypoints = NWaypoints;

    return drsCfg;
}


std::vector<std::shared_ptr<Obstacle>> buildDefaultObstacles() {
    std::vector<std::shared_ptr<Obstacle>> obstacles;

    // C1
    // Reykjanes Ridge
    obstacles.push_back(std::make_shared<CylinderObstacle>(
        Point(-25922.9, 37805.7, 0.0, -1), 8000.0, 2000.0, 100.0));
 
    // Snæfellsjökull
    obstacles.push_back(std::make_shared<CylinderObstacle>(
        Point(-31531.5, 97295.0, 0.0, -1), 5000.0, 1500.0, 90.0));
 
    // C2
 
    // Vestfjarðahæðir 
    obstacles.push_back(std::make_shared<CylinderObstacle>(
        Point(-75108.3, 179579.3, 0.0, -1), 5000.0, 1500.0, 80.0));
 
    // Ísafjarðarfjöll 
    obstacles.push_back(std::make_shared<CylinderObstacle>(
        Point(-19021.8, 177911.3, 0.0, -1), 5000.0, 1500.0, 80.0));
 
    // Drangajökull 
    obstacles.push_back(std::make_shared<CylinderObstacle>(
        Point(26578.9, 201262.3, 0.0, -1), 6000.0, 1500.0, 90.0));
 
    // C3
    // Langjökull 
    obstacles.push_back(std::make_shared<CylinderObstacle>(
        Point(127290.6, -14455.9, 0.0, -1), 18000.0, 1500.0, 120.0));
 
    // Hrafntinnusker
    obstacles.push_back(std::make_shared<CylinderObstacle>(
        Point(174793.3, -11898.4, 0.0, -1), 3500.0, 1500.0, 70.0));
 
    // Katla / Mýrdalsjökull
    obstacles.push_back(std::make_shared<CylinderObstacle>(
        Point(187522.5, -56821.2, 0.0, -1), 10000.0, 1500.0, 140.0));
 
    // C0
    // Hverfjall 
    obstacles.push_back(std::make_shared<CylinderObstacle>(
        Point(246779.0, 182359.1, 0.0, -1), 2500.0, 1500.0, 60.0));
 
    // Krafla 
    obstacles.push_back(std::make_shared<CylinderObstacle>(
        Point(285795.7, 190698.7, 0.0, -1), 5000.0, 1500.0, 80.0));
 
    // Herðubreið 
    obstacles.push_back(std::make_shared<CylinderObstacle>(
        Point(319935.2, 195146.5, 0.0, -1), 3500.0, 1500.0, 100.0));
 
    // Askja / Dyngjufjöll 
    obstacles.push_back(std::make_shared<CylinderObstacle>(
        Point(371388.4, 195813.7, 0.0, -1), 8000.0, 1500.0, 120.0));
 
    // Snæfell
    obstacles.push_back(std::make_shared<CylinderObstacle>(
        Point(388458.2, 185806.2, 0.0, -1), 5000.0, 1500.0, 110.0));
 
    // Kverkfjöll
    obstacles.push_back(std::make_shared<CylinderObstacle>(
        Point(408942.0, 145108.8, 0.0, -1), 6000.0, 1500.0, 130.0));
 
    // Bárðarbunga 
    obstacles.push_back(std::make_shared<CylinderObstacle>(
        Point(430888.8, 113418.3, 0.0, -1), 9000.0, 1500.0, 150.0));
 
    // Öræfajökull 
    obstacles.push_back(std::make_shared<CylinderObstacle>(
        Point(401870.2, 95071.1, 0.0, -1), 5000.0, 1500.0, 120.0));
 
    // Hofsjökull 
    obstacles.push_back(std::make_shared<CylinderObstacle>(
        Point(372607.7, 72832.1, 0.0, -1), 14000.0, 1500.0, 130.0));
 
    // Eiríksjökull 
    obstacles.push_back(std::make_shared<CylinderObstacle>(
        Point(107050.7, 86620.3, 0.0, -1), 3500.0, 1500.0, 80.0));
 
    // Langjökull
    obstacles.push_back(std::make_shared<CylinderObstacle>(
        Point(122413.5, 73944.1, 0.0, -1), 17000.0, 1500.0, 120.0));
 
    // Þórisjökull
    obstacles.push_back(std::make_shared<CylinderObstacle>(
        Point(152163.7, 85397.1, 0.0, -1), 4000.0, 1500.0, 80.0));
 
    // Hofsjökull 
    obstacles.push_back(std::make_shared<CylinderObstacle>(
        Point(179963.0, 93848.0, 0.0, -1), 13000.0, 1500.0, 130.0));
 
    // Vatnajökull 
    obstacles.push_back(std::make_shared<CylinderObstacle>(
        Point(234586.3, 68384.3, 0.0, -1), 22000.0, 1500.0, 150.0)); 

    return obstacles;
}

FitnessFunction makeDefaultFitness(double zMin, double zMax) {
    return FitnessFunction(buildDefaultObstacles(), sampleFitnessWeights(zMin, zMax));
}

FitnessFunction makeDefaultFitness(double zMin, double zMax, int nObstacles) {
    auto obs = buildDefaultObstacles();
    if (nObstacles >= 0 && nObstacles < static_cast<int>(obs.size()))
        obs.resize(nObstacles);
    return FitnessFunction(obs, sampleFitnessWeights(zMin, zMax));
}

// Converts all waypoints in path from local metric coordinates to GPS.
void convertToGPS(PointsList& path, double lon0, double lat0, double metersPerDegLon, double metersPerDegLat) {
    for (int i = 0; i < path.size(); ++i) {
        Point& p = path.extractPoint(i);
        p.setX(lon0 + p.getX() / metersPerDegLon);
        p.setY(lat0 + p.getY() / metersPerDegLat);
    }
}
// review
