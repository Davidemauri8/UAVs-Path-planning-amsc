#ifndef FITNESS_UTILITIES_HPP
#define FITNESS_UTILITIES_HPP

#include <vector>
#include "../pointsList.hpp"
#include "../obstacles/cylinderObstacle.hpp"
#include "../obstacles/obstacle.hpp"
#include "../fitness/fitnessFunction.hpp"
#include "structures/drstasa/drstasa.hpp"

// Returns a FitnessWeights struct with default tuning values for the given altitude band
FitnessWeights sampleFitnessWeights(double zMin, double zMax);

// Returns all default obstacles as a vector 
std::vector<std::shared_ptr<Obstacle>> buildDefaultObstacles();

// Returns a FitnessFunction with all default obstacles
FitnessFunction makeDefaultFitness(double zMin, double zMax);

// Returns a FitnessFunction with only the first nObstacles obstacles
FitnessFunction makeDefaultFitness(double zMin, double zMax, int nObstacles);

// Builds a DRSTASA
DRSTASA::Config GetConfigurationDRST(int NWaypoints);

// Converts all points in path from local metric coordinates (metres) to GPS
void convertToGPS(PointsList& path, double lon0, double lat0, double metersPerDegLon, double metersPerDegLat);

#endif
