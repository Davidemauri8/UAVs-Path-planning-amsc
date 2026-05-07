#ifndef FITNESS_UTILITIES_HPP
#define FITNESS_UTILITIES_HPP


#include <vector>
#include "../pointsList.hpp"
#include "../obstacles/cylinderObstacle.hpp"
#include "../obstacles/obstacle.hpp"
#include "../fitness/fitnessFunction.hpp"
#include "structures/drstasa/drstasa.hpp"

// Restituisce la struttura dei pesi
FitnessWeights sampleFitnessWeights(double zMin, double zMax);

// Restituisce la funzione di fitness completa
FitnessFunction sampleFitnessFunction(double zMin, double zMax);

/// Crea FitnessFunction con ostacoli e pesi di default per il test principale
FitnessFunction makeDefaultFitness(double zMin, double zMax);

DRSTASA::Config GetConfigurationDRST(int NWaypoints);

void convertToGPS(PointsList& path, double lon0, double lat0, double metersPerDegLon, double metersPerDegLat);




#endif