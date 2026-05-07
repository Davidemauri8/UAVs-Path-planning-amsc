#ifndef FITNESS_UTILITIES_HPP
#define FITNESS_UTILITIES_HPP

#include <vector>
#include "../pointsList.hpp"
#include "../obstacles/cylinderObstacle.hpp"
#include "../obstacles/obstacle.hpp"
#include "../fitness/fitnessFunction.hpp"
#include "structures/drstasa/drstasa.hpp"

// Returns a FitnessWeights struct with default tuning values for the given altitude band.
FitnessWeights sampleFitnessWeights(double zMin, double zMax);

// Returns a FitnessFunction with two sample cylinder obstacles and default weights.
// Useful for quick testing and benchmarking.
FitnessFunction sampleFitnessFunction(double zMin, double zMax);

// Returns a FitnessFunction with two default cylinder obstacles for the main pipeline.
FitnessFunction makeDefaultFitness(double zMin, double zMax);

// Builds a DRSTASA::Config tuned for the given number of intermediate waypoints.
DRSTASA::Config GetConfigurationDRST(int NWaypoints);

// Converts all points in path from local metric coordinates (metres) to GPS (lon/lat).
// lon0/lat0: origin of the local frame; metersPerDegLon/Lat: conversion factors.
void convertToGPS(PointsList& path, double lon0, double lat0, double metersPerDegLon, double metersPerDegLat);

#endif
