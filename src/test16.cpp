#include "structures/pipeline/pipelineRunner.hpp"
#include "structures/geo/geoUtils.hpp"
#include "structures/functions/fitnessUtilities.hpp"
#include "structures/obstacles/cylinderObstacle.hpp"
#include "structures/kmeans.hpp"
#include "exporters/pointsListReader.hpp"
#include <iostream>
#include <fstream>
#include <omp.h>

// Same scenario as test14 so results are directly comparable.
static FitnessFunction makeLAquilaFitness(double zMin, double zMax) {
    std::vector<std::shared_ptr<Obstacle>> obs;
    obs.push_back(std::make_shared<CylinderObstacle>(Point(    0.0,  2500.0, 0.0, -1), 200.0, 180.0, 100.0));
    obs.push_back(std::make_shared<CylinderObstacle>(Point(-2800.0,   900.0, 0.0, -1), 250.0, 180.0, 100.0));
    obs.push_back(std::make_shared<CylinderObstacle>(Point( 2200.0,  -600.0, 0.0, -1), 220.0, 180.0,  90.0));
    obs.push_back(std::make_shared<CylinderObstacle>(Point(-1300.0, -2300.0, 0.0, -1), 200.0, 180.0,  80.0));
    obs.push_back(std::make_shared<CylinderObstacle>(Point( 3000.0,  3000.0, 0.0, -1), 180.0, 180.0,  70.0));
    obs.push_back(std::make_shared<CylinderObstacle>(Point(-4500.0, -1700.0, 0.0, -1), 240.0, 180.0, 100.0));
    return FitnessFunction(obs, sampleFitnessWeights(zMin, zMax));
}

int main() {
    // Strong-scaling benchmark: K fixed, sweep num_threads from 1 to maxThreads.
    // K=10 has enough clusters to show OMP speedup while keeping serial time measurable.
    constexpr int NWaypoints = 4;
    constexpr int K_FIXED    = 10;
    const double zMin = 90.0, zMax = 180.0;

    GeoOrigin origin{42.370, 13.400};
    PointsList allPointsBase = PointsListReader::readCSV("../data/laquila_survey_200.csv");
    if (allPointsBase.size() == 0) { std::cerr << "Errore: nessun punto letto.\n"; return 1; }
    GeoUtils::toMeters(allPointsBase, origin);

    FitnessFunction fitness = makeLAquilaFitness(zMin, zMax);
    const int maxThreads = omp_get_max_threads();
    const int nPoints    = allPointsBase.size();

    std::cout << "Strong-scaling benchmark (test16)\n"
              << "  K=" << K_FIXED << " (fixed)   N=" << nPoints
              << "   max_threads=" << maxThreads << "\n\n";

    // Cluster once; clustering is not part of the timed parallel kernel.
    PointsList pts = allPointsBase;
    std::cout << "Clustering K=" << K_FIXED << " ..." << std::flush;
    KMeans(K_FIXED, 100).run(pts);
    std::cout << " OK\n\n";

    std::ofstream csv("../output/benchmark16.csv");
    csv << "K,N_points,num_threads,wall_time,sa_fit,drstasa_fit\n";

    for (int t = 1; t <= maxThreads; ++t) {
        std::cout << "  num_threads=" << t << "/" << maxThreads << " ..." << std::flush;
        BenchmarkResult res = runPipelineOptimization<NWaypoints>(
            pts, K_FIXED, fitness, zMin, zMax, t);
        std::cout << " time=" << res.wallTime << "s"
                  << "  SA=" << res.saFit
                  << "  DRSTASA=" << res.drstasaFit << "\n";
        csv << K_FIXED  << ","
            << nPoints  << ","
            << t        << ","
            << res.wallTime   << ","
            << res.saFit      << ","
            << res.drstasaFit << "\n";
        csv.flush();
    }

    csv.close();
    std::cout << "\nRisultati salvati in ../output/benchmark16.csv\n";
    return 0;
}
