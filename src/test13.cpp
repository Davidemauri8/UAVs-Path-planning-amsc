#include "structures/pipeline/pipelineRunner.hpp"
#include "structures/geo/geoUtils.hpp"
#include "structures/functions/fitnessUtilities.hpp"
#include "structures/kmeans.hpp"
#include "exporters/pointsListReader.hpp"
#include <iostream>
#include <fstream>
#include <vector>
#include <omp.h>

int main() {
    constexpr int NWaypoints = 4;
    const double zMin = 100.0, zMax = 200.0;
    GeoOrigin origin{45.4641, 9.1919};

    PointsList allPointsBase = PointsListReader::readCSV("../data/earthquake_sim_200.csv");
    if (allPointsBase.size() == 0) { std::cerr << "Errore: nessun punto letto.\n"; return 1; }
    GeoUtils::toMeters(allPointsBase, origin);

    FitnessFunction fitness = makeDefaultFitness(zMin, zMax);
    const int maxThreads = omp_get_max_threads();
    const int nPoints    = allPointsBase.size();

    std::cout << "Threads disponibili: " << maxThreads << "\n";
    std::cout << "Punti nel dataset:   " << nPoints    << "\n\n";

    const std::vector<int> kValues      = {3, 5, 8, 10, 15, 20};
    const std::vector<int> threadCounts = {1, maxThreads};

    std::ofstream csv("../output/benchmark.csv");
    csv << "K,N_points,num_threads,wall_time,sa_fit,drstasa_fit\n";

    for (int K : kValues) {
        if (K > nPoints) { std::cout << "K=" << K << " > N_points, skip.\n"; continue; }

        std::cout << "K=" << K << " — clustering..." << std::flush;
        PointsList pts = allPointsBase;
        KMeans(K, 100).run(pts);
        std::cout << " OK\n";

        for (int t : threadCounts) {
            std::cout << "  threads=" << t << " ..." << std::flush;
            BenchmarkResult res = runPipelineOptimization<NWaypoints>(
                pts, K, fitness, zMin, zMax, t);
            std::cout << " time=" << res.wallTime << "s"
                      << "  SA="      << res.saFit
                      << "  DRSTASA=" << res.drstasaFit << "\n";
            csv << K       << ","
                << nPoints << ","
                << t       << ","
                << res.wallTime    << ","
                << res.saFit       << ","
                << res.drstasaFit  << "\n";
        }
    }

    csv.close();
    std::cout << "\nRisultati salvati in ../output/benchmark.csv\n";
    return 0;
}
