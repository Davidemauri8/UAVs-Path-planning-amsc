#include "structures/pipeline/pipelineRunner.hpp"
#include "structures/geo/geoUtils.hpp"
#include "structures/functions/fitnessUtilities.hpp"
#include "structures/obstacles/cylinderObstacle.hpp"
#include "structures/kmeans.hpp"
#include "exporters/pointsListReader.hpp"
#include <iostream>
#include <fstream>
#include <vector>
#include <omp.h>

// ─── Mission layout (L'Aquila earthquake survey) ────────────────────────────
//
//  GeoOrigin : lat = 42.370 °N   lon = 13.400 °E
//  All metric coordinates below are in metres from that origin.
//
//  START  UAV base         (13.315, 42.343)  metric ≈ (−7000, −3000)  z = 100 m
//  END    Command post     (13.479, 42.406)  metric ≈ (+6500, +4000)  z = 100 m
//
//  Five natural survey clusters:
//    C1  Old Town          centre ≈ (−800,  +200)   alt 120–145 m   40 pts
//    C2  West Quarter      centre ≈ (−5500, +1800)  alt 130–160 m   39 pts
//    C3  East Outskirts    centre ≈ (+5200, −1200)  alt 135–165 m   40 pts
//    C4  North Villages    centre ≈ (+1000, +4800)  alt 140–170 m   40 pts
//    C5  South Zone        centre ≈ (−2000, −4500)  alt 110–140 m   39 pts
//
//  Six cylindrical obstacles on the main inter-cluster flight corridors.
//  Radii ≥ 180 m so the 3-D exclusion zone is effective at 100–170 m altitude.
// ────────────────────────────────────────────────────────────────────────────

static FitnessFunction makeLAquilaFitness(double zMin, double zMax) {
    std::vector<std::shared_ptr<Obstacle>> obs;
    // C1 ↔ C4 corridor  (old town → north villages)
    obs.push_back(std::make_shared<CylinderObstacle>(
        Point(    0.0,  2500.0, 0.0, -1), 200.0, 180.0, 100.0));
    // C2 ↔ C1 corridor  (west quarter → old town)
    obs.push_back(std::make_shared<CylinderObstacle>(
        Point(-2800.0,   900.0, 0.0, -1), 250.0, 180.0, 100.0));
    // C1 ↔ C3 corridor  (old town → east outskirts)
    obs.push_back(std::make_shared<CylinderObstacle>(
        Point( 2200.0,  -600.0, 0.0, -1), 220.0, 180.0,  90.0));
    // C1 ↔ C5 corridor  (old town → south zone)
    obs.push_back(std::make_shared<CylinderObstacle>(
        Point(-1300.0, -2300.0, 0.0, -1), 200.0, 180.0,  80.0));
    // C4 ↔ END corridor  (north villages → command post)
    obs.push_back(std::make_shared<CylinderObstacle>(
        Point( 3000.0,  3000.0, 0.0, -1), 180.0, 180.0,  70.0));
    // C2 ↔ C5 corridor  (west quarter → south zone)
    obs.push_back(std::make_shared<CylinderObstacle>(
        Point(-4500.0, -1700.0, 0.0, -1), 240.0, 180.0, 100.0));
    return FitnessFunction(obs, sampleFitnessWeights(zMin, zMax));
}

int main() {
    constexpr int NWaypoints = 4;
    const double zMin = 90.0, zMax = 180.0;

    GeoOrigin origin{42.370, 13.400};

    PointsList allPointsBase =
        PointsListReader::readCSV("../data/laquila_survey_200.csv");
    if (allPointsBase.size() == 0) {
        std::cerr << "Errore: nessun punto letto.\n";
        return 1;
    }
    GeoUtils::toMeters(allPointsBase, origin);

    FitnessFunction fitness = makeLAquilaFitness(zMin, zMax);
    const int maxThreads = omp_get_max_threads();
    const int nPoints    = allPointsBase.size();

    std::cout << "Threads disponibili : " << maxThreads << "\n";
    std::cout << "Punti nel dataset   : " << nPoints    << "\n";
    std::cout << "Start  : (13.315, 42.343)  UAV base     metric ≈ (−7000, −3000) m\n";
    std::cout << "End    : (13.479, 42.406)  Command post metric ≈ (+6500, +4000) m\n";
    std::cout << "zMin=" << zMin << " m   zMax=" << zMax << " m\n\n";

    const std::vector<int> kValues      = {5, 10, 15, 20};
    const std::vector<int> threadCounts = {1, maxThreads};

    std::ofstream csv("../output/benchmark14.csv");
    csv << "K,N_points,num_threads,wall_time,sa_fit,drstasa_fit\n";

    for (int K : kValues) {
        if (K > nPoints) {
            std::cout << "K=" << K << " > N_points, skip.\n";
            continue;
        }

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
            csv << K        << ","
                << nPoints  << ","
                << t        << ","
                << res.wallTime   << ","
                << res.saFit      << ","
                << res.drstasaFit << "\n";
        }
    }

    csv.close();
    std::cout << "\nRisultati salvati in ../output/benchmark14.csv\n";
    return 0;
}
