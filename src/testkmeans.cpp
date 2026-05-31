#include "structures/functions/fitnessUtilities.hpp"
#include "structures/geo/geoUtils.hpp"
#include "structures/pipeline/pipelineRunner.hpp"
#include "structures/kmeans.hpp"
#include "exporters/pointsListReader.hpp"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>

// K-scaling benchmark
// ---------------------------------------------------------------
// Hypothesis: speedup is bounded by min(K, NUM_THREADS), not by
// the hardware alone.  With K < NUM_THREADS some threads sit idle;
// with K >= NUM_THREADS all threads stay busy and wall time plateaus.
//
// Protocol:
//   - NUM_THREADS is fixed throughout the experiment.
//   - K sweeps from 1 (fully serial) to 20 (over-subscribed wrt threads).
//   - For each K: KMeans is run once; N_RUNS independent optimisations
//     reuse the same clustering so variance comes only from the solver.
//   - DRSTASA params are reduced (maxIter=200, popSize=20) so that
//     the benchmark completes in a few minutes.
//   - Zero obstacles: fitness = path length + altitude dev + smoothness.
//     This keeps each evaluation fast and isolates the parallel-scaling
//     behaviour from obstacle-check overhead.
// ---------------------------------------------------------------

static constexpr int    NWaypoints  = 4;
static constexpr int    NUM_THREADS = 4;   // fix to your physical core count
static constexpr double zMin        = 800.0;
static constexpr double zMax        = 950.0;
static constexpr int    N_RUNS      = 3;

int main()
{
    // Norway region — 8 natural clusters of 25 points spanning 59°–71° N
    const GeoOrigin origin = {65.0, 14.0};

    std::cout << "Loading points...\n";
    PointsList basePoints = PointsListReader::readCSV("../data/input_km.csv");
    if (basePoints.size() == 0) {
        std::cerr << "ERROR: could not load input_km.csv\n";
        return 1;
    }
    GeoUtils::toMeters(basePoints, origin);
    std::cout << basePoints.size() << " points loaded.\n\n";

    // No obstacles: fitness evaluations are faster and we isolate scaling behaviour
    FitnessFunction fitness = makeDefaultFitness(zMin, zMax, 0);

    // Reduced DRSTASA budget so the full K sweep finishes in minutes
    DRSTASA::Config fastCfg = GetConfigurationDRST(NWaypoints);
    fastCfg.zMin    = zMin;
    fastCfg.zMax    = zMax;
    fastCfg.maxIter = 200;
    fastCfg.popSize = 20;

    const std::vector<int> kValues = {1, 2, 3, 4, 5, 6, 8, 10, 12, 16, 20};

    std::ofstream csv("../output/bench_kmeans.csv");
    csv << std::fixed << std::setprecision(6);
    csv << "k,threads,run,sa_fit,drstasa_fit,wall_time\n";

    std::cout << "K-scaling benchmark  (fixed threads=" << NUM_THREADS << ")\n";
    std::cout << std::string(55, '-') << "\n";

    for (int k : kValues) {
        // Cluster once per K; all N_RUNS reuse the same assignment
        PointsList pts = basePoints;
        KMeans(k, 100).run(pts);

        for (int r = 0; r < N_RUNS; ++r) {
            std::cout << "  K=" << std::setw(2) << k
                      << "  run " << (r + 1) << "/" << N_RUNS
                      << " ... " << std::flush;

            BenchmarkResult res = runPipelineOptimization<NWaypoints>(
                pts, k, fitness, zMin, zMax, NUM_THREADS, &fastCfg);

            csv << k << "," << NUM_THREADS << "," << r << ","
                << res.saFit << "," << res.drstasaFit << ","
                << res.wallTime << "\n";
            csv.flush();

            std::cout << "done (" << std::fixed << std::setprecision(2)
                      << res.wallTime << "s)"
                      << "  SA=" << std::setprecision(1) << res.saFit
                      << "  DRS=" << res.drstasaFit << "\n";
        }
        std::cout << "\n";
    }

    std::cout << "-> ../output/bench_kmeans.csv\n";
    return 0;
}
