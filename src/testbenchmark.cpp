#include "structures/functions/fitnessUtilities.hpp"
#include "structures/geo/geoUtils.hpp"
#include "structures/pipeline/pipelineRunner.hpp"
#include "structures/kmeans.hpp"
#include "exporters/pointsListReader.hpp"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <cmath>

static constexpr int    NWaypoints = 4;
static constexpr int    K          = 4;
static constexpr double zMin       = 800.0;
static constexpr double zMax       = 950.0;
static constexpr int    N_RUNS     = 5;


// This software module serves as the entry-point for the experimental
// validation of route optimization algorithms for multi-UAV.
// EXPERIMENT 1: Evaluates execution time across varying CPU thread counts 
// to measure the efficiency of the software parallelization
// EXPERIMENT 2: Analyzes trajectory quality (Fitness) under a progressive 
// increase in active obstacles to verify the convergence capabilities

int main()
{
    const GeoOrigin origin = {63.985, -22.605};

    std::cout << "Loading points...\n";
    PointsList allPoints = PointsListReader::readCSV("../data/input_iceland.csv");
    if (allPoints.size() == 0) {
        std::cerr << "ERROR: could not load input_iceland.csv\n";
        return 1;
    }
    GeoUtils::toMeters(allPoints, origin);
    KMeans(K, 100).run(allPoints);
    std::cout << allPoints.size() << " points loaded, K=" << K << " clusters.\n\n";

    // Experiment 1: wall time vs number of threads
    {
        const std::vector<int> threadCounts = {1, 2, 3, 4};
        FitnessFunction fitness = makeDefaultFitness(zMin, zMax);

        std::ofstream csv("../output/bench_parallel.csv");
        csv << std::fixed << std::setprecision(6);
        csv << "threads,run,sa_fit,drstasa_fit,wall_time\n";

        std::cout << "Experiment 1: Parallelization\n";
        for (int t : threadCounts) {
            for (int r = 0; r < N_RUNS; ++r) {
                std::cout << "  threads=" << t
                          << "  run " << (r + 1) << "/" << N_RUNS
                          << " ... " << std::flush;
                BenchmarkResult res = runPipelineOptimization<NWaypoints>(
                    allPoints, K, fitness, zMin, zMax, t);
                csv << t << "," << r << ","
                    << res.saFit << "," << res.drstasaFit << ","
                    << res.wallTime << "\n";
                csv.flush();
                std::cout << "done (" << std::fixed << std::setprecision(1)
                          << res.wallTime << "s)\n";
            }
        }
        std::cout << "-> ../output/bench_parallel.csv\n\n";
    }

    // Experiment 2: fitness vs number of obstacles 
    // numThreads is fixed to K; obstacle count varies from 0 to max
    {
        const int maxObs  = static_cast<int>(buildDefaultObstacles().size());
        const int nPoints = 7;
        std::vector<int> obsCounts;
        for (int i = 0; i < nPoints; ++i)
            obsCounts.push_back(
                static_cast<int>(std::round(i * maxObs / (nPoints - 1.0))));

        std::ofstream csv("../output/bench_obstacles.csv");
        csv << std::fixed << std::setprecision(6);
        csv << "n_obstacles,run,sa_fit,drstasa_fit,wall_time\n";

        std::cout << "Experiment 2: Fitness vs N Obstacles (max=" << maxObs << ") ===\n";
        for (int nObs : obsCounts) {
            FitnessFunction fitness = makeDefaultFitness(zMin, zMax, nObs);
            for (int r = 0; r < N_RUNS; ++r) {
                std::cout << "  n_obs=" << std::setw(3) << nObs
                          << "  run " << (r + 1) << "/" << N_RUNS
                          << " ... " << std::flush;
                BenchmarkResult res = runPipelineOptimization<NWaypoints>(
                    allPoints, K, fitness, zMin, zMax, K);
                csv << nObs << "," << r << ","
                    << res.saFit << "," << res.drstasaFit << ","
                    << res.wallTime << "\n";
                csv.flush();
                std::cout << std::fixed << std::setprecision(2)
                          << "done  SA=" << res.saFit
                          << "  DRS=" << res.drstasaFit << "\n";
            }
        }
        std::cout << "-> ../output/bench_obstacles.csv\n";
    }

    return 0;
}
