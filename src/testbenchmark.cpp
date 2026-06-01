#include "structures/functions/fitnessUtilities.hpp"
#include "structures/geo/geoUtils.hpp"
#include "structures/pipeline/pipelineRunner.hpp"
#include "structures/kmeans.hpp"
#include "exporters/pointsListReader.hpp"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <numeric>
#include <vector>
#include <cmath>

static constexpr int    NWaypoints   = 4;
static constexpr int    K_DEFAULT    = 4;
static constexpr double zMin         = 800.0;
static constexpr double zMax         = 950.0;
static constexpr int    N_RUNS       = 5;

// This software module serves as the entry-point for the experimental
// validation of route optimization algorithms for multi-UAV.
// EXPERIMENT 1: Evaluates execution time across varying CPU thread counts
//               to measure parallelization efficiency; derives Amdahl serial fraction.
// EXPERIMENT 2: Analyzes trajectory fitness under increasing active obstacles
//               to verify convergence of SA vs DRSTASA.
// EXPERIMENT 3: Wall-clock time vs K (number of clusters) with fixed threads
//               to show how speedup scales with K.

int main()
{
    const GeoOrigin origin = {63.985, -22.605};

    std::cout << "Loading points...\n";
    PointsList rawPoints = PointsListReader::readCSV("../data/input_iceland.csv");
    if (rawPoints.size() == 0) {
        std::cerr << "ERROR: could not load input_iceland.csv\n";
        return 1;
    }
    GeoUtils::toMeters(rawPoints, origin);
    std::cout << rawPoints.size() << " points loaded.\n\n";

    // ── Experiment 1: wall time vs number of threads ─────────────────────────
    {
        const std::vector<int> threadCounts = {1, 2, 3, 4};
        FitnessFunction fitness = makeDefaultFitness(zMin, zMax);

        // Cluster once with K_DEFAULT; same labelling for every thread-count run
        PointsList allPoints = rawPoints;
        KMeans(K_DEFAULT, 100).run(allPoints);

        std::ofstream csv("../output/bench_parallel.csv");
        csv << std::fixed << std::setprecision(6);
        csv << "threads,run,sa_fit,drstasa_fit,wall_time,"
               "sa_time,drstasa_time,critical_time\n";

        std::cout << "Experiment 1: Parallelization (K=" << K_DEFAULT << ")\n";

        // Collect mean wall-times per thread count for Amdahl estimation
        std::vector<double> meanWall(threadCounts.size(), 0.0);

        for (size_t ti = 0; ti < threadCounts.size(); ++ti) {
            int t = threadCounts[ti];
            double sumWall = 0.0;
            for (int r = 0; r < N_RUNS; ++r) {
                std::cout << "  threads=" << t
                          << "  run " << (r+1) << "/" << N_RUNS
                          << " ... " << std::flush;
                BenchmarkResult res = runPipelineOptimization<NWaypoints>(
                    allPoints, K_DEFAULT, fitness, zMin, zMax, t);
                csv << t   << "," << r             << ","
                    << res.saFit      << "," << res.drstasaFit  << ","
                    << res.wallTime   << ","
                    << res.saTime     << "," << res.drstasaTime << ","
                    << res.criticalTime << "\n";
                csv.flush();
                sumWall += res.wallTime;
                std::cout << "done (" << std::fixed << std::setprecision(1)
                          << res.wallTime << "s)\n";
            }
            meanWall[ti] = sumWall / N_RUNS;
        }
        std::cout << "-> ../output/bench_parallel.csv\n\n";

        // ── Amdahl's law estimate ─────────────────────────────────────────
        // f = (p/S_p - 1) / (p - 1)  where S_p = T_1 / T_p
        std::cout << "Amdahl serial-fraction estimate:\n";
        double T1 = meanWall[0];
        double fSum = 0.0;
        int    fCount = 0;
        for (size_t ti = 1; ti < threadCounts.size(); ++ti) {
            int    p  = threadCounts[ti];
            double Sp = T1 / meanWall[ti];
            double f  = (static_cast<double>(p) / Sp - 1.0) /
                        (static_cast<double>(p) - 1.0);
            f = std::max(0.0, std::min(1.0, f));   // clamp to [0,1]
            std::cout << "  p=" << p
                      << "  speedup=" << std::fixed << std::setprecision(2) << Sp
                      << "  f_serial=" << std::fixed << std::setprecision(4) << f
                      << "\n";
            fSum += f; ++fCount;
        }
        const double fMean = fSum / fCount;
        const double maxSpeedup = 1.0 / fMean;
        std::cout << "  mean f_serial = " << std::fixed << std::setprecision(4) << fMean
                  << "  => theoretical max speedup = "
                  << std::fixed << std::setprecision(1) << maxSpeedup << "x\n\n";
    }

    // ── Experiment 2: fitness vs number of obstacles ──────────────────────────
    {
        const int maxObs  = static_cast<int>(buildDefaultObstacles().size());
        const int nPoints = 7;
        std::vector<int> obsCounts;
        for (int i = 0; i < nPoints; ++i)
            obsCounts.push_back(
                static_cast<int>(std::round(i * maxObs / (nPoints - 1.0))));

        PointsList allPoints = rawPoints;
        KMeans(K_DEFAULT, 100).run(allPoints);

        std::ofstream csv("../output/bench_obstacles.csv");
        csv << std::fixed << std::setprecision(6);
        csv << "n_obstacles,run,sa_fit,drstasa_fit,wall_time,"
               "sa_time,drstasa_time,critical_time\n";

        std::cout << "Experiment 2: Fitness vs N Obstacles (max=" << maxObs << ")\n";
        for (int nObs : obsCounts) {
            FitnessFunction fitness = makeDefaultFitness(zMin, zMax, nObs);
            for (int r = 0; r < N_RUNS; ++r) {
                std::cout << "  n_obs=" << std::setw(3) << nObs
                          << "  run " << (r+1) << "/" << N_RUNS
                          << " ... " << std::flush;
                BenchmarkResult res = runPipelineOptimization<NWaypoints>(
                    allPoints, K_DEFAULT, fitness, zMin, zMax, K_DEFAULT);
                csv << nObs << "," << r           << ","
                    << res.saFit      << "," << res.drstasaFit  << ","
                    << res.wallTime   << ","
                    << res.saTime     << "," << res.drstasaTime << ","
                    << res.criticalTime << "\n";
                csv.flush();
                std::cout << std::fixed << std::setprecision(2)
                          << "done  SA=" << res.saFit
                          << "  DRS=" << res.drstasaFit << "\n";
            }
        }
        std::cout << "-> ../output/bench_obstacles.csv\n\n";
    }

    // ── Experiment 3: wall time vs K (number of clusters), fixed threads ──────
    // Shows how speedup scales with K independently of the thread ceiling.
    {
        const std::vector<int> kValues      = {2, 4, 6, 8};
        const int              fixedThreads = 4;
        FitnessFunction fitness = makeDefaultFitness(zMin, zMax);

        std::ofstream csv("../output/bench_scaling.csv");
        csv << std::fixed << std::setprecision(6);
        csv << "k,run,sa_fit,drstasa_fit,wall_time,"
               "sa_time,drstasa_time,critical_time\n";

        std::cout << "Experiment 3: K-Scaling (threads=" << fixedThreads << ")\n";
        for (int k : kValues) {
            // Re-cluster with this K value
            PointsList pts = rawPoints;
            KMeans(k, 100).run(pts);

            for (int r = 0; r < N_RUNS; ++r) {
                std::cout << "  K=" << k
                          << "  run " << (r+1) << "/" << N_RUNS
                          << " ... " << std::flush;
                BenchmarkResult res = runPipelineOptimization<NWaypoints>(
                    pts, k, fitness, zMin, zMax, fixedThreads);
                csv << k   << "," << r             << ","
                    << res.saFit      << "," << res.drstasaFit  << ","
                    << res.wallTime   << ","
                    << res.saTime     << "," << res.drstasaTime << ","
                    << res.criticalTime << "\n";
                csv.flush();
                std::cout << "done (" << std::fixed << std::setprecision(1)
                          << res.wallTime << "s)\n";
            }
        }
        std::cout << "-> ../output/bench_scaling.csv\n";
    }

    return 0;
}
