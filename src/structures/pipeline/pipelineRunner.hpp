#ifndef PIPELINE_RUNNER_HPP
#define PIPELINE_RUNNER_HPP

#include "kmeans.hpp"
#include "tsp/tspSA.hpp"
#include "segment/segmentOptimizer.hpp"
#include "segment/segmentAppender.hpp"
#include "functions/fitnessUtilities.hpp"
#include "drstasa/drstasa.hpp"
#include "criterion/criterion.hpp"
#include "scheduler/scheduler.hpp"
#include <omp.h>

// Aggregated benchmark output
struct BenchmarkResult {
    double saFit;
    double drstasaFit;
    double wallTime;
    // CPU time (summed across threads) spent inside each optimizer
    double saTime;
    double drstasaTime;
    // Contention overhead: total time each thread spent waiting+executing critical sections
    double criticalTime;
    PointsList drstasaPath;
    PointsList saPath;
    std::vector<PointsList> saPerCluster;
    std::vector<PointsList> drstasaPerCluster;
    std::vector<PointsList> orderedClusters;
};

// Runs the full pipeline
template<int NWaypoints>
BenchmarkResult runPipelineOptimization(
    const PointsList&      allPoints,
    int                    K,
    const FitnessFunction& fitness,
    double zMin, double zMax,
    int numThreads)
{
    omp_set_dynamic(0);
    omp_set_num_threads(numThreads);

    DRSTASA::Config drsCfg = GetConfigurationDRST(NWaypoints);
    drsCfg.zMin = zMin;
    drsCfg.zMax = zMax;

    PointsList saPath, drstasaPath;
    std::vector<PointsList> saPerCluster(K), drstasaPerCluster(K), orderedClusters(K);

    double totalSAFit      = 0.0, totalDRSTASAFit  = 0.0;
    double totalSATime     = 0.0, totalDRSTASATime  = 0.0;
    double totalCritTime   = 0.0;

    const double t0 = omp_get_wtime();

    #pragma omp parallel for schedule(dynamic) \
        reduction(+: totalSAFit, totalDRSTASAFit, \
                     totalSATime, totalDRSTASATime, totalCritTime)
    for (int k = 0; k < K; ++k) {
        PointsList cluster = allPoints.extractCluster(k);
        if (cluster.size() == 0) continue;
        if (cluster.size() < 2) {
            Point p(cluster.getX(0), cluster.getY(0), cluster.getZ(0), -1);
            #pragma omp critical
            { saPath.addPoint(p); drstasaPath.addPoint(p); }
            continue;
        }

        // Order waypoints in the cluster using TSP-SA
        PointsList ordered = TspSA(fitness, std::make_shared<MetropolisCriterion>(),
            std::make_shared<ExponentialScheduler>(100.0, 0.01, 200, 0.95), 500)
            .run(cluster);

        PointsList localSA, localDRS;
        double localSATime = 0.0, localDRSTime = 0.0;

        for (int i = 0; i + 1 < ordered.size(); ++i) {
            Point segStart(ordered.getX(i),   ordered.getY(i),   ordered.getZ(i),   -1);
            Point segEnd  (ordered.getX(i+1), ordered.getY(i+1), ordered.getZ(i+1), -1);

            PointsList segPair;
            segPair.addPoint(segStart);
            segPair.addPoint(segEnd);
            SegmentBounds b = computeBounds(segPair);

            // ── Classic multi-start SA ──────────────────────────────────────
            const double tSA0 = omp_get_wtime();
            auto saRes = runSegmentSAMultiStart<NWaypoints>(
                segStart, segEnd, fitness,
                b.xMin, b.xMax, b.yMin, b.yMax, zMin, zMax);
            localSATime += omp_get_wtime() - tSA0;

            totalSAFit += saRes.fitness;
            appendSASegment<NWaypoints>(localSA, saRes, segStart);

            // ── DRSTASA ─────────────────────────────────────────────────────
            DRSTASA::Config localCfg = drsCfg;
            localCfg.xMin = b.xMin; localCfg.xMax = b.xMax;
            localCfg.yMin = b.yMin; localCfg.yMax = b.yMax;

            const double tDRS0 = omp_get_wtime();
            DRSTASA drstasa(fitness, localCfg);
            PointsList drstasaSeg = drstasa.run(segStart, segEnd);
            localDRSTime += omp_get_wtime() - tDRS0;

            totalDRSTASAFit += drstasa.lastBestFit();
            appendDRSTASASegment(localDRS, drstasaSeg);
        }

        totalSATime     += localSATime;
        totalDRSTASATime += localDRSTime;

        // Append the final waypoint
        Point lastPt(ordered.getX(ordered.size()-1),
                     ordered.getY(ordered.size()-1),
                     ordered.getZ(ordered.size()-1), -1);
        localSA.addPoint(lastPt);
        localDRS.addPoint(lastPt);

        // Measure critical-section contention (wait + execution time per thread)
        const double tCrit0 = omp_get_wtime();
        #pragma omp critical
        {
            // This appends clusters in thread-completion order, not cluster index order.
            // With `schedule(dynamic)` the raw `saPath` / `drstasaPath` become nondeterministic
            // and can export a mission path with clusters shuffled between runs.
            // It is not necessarily an error. But it is useful to note.
            appendPath(saPath, localSA);
            appendPath(drstasaPath, localDRS);
            saPerCluster[k]      = localSA;
            drstasaPerCluster[k] = localDRS;
            orderedClusters[k]   = ordered;
        }
        totalCritTime += omp_get_wtime() - tCrit0;
    }

    return { totalSAFit, totalDRSTASAFit,
             omp_get_wtime() - t0,
             totalSATime, totalDRSTASATime, totalCritTime,
             drstasaPath, saPath,
             saPerCluster, drstasaPerCluster, orderedClusters };
}

#endif
