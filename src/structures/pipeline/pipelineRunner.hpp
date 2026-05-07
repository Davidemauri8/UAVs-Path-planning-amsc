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
};

// Runs the full pipeline 
// numThreads=1 runs serially; numThreads>1 distributes clusters across OMP threads
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

    const DRSTASA::Config drsCfg(NWaypoints, zMin, zMax);
    PointsList saPath, drstasaPath;
    double totalSAFit = 0.0, totalDRSTASAFit = 0.0;

    double t0 = omp_get_wtime();

    #pragma omp parallel for schedule(dynamic) \
        reduction(+:totalSAFit, totalDRSTASAFit)
    for (int k = 0; k < K; ++k) {
        PointsList cluster = allPoints.extractCluster(k);
        if (cluster.size() == 0) continue;
        if (cluster.size() < 2) {
            Point p(cluster.getX(0), cluster.getY(0), cluster.getZ(0), -1);
            #pragma omp critical
            { saPath.addPoint(p); drstasaPath.addPoint(p); }
            continue;
        }

        // Order waypoints in the cluster using TSP-SA.
        PointsList ordered = TspSA(fitness, std::make_shared<MetropolisCriterion>(),
            std::make_shared<ExponentialScheduler>(100.0, 0.01, 200, 0.95), 500).run(cluster);

        PointsList localSA, localDRS;

        for (int i = 0; i + 1 < ordered.size(); ++i) {
            Point segStart(ordered.getX(i),   ordered.getY(i),   ordered.getZ(i),   -1);
            Point segEnd  (ordered.getX(i+1), ordered.getY(i+1), ordered.getZ(i+1), -1);

            SegmentBounds b = computeBounds(ordered);

            // Classic multi-start SA for this segment.
            auto saRes = runSegmentSAMultiStart<NWaypoints>(
                segStart, segEnd, fitness,
                b.xMin, b.xMax, b.yMin, b.yMax, zMin, zMax);
            totalSAFit += saRes.fitness;
            appendSASegment<NWaypoints>(localSA, saRes, segStart);

            // DRSTASA for the same segment with locally-adjusted bounds.
            DRSTASA::Config localCfg = drsCfg;
            localCfg.xMin = b.xMin; localCfg.xMax = b.xMax;
            localCfg.yMin = b.yMin; localCfg.yMax = b.yMax;
            DRSTASA drstasa(fitness, localCfg);
            PointsList drstasaSeg = drstasa.run(segStart, segEnd);
            totalDRSTASAFit += drstasa.lastBestFit();
            appendDRSTASASegment(localDRS, drstasaSeg);
        }

        // Append the final waypoint (the segment end-point of the last segment).
        Point lastPt(ordered.getX(ordered.size()-1),
                     ordered.getY(ordered.size()-1),
                     ordered.getZ(ordered.size()-1), -1);
        localSA.addPoint(lastPt);
        localDRS.addPoint(lastPt);

        #pragma omp critical
        { appendPath(saPath, localSA); appendPath(drstasaPath, localDRS); }
    }

    return { totalSAFit, totalDRSTASAFit, omp_get_wtime() - t0 };
}

#endif
