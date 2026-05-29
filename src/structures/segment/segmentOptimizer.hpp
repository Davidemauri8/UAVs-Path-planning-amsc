#ifndef SEGMENT_OPTIMIZER_HPP
#define SEGMENT_OPTIMIZER_HPP

#include "structures/fitness/fitnessFunction.hpp"
#include "structures/pointsList.hpp"
#include "structures/point.hpp"
#include "structures/lhs/lhs.hpp"
#include "structures/drstasa/drstasa.hpp"
#include "structures/functions/fitnessUtilities.hpp"

#include "geometry/point.hpp"
#include "domain/domain.hpp"
#include "scheduler/scheduler.hpp"
#include "sampler/local_sampler_fixed.hpp"
#include <cmath>
#include "criterion/criterion.hpp"
#include "algorithms/annealing_policy.hpp"
#include "algorithms/serial_simulated_annealing.hpp"

#include <cmath>
#include <memory>
#include <omp.h>

// Axis-aligned bounding box for a cluster, with an optional safety margin
struct SegmentBounds { double xMin, xMax, yMin, yMax; };

// Computes the XY bounding box of ordered waypoints and expands it by a margin
SegmentBounds computeBounds(const PointsList& ordered,
                            double marginFactor = 1.0,
                            double marginMin    = 10.0);

// Result of a single classic SA run on one path segment
template<int NWaypoints>
struct SegmentSAResult {
    point_nd<NWaypoints * 3> bestPoint;
    double                   fitness;
};

// Runs a single classic SA optimisation 
// The search domain is a sphere centred on the cluster's bounding box midpoint
template<int NWaypoints>
SegmentSAResult<NWaypoints> runSegmentSA(
    const Point&           segStart,
    const Point&           segEnd,
    const FitnessFunction& fitness,
    double cxMin, double cxMax,
    double cyMin, double cyMax,
    double zMin,  double zMax,
    long   maxIter = 5000)
{
    constexpr int Dim = NWaypoints * 3;

    Lhs lhs(cxMin, cxMax, cyMin, cyMax, zMin, zMax, NWaypoints, -1);
    point_nd<Dim> startPoint = lhs.toPointNd<NWaypoints>();

    auto segObjective = [segStart, segEnd, &fitness](const point_nd<Dim>& p) -> double {
        PointsList path;
        path.addPoint(segStart);
        for (int w = 0; w < NWaypoints; ++w)
            path.addPoint(Point(p[w*3+0], p[w*3+1], p[w*3+2], -1));
        path.addPoint(segEnd);
        double cost = fitness.evaluate(path);
        return (std::isinf(cost) || std::isnan(cost)) ? 1e12 : cost;
    };

    point_nd<Dim> domRef   = 0.0;
    point_nd<Dim> domSides = 0.0;
    for (int w = 0; w < NWaypoints; ++w) {
        domRef[w*3+0]   = cxMin; domSides[w*3+0] = cxMax - cxMin;
        domRef[w*3+1]   = cyMin; domSides[w*3+1] = cyMax - cyMin;
        domRef[w*3+2]   = zMin;  domSides[w*3+2] = zMax  - zMin;
    }
    // Step radius: use only the x/y diagonal to avoid the scale mismatch
    // between the large geographic extent (km) and the narrow z range (150 m).
    // Dividing by sqrt(Dim) keeps the per-dimension perturbation proportional
    // to each coordinate's own range instead of the full hypersphere radius.
    double xyDiag     = std::sqrt(std::pow(cxMax - cxMin, 2) +
                                   std::pow(cyMax - cyMin, 2));
    double stepRadius = 0.1 * xyDiag / std::sqrt(static_cast<double>(Dim));

    auto domain    = RnDomainFactory::make_hyperrectangle(domSides, domRef);
    auto saNeigh   = std::make_shared<CircleNeighbourhood<Dim>>(domain, stepRadius);
    auto saSampler = std::make_shared<LocalSamplerFixed<CircleNeighbourhood<Dim>>>(0.1);
    auto saCrit    = std::make_shared<MetropolisCriterion>();
    // stab_it=15, maxIter=5000 → 75,000 evaluations, matching DRSTASA budget
    // (popSize=30 × 5 operators × maxIter=500 ≈ 75,000).
    auto saSched   = std::make_shared<ExponentialScheduler>(100.0, 0.01, 15, 0.95);
    saNeigh->from(startPoint, startPoint);

    AnnealingExecutionPolicy<LocalSamplerFixed<CircleNeighbourhood<Dim>>> saPolicy(
        saCrit, saSampler, saSched, StoppingCriterion::temp_zero, *saNeigh);
    SerialSimulatedAnnealing ssa(maxIter);
    auto saResult = ssa.run(segObjective, startPoint, saPolicy);

    return { saResult, segObjective(saResult) };
}

// runs nRestarts independent SA restarts and returns the best result
template<int NWaypoints>
SegmentSAResult<NWaypoints> runSegmentSAMultiStart(
    const Point&           segStart,
    const Point&           segEnd,
    const FitnessFunction& fitness,
    double cxMin, double cxMax,
    double cyMin, double cyMax,
    double zMin,  double zMax,
    int  nRestarts         = 4,
    long maxIterPerRestart  = 6000)
{
    auto globalBest = runSegmentSA<NWaypoints>(
        segStart, segEnd, fitness,
        cxMin, cxMax, cyMin, cyMax, zMin, zMax, maxIterPerRestart);

    #pragma omp parallel for schedule(static) if(!omp_in_parallel())
    for (int run = 1; run < nRestarts; ++run) {
        auto res = runSegmentSA<NWaypoints>(
            segStart, segEnd, fitness,
            cxMin, cxMax, cyMin, cyMax, zMin, zMax, maxIterPerRestart);

        #pragma omp critical
        if (res.fitness < globalBest.fitness)
            globalBest = res;
    }
    return globalBest;
}

// Holds the results of both optimisers for a single segment
struct SegmentResult {
    PointsList saPath;
    PointsList drstasaPath;
    double     saFit;
    double     drstasaFit;
};

// Appends all points of src to dest
inline void appendPath(PointsList& dest, const PointsList& src) {
    for (int w = 0; w < src.size(); ++w)
        dest.addPoint(Point(src.getX(w), src.getY(w), src.getZ(w), -1));
}

// Runs both classic SA and DRSTASA on the segment and returns both paths
template <int NWaypoints>
SegmentResult optimizeSegment(
    const Point&          segStart,
    const Point&          segEnd,
    const FitnessFunction& fitness,
    const SegmentBounds&  bounds,
    double                zMin,
    double                zMax,
    const DRSTASA::Config& drsCfg)
{
    constexpr int Dim = NWaypoints * 3;

    auto segObjective = [&](const point_nd<Dim>& p) -> double {
        PointsList path;
        path.addPoint(segStart);
        for (int w = 0; w < NWaypoints; ++w)
            path.addPoint(Point(p[w*3+0], p[w*3+1], p[w*3+2], -1));
        path.addPoint(segEnd);
        double cost = fitness.evaluate(path);
        return (std::isinf(cost) || std::isnan(cost)) ? 1e12 : cost;
    };

    // SA
    Lhs lhs(bounds.xMin, bounds.xMax, bounds.yMin, bounds.yMax, zMin, zMax, NWaypoints, -1);
    point_nd<Dim> startPoint = lhs.toPointNd<NWaypoints>();

    point_nd<Dim> domRef   = 0.0;
    point_nd<Dim> domSides = 0.0;
    for (int w = 0; w < NWaypoints; ++w) {
        domRef[w*3+0]   = bounds.xMin; domSides[w*3+0] = bounds.xMax - bounds.xMin;
        domRef[w*3+1]   = bounds.yMin; domSides[w*3+1] = bounds.yMax - bounds.yMin;
        domRef[w*3+2]   = zMin;        domSides[w*3+2] = zMax        - zMin;
    }
    double xyDiag2    = std::sqrt(std::pow(bounds.xMax - bounds.xMin, 2) +
                                   std::pow(bounds.yMax - bounds.yMin, 2));
    double stepRadius = 0.1 * xyDiag2 / std::sqrt(static_cast<double>(Dim));

    auto domain    = RnDomainFactory::make_hyperrectangle(domSides, domRef);
    auto saNeigh   = std::make_shared<CircleNeighbourhood<Dim>>(domain, stepRadius);
    auto saSampler = std::make_shared<LocalSamplerFixed<CircleNeighbourhood<Dim>>>(0.1);
    auto saCrit    = std::make_shared<MetropolisCriterion>();
    // stab_it=15, maxIter=5000 → 75,000 evaluations, matching DRSTASA budget.
    auto saSched   = std::make_shared<ExponentialScheduler>(100.0, 0.01, 15, 0.95);
    saNeigh->from(startPoint, startPoint);

    AnnealingExecutionPolicy<LocalSamplerFixed<CircleNeighbourhood<Dim>>> saPolicy(
        saCrit, saSampler, saSched, StoppingCriterion::temp_zero, *saNeigh);
    SerialSimulatedAnnealing ssa(5000);
    auto saResult = ssa.run(segObjective, startPoint, saPolicy);

    SegmentResult res;
    res.saFit = segObjective(saResult);
    res.saPath.addPoint(segStart);
    for (int w = 0; w < NWaypoints; ++w)
        res.saPath.addPoint(Point(saResult[w*3+0], saResult[w*3+1], saResult[w*3+2], -1));

    // DRSTASA 
    DRSTASA drstasa(fitness, drsCfg);
    PointsList drstasaSeg = drstasa.run(segStart, segEnd);
    res.drstasaFit = drstasa.lastBestFit();

    // drstasaSeg contains start+waypoints+end; exclude the end-point.
    for (int w = 0; w < drstasaSeg.size() - 1; ++w)
        res.drstasaPath.addPoint(Point(drstasaSeg.getX(w), drstasaSeg.getY(w),
                                      drstasaSeg.getZ(w), -1));

    return res;
}

#endif
