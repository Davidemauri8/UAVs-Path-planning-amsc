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
#include "sampler/local_sampler.hpp"
#include "criterion/criterion.hpp"
#include "algorithms/annealing_policy.hpp"
#include "algorithms/serial_simulated_annealing.hpp"

#include <cmath>
#include <memory>
#include <omp.h>

struct SegmentBounds { double xMin, xMax, yMin, yMax; };

/// Calcola i bounds del cluster ordinato con margine (default 20%, minimo 50m).
/// Implementazione in segmentOptimizer.cpp.
SegmentBounds computeBounds(const PointsList& ordered,
                            double marginFactor = 0.2,
                            double marginMin    = 50.0);

/// Risultato della sola ottimizzazione SA di un segmento
template<int NWaypoints>
struct SegmentSAResult {
    point_nd<NWaypoints * 3> bestPoint;
    double                   fitness;
};

/// SA classico su un singolo segmento start→end. Nessun side effect su stato esterno.
template<int NWaypoints>
SegmentSAResult<NWaypoints> runSegmentSA(
    const Point&           segStart,
    const Point&           segEnd,
    const FitnessFunction& fitness,
    double cxMin, double cxMax,
    double cyMin, double cyMax,
    double zMin,  double zMax,
    long   maxIter = 6000)
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

    point_nd<Dim> domCenter = 0.0;
    for (int w = 0; w < NWaypoints; ++w) {
        domCenter[w*3+0] = (cxMin + cxMax) / 2.0;
        domCenter[w*3+1] = (cyMin + cyMax) / 2.0;
        domCenter[w*3+2] = (zMin  + zMax)  / 2.0;
    }
    double domRadius = std::sqrt(std::pow(cxMax - cxMin, 2) +
                                 std::pow(cyMax - cyMin, 2) +
                                 std::pow(zMax  - zMin,  2));

    auto domain    = RnDomainFactory::make_sphere(domRadius, domCenter);
    auto saNeigh   = std::make_shared<CircleNeighbourhood<Dim>>(domain, domRadius * 0.1);
    auto saSampler = std::make_shared<LocalSampler<CircleNeighbourhood<Dim>>>(0.1);
    auto saCrit    = std::make_shared<MetropolisCriterion>();
    auto saSched   = std::make_shared<ExponentialScheduler>(100.0, 0.01, 1, 0.95);
    saNeigh->from(startPoint, startPoint);

    AnnealingExecutionPolicy<LocalSampler<CircleNeighbourhood<Dim>>> saPolicy(
        saCrit, saSampler, saSched, StoppingCriterion::temp_zero, *saNeigh);
    SerialSimulatedAnnealing ssa(maxIter);
    auto saResult = ssa.run(segObjective, startPoint, saPolicy);

    return { saResult, segObjective(saResult) };
}

/// Multi-start SA: nRestarts restart indipendenti, restituisce il migliore.
/// Budget equalizzato con DRSTASA: nRestarts × maxIterPerRestart = popSize × maxIter × 4.
/// Parallelo automaticamente quando non già dentro una regione OMP (if(!omp_in_parallel())).
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

/// Risultato dell'ottimizzazione di un segmento: percorso SA e DRSTASA (start+waypoints, senza end)
struct SegmentResult {
    PointsList saPath;
    PointsList drstasaPath;
    double     saFit;
    double     drstasaFit;
};

/// Appende tutti i punti di src in dest
inline void appendPath(PointsList& dest, const PointsList& src) {
    for (int w = 0; w < src.size(); ++w)
        dest.addPoint(Point(src.getX(w), src.getY(w), src.getZ(w), -1));
}

/// Esegue SA classico e DRSTASA su un segmento start→end, restituisce entrambi i percorsi
/// I percorsi nel risultato contengono start+waypoints ma NON l'end (aggiunto dal chiamante)
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

    // ── SA classico ──────────────────────────────────────────────────────────
    Lhs lhs(bounds.xMin, bounds.xMax, bounds.yMin, bounds.yMax, zMin, zMax, NWaypoints, -1);
    point_nd<Dim> startPoint = lhs.toPointNd<NWaypoints>();

    point_nd<Dim> domCenter = 0.0;
    for (int w = 0; w < NWaypoints; ++w) {
        domCenter[w*3+0] = (bounds.xMin + bounds.xMax) / 2.0;
        domCenter[w*3+1] = (bounds.yMin + bounds.yMax) / 2.0;
        domCenter[w*3+2] = (zMin + zMax) / 2.0;
    }
    double domRadius = std::sqrt(std::pow(bounds.xMax - bounds.xMin, 2) +
                                 std::pow(bounds.yMax - bounds.yMin, 2) +
                                 std::pow(zMax - zMin, 2));

    auto domain    = RnDomainFactory::make_sphere(domRadius, domCenter);
    auto saNeigh   = std::make_shared<CircleNeighbourhood<Dim>>(domain, domRadius * 0.1);
    auto saSampler = std::make_shared<LocalSampler<CircleNeighbourhood<Dim>>>(0.1);
    auto saCrit    = std::make_shared<MetropolisCriterion>();
    auto saSched   = std::make_shared<ExponentialScheduler>(100.0, 0.01, 1, 0.95);
    saNeigh->from(startPoint, startPoint);

    AnnealingExecutionPolicy<LocalSampler<CircleNeighbourhood<Dim>>> saPolicy(
        saCrit, saSampler, saSched, StoppingCriterion::temp_zero, *saNeigh);
    SerialSimulatedAnnealing ssa(6000);
    auto saResult = ssa.run(segObjective, startPoint, saPolicy);

    SegmentResult res;
    res.saFit = segObjective(saResult);
    res.saPath.addPoint(segStart);
    for (int w = 0; w < NWaypoints; ++w)
        res.saPath.addPoint(Point(saResult[w*3+0], saResult[w*3+1], saResult[w*3+2], -1));

    // ── DRSTASA ──────────────────────────────────────────────────────────────
    DRSTASA drstasa(fitness, drsCfg);
    PointsList drstasaSeg = drstasa.run(segStart, segEnd);
    res.drstasaFit = drstasa.lastBestFit();

    // drstasaSeg contiene start+waypoints+end: escludiamo l'end
    for (int w = 0; w < drstasaSeg.size() - 1; ++w)
        res.drstasaPath.addPoint(Point(drstasaSeg.getX(w), drstasaSeg.getY(w),
                                      drstasaSeg.getZ(w), -1));

    return res;
}

#endif
