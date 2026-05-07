#include "structures/fitness/fitnessFunction.hpp"
#include "structures/obstacles/cylinderObstacle.hpp"
#include "structures/pointsList.hpp"
#include "structures/lhs/lhs.hpp"
#include "exporters/pointsListExporter.hpp"

#include "geometry/point.hpp"
#include "domain/domain.hpp"
#include "scheduler/scheduler.hpp"
#include "sampler/sampler_base.hpp"
#include "sampler/uniform_sampler.hpp"
#include "sampler/local_sampler.hpp"
#include "neighbourhoods/neighbourhood_gen.hpp"
#include "criterion/criterion.hpp"
#include "algorithms/annealing_policy.hpp"
#include "algorithms/simulated_annealing.hpp"
#include "algorithms/serial_simulated_annealing.hpp"

#include <iostream>
#include <vector>
#include <memory>
#include <cmath>

FitnessWeights sampleFitnessWeights(const double zMin, const double zMax){

    FitnessWeights weights;
    weights.b1   = 5.0;
    weights.b2   = 10.0;
    weights.b3   = 1.0;
    weights.b4   = 5.0;
    weights.a1   = 1.0;
    weights.a2   = 1.0;
    weights.hMin = zMin;
    weights.hMax = zMax;

    return weights;    
}

FitnessFunction sampleFitnessFuction(const double zMin, const double zMax){

    std::vector<Obstacle*> obstacles;
    obstacles.push_back(new CylinderObstacle(
        Point(250.0, 350.0, 0.0, -1), 40.0, 200.0, 50.0));
    obstacles.push_back(new CylinderObstacle(
        Point(350.0, 450.0, 0.0, -1), 40.0, 200.0, 50.0));

    // ── 3. Fitness function ───────────────────────────────────────
    FitnessWeights weights = sampleFitnessWeights(zMin, zMax);

    FitnessFunction fitness(obstacles, weights);

    return fitness;
}


int main() {

    // ── 1. Parametri missione ─────────────────────────────────────
    constexpr int NWaypoints = 4;
    constexpr int Dim = NWaypoints * 3;  // 12 variabili continue

    const Point start(100.0, 200.0, 120.0, -1);
    const Point end  (400.0, 500.0, 130.0, -1);

    const double xMin = 80.0,  xMax = 450.0;
    const double yMin = 180.0, yMax = 550.0;
    const double zMin = 100.0, zMax = 180.0;

    //
    FitnessFunction fitness = sampleFitnessFuction(zMin, zMax);

    // ── 4. Inizializzazione con LHS ───────────────────────────────
    Lhs lhs(xMin, xMax, yMin, yMax, zMin, zMax, NWaypoints, -1);
    point_nd<Dim> startPoint = lhs.toPointNd<NWaypoints>();

    std::cout << "Waypoint iniziali (LHS):" << std::endl;
    for (int i = 0; i < NWaypoints; ++i) {
        std::cout << "  W" << i+1 << ": ("
                  << startPoint[i*3+0] << ", "
                  << startPoint[i*3+1] << ", "
                  << startPoint[i*3+2] << ")" << std::endl;
    }

    // ── 5. Funzione obiettivo ─────────────────────────────────────
    auto drone_objective = [&](const point_nd<Dim>& p) -> double {
        PointsList path;

        path.addPoint(start);

        for (int i = 0; i < NWaypoints; ++i)
            path.addPoint(Point(p[i*3+0], p[i*3+1], p[i*3+2], -1));

        path.addPoint(end);

        double cost = fitness.evaluate(path);
        if (std::isinf(cost) || std::isnan(cost)) return 1e12;
        return cost;
    };

    // ── 6. Dominio continuo ───────────────────────────────────────
    point_nd<Dim> domainCenter = 0.0;
    for (int i = 0; i < NWaypoints; ++i) {
        domainCenter[i*3+0] = (xMin + xMax) / 2.0;
        domainCenter[i*3+1] = (yMin + yMax) / 2.0;
        domainCenter[i*3+2] = (zMin + zMax) / 2.0;
    }

    const double domainRadius = std::sqrt(
        std::pow(xMax - xMin, 2) +
        std::pow(yMax - yMin, 2) +
        std::pow(zMax - zMin, 2)
    );

    auto domain = RnDomainFactory::make_sphere(domainRadius, domainCenter);
    auto neigh  = std::make_shared<CircleNeighbourhood<Dim>>(domain, domainRadius * 0.1);

    // ── 7. Configurazione SA ──────────────────────────────────────
    auto sampler   = std::make_shared<LocalSampler<CircleNeighbourhood<Dim>>>(0.1);
    auto criterion = std::make_shared<MetropolisCriterion>();

    // ordine corretto: T0, Tmin, stab_it, alpha
    std::shared_ptr<ParamScheduler> scheduler =
        std::make_shared<ExponentialScheduler>(100.0, 0.01, 500, 0.95);

    constexpr long maxIter = 1000;
    SerialSimulatedAnnealing ssa(maxIter);

    // ── 8. Esecuzione ─────────────────────────────────────────────
    neigh->from(startPoint, startPoint);

    AnnealingExecutionPolicy<LocalSampler<CircleNeighbourhood<Dim>>> policy(
        criterion, sampler, scheduler,
        StoppingCriterion::temp_zero, *neigh
    );

    std::cout << "\nAvvio ottimizzazione..." << std::endl;
    std::cout << "START: (" << start.getX() << ", "
                            << start.getY() << ", "
                            << start.getZ() << ")" << std::endl;
    std::cout << "END:   (" << end.getX()   << ", "
                            << end.getY()   << ", "
                            << end.getZ()   << ")" << std::endl;

    const auto result = ssa.run(drone_objective, startPoint, policy);

    // ── 9. Output ─────────────────────────────────────────────────
    std::cout << "\n=== PERCORSO OTTIMIZZATO ===" << std::endl;
    std::cout << "W0 (start): ("
              << start.getX() << ", "
              << start.getY() << ", "
              << start.getZ() << ")" << std::endl;

    for (int i = 0; i < NWaypoints; ++i) {
        std::cout << "W" << i+1 << ": ("
                  << result[i*3+0] << ", "
                  << result[i*3+1] << ", "
                  << result[i*3+2] << ")" << std::endl;
    }

    std::cout << "W" << NWaypoints+1 << " (end): ("
              << end.getX() << ", "
              << end.getY() << ", "
              << end.getZ() << ")" << std::endl;

    // fitness e distanza finali
    PointsList finalPath;
    finalPath.addPoint(start);
    for (int i = 0; i < NWaypoints; ++i)
        finalPath.addPoint(Point(result[i*3+0], result[i*3+1], result[i*3+2], -1));
    finalPath.addPoint(end);

    std::cout << "\nFitness finale:      " << fitness.evaluate(finalPath) << std::endl;
    std::cout << "Lunghezza percorso:  " << finalPath.totalDistance()    << std::endl;


    PointsListExporter::writeKML(finalPath, "path.kml");

    // ── 10. Cleanup ───────────────────────────────────────────────
    for (auto obs : obstacles) delete obs;

    return 0;
}