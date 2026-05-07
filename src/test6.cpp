#include "structures/fitness/fitnessFunction.hpp"
#include "structures/obstacles/cylinderObstacle.hpp"
#include "structures/pointsList.hpp"
#include "structures/kmeans.hpp"
#include "structures/lhs/lhs.hpp"
#include "structures/tsp/tspSA.hpp"
#include "exporters/pointsListExporter.hpp"
#include "exporters/pointsListReader.hpp"

#include "geometry/point.hpp"
#include "domain/domain.hpp"
#include "scheduler/scheduler.hpp"
#include "sampler/sampler_base.hpp"
#include "sampler/local_sampler.hpp"
#include "criterion/criterion.hpp"
#include "algorithms/annealing_policy.hpp"
#include "algorithms/serial_simulated_annealing.hpp"

#include <iostream>
#include <vector>
#include <memory>
#include <cmath>

int main() {

    // ════════════════════════════════════════════════════════════
    // 0. CONFIGURAZIONE MISSIONE
    // ════════════════════════════════════════════════════════════
    const int     K          = 3;
    const int     NWaypoints = 4;
    constexpr int Dim        = NWaypoints * 3;

    const double zMin = 100.0, zMax = 200.0;

    // origine geografica per conversione GPS → metri
    const double lat0 = 45.4641;
    const double lon0 = 9.1919;
    const double metersPerDegLat = 111320.0;
    const double metersPerDegLon = 111320.0 * std::cos(lat0 * M_PI / 180.0);

    // ════════════════════════════════════════════════════════════
    // 1. LETTURA INPUT
    // ════════════════════════════════════════════════════════════
    std::cout << "=== 1. Lettura dati ===" << std::endl;
    PointsList allPoints = PointsListReader::readCSV("../data/input2.csv");
    std::cout << "Punti letti: " << allPoints.size() << std::endl;

    if (allPoints.size() == 0) {
        std::cerr << "Errore: nessun punto letto." << std::endl;
        return 1;
    }

    // ── conversione GPS → metri ───────────────────────────────
    for (int i = 0; i < allPoints.size(); ++i) {
        Point& p = allPoints.extractPoint(i);
        double x = (p.getX() - lon0) * metersPerDegLon;
        double y = (p.getY() - lat0) * metersPerDegLat;
        p.setX(x);
        p.setY(y);
    }
    std::cout << "Coordinate convertite in metri." << std::endl;

    // ════════════════════════════════════════════════════════════
    // 2. OSTACOLI E FITNESS
    // ════════════════════════════════════════════════════════════
    std::vector<std::shared_ptr<Obstacle>> obstacles;
    obstacles.push_back(std::make_shared<CylinderObstacle>(
        Point(200.0, 300.0, 0.0, -1), 50.0, 200.0, 60.0));
    obstacles.push_back(std::make_shared<CylinderObstacle>(
        Point(-300.0, -200.0, 0.0, -1), 50.0, 200.0, 60.0));

    FitnessWeights weights;
    weights.b1   = 5.0;
    weights.b2   = 10.0;
    weights.b3   = 1.0;
    weights.b4   = 5.0;
    weights.a1   = 1.0;
    weights.a2   = 1.0;
    weights.hMin = zMin;
    weights.hMax = zMax;

    FitnessFunction fitness(obstacles, weights);

    // ════════════════════════════════════════════════════════════
    // 3. K-MEANS
    // ════════════════════════════════════════════════════════════
    std::cout << "\n=== 2. K-Means (K=" << K << ") ===" << std::endl;
    KMeans kmeans(K, 100);
    kmeans.run(allPoints);
    kmeans.printResults();

    for (int k = 0; k < K; ++k) {
        PointsList check = allPoints.extractCluster(k);
        std::cout << "  Cluster " << k << ": "
                  << check.size() << " punti" << std::endl;
    }

    // ════════════════════════════════════════════════════════════
    // 4. LOOP SUI CLUSTER
    // ════════════════════════════════════════════════════════════
    PointsList fullPath;

    for (int k = 0; k < K; ++k) {

        std::cout << "\n=== Drone " << k
                  << " — Cluster " << k << " ===" << std::endl;

        PointsList cluster = allPoints.extractCluster(k);

        if (cluster.size() == 0) {
            std::cout << "  Cluster vuoto, skip." << std::endl;
            continue;
        }
        std::cout << "  Punti nel cluster: " << cluster.size() << std::endl;

        // ── 4a. TSP SA ────────────────────────────────────────
        std::cout << "  Avvio TSP SA..." << std::endl;

        std::shared_ptr<TransitionCriterion> tspCriterion =
            std::make_shared<MetropolisCriterion>();
        std::shared_ptr<ParamScheduler> tspScheduler =
            std::make_shared<ExponentialScheduler>(100.0, 0.01, 200, 0.95);

        TspSA tspSA(fitness, tspCriterion, tspScheduler, 500);
        PointsList orderedCluster = tspSA.run(cluster);

        std::cout << "  Ordine trovato:" << std::endl;
        for (int i = 0; i < orderedCluster.size(); ++i) {
            std::cout << "    P" << i << ": ("
                      << orderedCluster.getX(i) << ", "
                      << orderedCluster.getY(i) << ", "
                      << orderedCluster.getZ(i) << ")" << std::endl;
        }

        // ── 4b. SA CONTINUO ───────────────────────────────────
        if (orderedCluster.size() < 2) {
            fullPath.addPoint(Point(
                orderedCluster.getX(0),
                orderedCluster.getY(0),
                orderedCluster.getZ(0), -1));
            continue;
        }

        std::cout << "  Avvio SA continuo per traiettorie..." << std::endl;

        for (int i = 0; i + 1 < orderedCluster.size(); ++i) {

            Point segStart(
                orderedCluster.getX(i),
                orderedCluster.getY(i),
                orderedCluster.getZ(i), -1);

            Point segEnd(
                orderedCluster.getX(i + 1),
                orderedCluster.getY(i + 1),
                orderedCluster.getZ(i + 1), -1);

            std::cout << "    Segmento " << i << ": ("
                      << segStart.getX() << ","
                      << segStart.getY() << ") → ("
                      << segEnd.getX()   << ","
                      << segEnd.getY()   << ")" << std::endl;

            // ── bounds dinamici per questo cluster ────────────
            double cxMin = orderedCluster.getXMin();
            double cxMax = orderedCluster.getXMax();
            double cyMin = orderedCluster.getYMin();
            double cyMax = orderedCluster.getYMax();

            double marginX = (cxMax - cxMin) * 0.2;
            double marginY = (cyMax - cyMin) * 0.2;
            if (marginX < 50.0) marginX = 50.0;
            if (marginY < 50.0) marginY = 50.0;

            cxMin -= marginX;  cxMax += marginX;
            cyMin -= marginY;  cyMax += marginY;

            // ── LHS inizializzato sui bounds del cluster ──────
            Lhs lhs(cxMin, cxMax, cyMin, cyMax, zMin, zMax, NWaypoints, -1);
            point_nd<Dim> startPoint = lhs.toPointNd<NWaypoints>();

            // ── funzione obiettivo ────────────────────────────
            auto segObjective = [&](const point_nd<Dim>& p) -> double {
                PointsList path;
                path.addPoint(segStart);
                for (int w = 0; w < NWaypoints; ++w)
                    path.addPoint(Point(p[w*3+0], p[w*3+1], p[w*3+2], -1));
                path.addPoint(segEnd);
                double cost = fitness.evaluate(path);
                if (std::isinf(cost) || std::isnan(cost)) return 1e12;
                return cost;
            };

            // ── dominio SA sui bounds del cluster ─────────────
            point_nd<Dim> domCenter = 0.0;
            for (int w = 0; w < NWaypoints; ++w) {
                domCenter[w*3+0] = (cxMin + cxMax) / 2.0;
                domCenter[w*3+1] = (cyMin + cyMax) / 2.0;
                domCenter[w*3+2] = (zMin  + zMax)  / 2.0;
            }

            const double domRadius = std::sqrt(
                std::pow(cxMax - cxMin, 2) +
                std::pow(cyMax - cyMin, 2) +
                std::pow(zMax  - zMin,  2));

            auto domain = RnDomainFactory::make_sphere(domRadius, domCenter);
            auto neigh  = std::make_shared<CircleNeighbourhood<Dim>>(
                domain, domRadius * 0.1);

            auto sampler   = std::make_shared<LocalSampler<CircleNeighbourhood<Dim>>>(0.1);
            auto criterion = std::make_shared<MetropolisCriterion>();
            std::shared_ptr<ParamScheduler> scheduler =
                std::make_shared<ExponentialScheduler>(100.0, 0.01, 500, 0.95);

            neigh->from(startPoint, startPoint);

            AnnealingExecutionPolicy<LocalSampler<CircleNeighbourhood<Dim>>> policy(
                criterion, sampler, scheduler,
                StoppingCriterion::temp_zero, *neigh);

            SerialSimulatedAnnealing ssa(1000);
            const auto result = ssa.run(segObjective, startPoint, policy);

            // aggiungi start + waypoint ottimali
            fullPath.addPoint(segStart);
            for (int w = 0; w < NWaypoints; ++w)
                fullPath.addPoint(Point(
                    result[w*3+0], result[w*3+1], result[w*3+2], -1));
        }

        // aggiungi ultimo punto del cluster
        int last = orderedCluster.size() - 1;
        fullPath.addPoint(Point(
            orderedCluster.getX(last),
            orderedCluster.getY(last),
            orderedCluster.getZ(last), -1));
    }

    // ════════════════════════════════════════════════════════════
    // 5. CONVERSIONE METRI → GPS E EXPORT KML
    // ════════════════════════════════════════════════════════════
    std::cout << "\n=== 5. Export KML ===" << std::endl;
    std::cout << "Punti totali nel percorso: " << fullPath.size() << std::endl;

    // riconverti da metri a GPS prima di esportare
    for (int i = 0; i < fullPath.size(); ++i) {
        Point& p = fullPath.extractPoint(i);
        double lon = lon0 + (p.getX() / metersPerDegLon);
        double lat = lat0 + (p.getY() / metersPerDegLat);
        p.setX(lon);
        p.setY(lat);
    }

    PointsListExporter::writeKML(fullPath, "../output/path.kml");
    std::cout << "File salvato in ../output/path.kml" << std::endl;

    // ════════════════════════════════════════════════════════════
    // 6. CLEANUP
    // ════════════════════════════════════════════════════════════

    return 0;
}