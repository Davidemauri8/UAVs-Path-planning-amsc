#include "structures/fitness/fitnessFunction.hpp"
#include "structures/obstacles/cylinderObstacle.hpp"
#include "structures/pointsList.hpp"
#include "structures/tsp/tspSA.hpp"

#include "criterion/criterion.hpp"
#include "scheduler/scheduler.hpp"

#include <iostream>
#include <vector>
#include <memory>

int main() {

    // ── 1. Ostacoli ───────────────────────────────────────────────
    std::vector<Obstacle*> obstacles;
    obstacles.push_back(new CylinderObstacle(
        Point(250.0, 350.0, 0.0, -1), 40.0, 200.0, 50.0));
    obstacles.push_back(new CylinderObstacle(
        Point(350.0, 450.0, 0.0, -1), 40.0, 200.0, 50.0));

    // ── 2. Fitness ────────────────────────────────────────────────
    FitnessWeights weights;
    weights.b1   = 5.0;
    weights.b2   = 10.0;
    weights.b3   = 1.0;
    weights.b4   = 5.0;
    weights.a1   = 1.0;
    weights.a2   = 1.0;
    weights.hMin = 100.0;
    weights.hMax = 200.0;

    FitnessFunction fitness(obstacles, weights);

    // ── 3. Cluster da ordinare ────────────────────────────────────
    PointsList cluster;
    cluster.addPoint(Point(100.0, 200.0, 150.0, 0));
    cluster.addPoint(Point(300.0, 400.0, 150.0, 0));
    cluster.addPoint(Point(200.0, 500.0, 150.0, 0));
    cluster.addPoint(Point(400.0, 300.0, 150.0, 0));
    cluster.addPoint(Point(150.0, 450.0, 150.0, 0));

    // ── 4. Configurazione SA ──────────────────────────────────────
    std::shared_ptr<TransitionCriterion> criterion =
        std::make_shared<MetropolisCriterion>();

    std::shared_ptr<ParamScheduler> scheduler =
        std::make_shared<ExponentialScheduler>(100.0, 0.01, 200, 0.95);

    TspSA tspSA(fitness, criterion, scheduler, 500);

    // ── 5. Esecuzione ─────────────────────────────────────────────
    std::cout << "Avvio TSP SA..." << std::endl;
    PointsList orderedPath = tspSA.run(cluster);

    // ── 6. Output ─────────────────────────────────────────────────
    std::cout << "\n=== ORDINE OTTIMALE ===" << std::endl;
    for (int i = 0; i < orderedPath.size(); ++i) {
        std::cout << "P" << i << ": ("
                  << orderedPath.getX(i) << ", "
                  << orderedPath.getY(i) << ", "
                  << orderedPath.getZ(i) << ")" << std::endl;
    }

    // ── 7. Cleanup ────────────────────────────────────────────────
    for (auto obs : obstacles) delete obs;

    return 0;
}