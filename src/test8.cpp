#include "structures/fitness/fitnessFunction.hpp"
#include "structures/obstacles/cylinderObstacle.hpp"
#include "structures/pointsList.hpp"
#include "structures/kmeans.hpp"
#include "structures/lhs/lhs.hpp"
#include "structures/tsp/tspSA.hpp"
#include "structures/drstasa/drstasa.hpp"
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

    const double lat0 = 45.4641;
    const double lon0 = 9.1919;
    const double metersPerDegLat = 111320.0;
    const double metersPerDegLon = 111320.0 * std::cos(lat0 * M_PI / 180.0);

    // ════════════════════════════════════════════════════════════
    // 1. LETTURA INPUT
    // ════════════════════════════════════════════════════════════
    std::cout << "=== 1. Lettura dati ===" << std::endl;
    PointsList allPoints = PointsListReader::readCSV("../data/input.csv");
    std::cout << "Punti letti: " << allPoints.size() << std::endl;

    if (allPoints.size() == 0) {
        std::cerr << "Errore: nessun punto letto." << std::endl;
        return 1;
    }

    for (int i = 0; i < allPoints.size(); ++i) {
        Point& p = allPoints.extractPoint(i);
        double x = (p.getX() - lon0) * metersPerDegLon;
        double y = (p.getY() - lat0) * metersPerDegLat;
        p.setX(x);
        p.setY(y);
    }

    // ════════════════════════════════════════════════════════════
    // 2. OSTACOLI E FITNESS
    // ════════════════════════════════════════════════════════════
    std::vector<Obstacle*> obstacles;
    obstacles.push_back(new CylinderObstacle(
        Point(200.0, 300.0, 0.0, -1), 50.0, 200.0, 60.0));
    obstacles.push_back(new CylinderObstacle(
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

    // ════════════════════════════════════════════════════════════
    // 4. PARAMETRI DRSTASA (settati qui nel main)
    // ════════════════════════════════════════════════════════════
    // Nota: popSize e maxIter ridotti rispetto al paper (500/2000)
    //       per mantenere tempi accettabili nel test.
    //       Incrementarli per risultati più vicini al paper.
    DRSTASA::Config drsCfg;
    drsCfg.popSize    = 20;
    drsCfg.maxIter    = 300;
    drsCfg.T0         = 100.0;
    drsCfg.alpha      = 0.93;
    drsCfg.p          = 0.5;
    drsCfg.C0         = 2.0;
    // eps calibrati per coordinate in metri (dominio ~500-1000m):
    // eps_rot/trans scalano con norm del path → valore alto
    // eps_scale/axis sono moltiplicativi → valore piccolo (±5%)
    drsCfg.eps_rot    = 150.0;
    drsCfg.eps_trans  = 100.0;
    drsCfg.eps_scale  = 0.05;
    drsCfg.eps_axis   = 0.05;
    drsCfg.nWaypoints = NWaypoints;
    // xMin/xMax/yMin/yMax saranno aggiornati per ogni segmento

    // ════════════════════════════════════════════════════════════
    // 5. LOOP SUI CLUSTER
    // ════════════════════════════════════════════════════════════
    PointsList saPath;       // percorso con SA classico
    PointsList drstasaPath;  // percorso con DRSTASA
    double totalSAFit = 0.0, totalDRSTASAFit = 0.0;

    for (int k = 0; k < K; ++k) {

        std::cout << "\n=== Drone " << k
                  << " — Cluster " << k << " ===" << std::endl;

        PointsList cluster = allPoints.extractCluster(k);
        if (cluster.size() == 0) { std::cout << "  Cluster vuoto." << std::endl; continue; }

        // ── 5a. TSP SA (identico per entrambi) ───────────────────
        auto tspCrit  = std::make_shared<MetropolisCriterion>();
        auto tspSched = std::make_shared<ExponentialScheduler>(100.0, 0.01, 200, 0.95);
        TspSA tspSA(fitness, tspCrit, tspSched, 500);
        PointsList ordered = tspSA.run(cluster);

        if (ordered.size() < 2) {
            Point p(ordered.getX(0), ordered.getY(0), ordered.getZ(0), -1);
            saPath.addPoint(p);
            drstasaPath.addPoint(p);
            continue;
        }

        // ── 5b. Ottimizzazione continua per ogni segmento ─────────
        for (int i = 0; i + 1 < ordered.size(); ++i) {

            Point segStart(ordered.getX(i),   ordered.getY(i),   ordered.getZ(i),   -1);
            Point segEnd  (ordered.getX(i+1), ordered.getY(i+1), ordered.getZ(i+1), -1);

            std::cout << "  Segmento " << i << ": ("
                      << segStart.getX() << "," << segStart.getY() << ") → ("
                      << segEnd.getX()   << "," << segEnd.getY()   << ")" << std::endl;

            // Bounds dinamici per questo segmento
            double cxMin = ordered.getXMin(), cxMax = ordered.getXMax();
            double cyMin = ordered.getYMin(), cyMax = ordered.getYMax();
            double marginX = std::max(50.0, (cxMax-cxMin)*0.2);
            double marginY = std::max(50.0, (cyMax-cyMin)*0.2);
            cxMin -= marginX;  cxMax += marginX;
            cyMin -= marginY;  cyMax += marginY;

            // ── SA CLASSICO ──────────────────────────────────────
            Lhs lhs(cxMin, cxMax, cyMin, cyMax, zMin, zMax, NWaypoints, -1);
            point_nd<Dim> startPoint = lhs.toPointNd<NWaypoints>();

            auto segObjective = [&](const point_nd<Dim>& p) -> double {
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
                domCenter[w*3+0] = (cxMin+cxMax)/2.0;
                domCenter[w*3+1] = (cyMin+cyMax)/2.0;
                domCenter[w*3+2] = (zMin+zMax)/2.0;
            }
            double domRadius = std::sqrt(std::pow(cxMax-cxMin,2) +
                                         std::pow(cyMax-cyMin,2) +
                                         std::pow(zMax-zMin,2));

            auto domain    = RnDomainFactory::make_sphere(domRadius, domCenter);
            auto saNeigh   = std::make_shared<CircleNeighbourhood<Dim>>(domain, domRadius*0.1);
            auto saSampler = std::make_shared<LocalSampler<CircleNeighbourhood<Dim>>>(0.1);
            auto saCrit    = std::make_shared<MetropolisCriterion>();
            // stab_it=1, maxIter=300 → 300 valutazioni per iter di temperatura
            // DRSTASA: popSize=20, maxIter=300 → 6000 valutazioni (20× di più)
            // Per parificare: SA maxIter=6000, stab_it=1 ≈ stessa budget totale
            auto saSched   = std::make_shared<ExponentialScheduler>(100.0, 0.01, 1, 0.95);
            saNeigh->from(startPoint, startPoint);

            AnnealingExecutionPolicy<LocalSampler<CircleNeighbourhood<Dim>>> saPolicy(
                saCrit, saSampler, saSched, StoppingCriterion::temp_zero, *saNeigh);
            SerialSimulatedAnnealing ssa(6000);
            auto saResult = ssa.run(segObjective, startPoint, saPolicy);

            double saFit = segObjective(saResult);
            totalSAFit += saFit;

            saPath.addPoint(segStart);
            for (int w = 0; w < NWaypoints; ++w)
                saPath.addPoint(Point(saResult[w*3+0], saResult[w*3+1],
                                     saResult[w*3+2], -1));

            // ── DRSTASA ──────────────────────────────────────────
            drsCfg.xMin = cxMin; drsCfg.xMax = cxMax;
            drsCfg.yMin = cyMin; drsCfg.yMax = cyMax;
            drsCfg.zMin = zMin;  drsCfg.zMax = zMax;

            DRSTASA drstasa(fitness, drsCfg);
            PointsList drstasaSeg = drstasa.run(segStart, segEnd);
            double drsFit = drstasa.lastBestFit();
            totalDRSTASAFit += drsFit;

            // drstasaSeg include già start+waypoints+end: aggiungi solo start+waypoints
            for (int w = 0; w < drstasaSeg.size() - 1; ++w)
                drstasaPath.addPoint(Point(drstasaSeg.getX(w), drstasaSeg.getY(w),
                                          drstasaSeg.getZ(w), -1));

            std::cout << "    SA fitness:      " << saFit    << std::endl;
            std::cout << "    DRSTASA fitness: " << drsFit   << std::endl;
        }

        // Aggiunge l'ultimo punto del cluster
        int last = ordered.size() - 1;
        Point lastPt(ordered.getX(last), ordered.getY(last), ordered.getZ(last), -1);
        saPath.addPoint(lastPt);
        drstasaPath.addPoint(lastPt);
    }

    // ════════════════════════════════════════════════════════════
    // 6. RIEPILOGO CONFRONTO
    // ════════════════════════════════════════════════════════════
    std::cout << "\n=== CONFRONTO FINALE ===" << std::endl;
    std::cout << "Fitness totale SA:      " << totalSAFit      << std::endl;
    std::cout << "Fitness totale DRSTASA: " << totalDRSTASAFit << std::endl;
    if (totalDRSTASAFit < totalSAFit)
        std::cout << ">> DRSTASA migliore di SA del "
                  << 100.0*(totalSAFit-totalDRSTASAFit)/totalSAFit << "%" << std::endl;
    else
        std::cout << ">> SA migliore (o equivalente) di DRSTASA" << std::endl;

    // ════════════════════════════════════════════════════════════
    // 7. EXPORT KML
    // ════════════════════════════════════════════════════════════
    // Riconversione metri → GPS
    auto toGPS = [&](PointsList& path) {
        for (int i = 0; i < path.size(); ++i) {
            Point& p = path.extractPoint(i);
            p.setX(lon0 + p.getX() / metersPerDegLon);
            p.setY(lat0 + p.getY() / metersPerDegLat);
        }
    };
    toGPS(saPath);
    toGPS(drstasaPath);

    PointsListExporter::writeDroneDAE("../output/drone.dae");

    PointsListExporter::writeKML(saPath,      "../output/sa_path.kml");
    PointsListExporter::writeKML(drstasaPath, "../output/drstasa_path.kml");
    PointsListExporter::writeKMLWithDroneTrack(drstasaPath, "../output/drstasa_animated.kml");

    std::cout << "\nExport completato:" << std::endl;
    std::cout << "  ../output/sa_path.kml" << std::endl;
    std::cout << "  ../output/drstasa_path.kml" << std::endl;
    std::cout << "  ../output/drstasa_animated.kml  (drone animato)" << std::endl;

    for (auto obs : obstacles) delete obs;
    return 0;
}
