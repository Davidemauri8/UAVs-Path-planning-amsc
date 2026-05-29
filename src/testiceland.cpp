#include "structures/functions/fitnessUtilities.hpp"
#include "structures/obstacles/cylinderObstacle.hpp"
#include "structures/geo/geoUtils.hpp"
#include "structures/pipeline/pipelineRunner.hpp"
#include "structures/kmeans.hpp"
#include "exporters/pointsListReader.hpp"
#include "exporters/pointsListExporter.hpp"
#include <iostream>
#include <cmath>

int main()
{
    constexpr int NWaypoints = 4;
    constexpr int K          = 4;
    const double  zMin = 800.0, zMax = 950.0;

    const GeoOrigin origin = {63.985, -22.605};

    // ── 1. Caricamento CSV ────────────────────────────────────────────────────
    std::cout << "[1/5] Caricamento input_iceland.csv ... " << std::flush;
    PointsList allPoints = PointsListReader::readCSV("../data/input_iceland.csv");
    if (allPoints.size() == 0) {
        std::cerr << "ERRORE: file non trovato o vuoto.\n";
        return 1;
    }
    GeoUtils::toMeters(allPoints, origin);
    std::cout << allPoints.size() << " punti caricati e convertiti in metri.\n";

    // ── 2. Fitness ────────────────────────────────────────────────────────────
    std::cout << "[2/5] Costruzione fitness ... " << std::flush;
    FitnessFunction fitness = makeDefaultFitness(zMin, zMax);
    std::cout << "OK.\n";

    // Pre-convert obstacles to GPS circles for the comparison KML
    const double kMPDLat = 111320.0;
    const double kMPDLon = kMPDLat * std::cos(origin.lat0 * M_PI / 180.0);
    std::vector<PointsListExporter::ObstacleCircle> obsCircles;
    for (const auto& obs : fitness.getObstacles()) {
        const auto* cyl = dynamic_cast<const CylinderObstacle*>(obs.get());
        if (!cyl) continue;
        PointsListExporter::ObstacleCircle oc;
        oc.centerLon    = origin.lon0 + cyl->getX() / kMPDLon;
        oc.centerLat    = origin.lat0 + cyl->getY() / kMPDLat;
        oc.radiusDegLon = cyl->getRadius() / kMPDLon;
        oc.radiusDegLat = cyl->getRadius() / kMPDLat;
        obsCircles.push_back(oc);
    }

    // ── 3. K-Means clustering ─────────────────────────────────────────────────
    std::cout << "[3/5] K-Means clustering (K=" << K << ") ... " << std::flush;
    KMeans(K, 100).run(allPoints);
    std::cout << "OK.\n";

    // ── 4. Ottimizzazione seriale ─────────────────────────────────────────────
    std::cout << "[4/5] Ottimizzazione SA + DRSTASA (seriale, NWaypoints=" << NWaypoints << ") ...\n";
    BenchmarkResult res = runPipelineOptimization<NWaypoints>(
        allPoints, K, fitness, zMin, zMax, /*numThreads=*/K);
    std::cout << "      Completata in " << res.wallTime << "s.\n";
    std::cout << "      Fitness SA:      " << res.saFit      << "\n";
    std::cout << "      Fitness DRSTASA: " << res.drstasaFit << "\n";
    PointsListExporter::printComparison(res.saFit, res.drstasaFit);

    // ── 5. Export KML ─────────────────────────────────────────────────────────
    std::cout << "[5/5] Export KML ... " << std::flush;

    // Percorsi ottimizzati unificati
    PointsList saGPS      = res.saPath;
    PointsList drstasaGPS = res.drstasaPath;
    GeoUtils::toGPS(saGPS,      origin);
    GeoUtils::toGPS(drstasaGPS, origin);
    PointsListExporter::writeKML(saGPS,      "../output/iceland_sa.kml");
    PointsListExporter::writeKML(drstasaGPS, "../output/iceland_drstasa.kml");

    // Waypoint originali per-cluster (ordine TSP)
    std::vector<PointsList> clustersGPS = res.orderedClusters;
    for (auto& cl : clustersGPS) GeoUtils::toGPS(cl, origin);
    PointsListExporter::writeKMLClusters(clustersGPS, "../output/iceland_clusters.kml");

    // Percorsi SA per-cluster: linea + label solo sui target point originali
    std::vector<PointsList> saClustersGPS = res.saPerCluster;
    for (auto& cl : saClustersGPS) GeoUtils::toGPS(cl, origin);
    PointsListExporter::writeKMLClustersWithTargets(saClustersGPS, clustersGPS, "../output/iceland_sa_clusters.kml");

    // Percorsi DRSTASA per-cluster: linea + label solo sui target point originali
    std::vector<PointsList> drstasaClustersGPS = res.drstasaPerCluster;
    for (auto& cl : drstasaClustersGPS) GeoUtils::toGPS(cl, origin);
    PointsListExporter::writeKMLClustersWithTargets(drstasaClustersGPS, clustersGPS, "../output/iceland_drstasa_clusters.kml");

    PointsListExporter::writeCSVClusters(
        clustersGPS, saClustersGPS, drstasaClustersGPS,
        "../output/iceland_clusters.csv");

    PointsListExporter::writeKMLComparison(
        saClustersGPS, drstasaClustersGPS, clustersGPS,
        obsCircles, "../output/iceland_comparison.kml");

    std::cout << "OK.\n";
    std::cout << "      -> ../output/iceland_sa.kml\n";
    std::cout << "      -> ../output/iceland_drstasa.kml\n";
    std::cout << "      -> ../output/iceland_clusters.kml\n";
    std::cout << "      -> ../output/iceland_sa_clusters.kml\n";
    std::cout << "      -> ../output/iceland_drstasa_clusters.kml\n";
    std::cout << "      -> ../output/iceland_clusters.csv\n";
    std::cout << "      -> ../output/iceland_comparison.kml\n";

    return 0;
}
