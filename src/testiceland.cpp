#include "scenarios/iceland.hpp"
#include "structures/pipeline/pipelineRunner.hpp"
#include "structures/kmeans.hpp"
#include "exporters/pointsListReader.hpp"
#include "exporters/pointsListExporter.hpp"
#include <iostream>

int main()
{
    constexpr int NWaypoints = 5;
    constexpr int K          = 4;
    const double  zMin = 60.0, zMax = 200.0;

    const GeoOrigin origin = icelandOrigin();

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
    std::cout << "[2/5] Costruzione fitness (4 ostacoli vulcanici) ... " << std::flush;
    FitnessFunction fitness = makeIcelandFitness(zMin, zMax);
    std::cout << "OK.\n";

    // ── 3. K-Means clustering ─────────────────────────────────────────────────
    std::cout << "[3/5] K-Means clustering (K=" << K << ") ... " << std::flush;
    KMeans(K, 100).run(allPoints);
    std::cout << "OK.\n";

    // ── 4. Ottimizzazione seriale ─────────────────────────────────────────────
    std::cout << "[4/5] Ottimizzazione SA + DRSTASA (seriale, NWaypoints=" << NWaypoints << ") ...\n";
    BenchmarkResult res = runPipelineOptimization<NWaypoints>(
        allPoints, K, fitness, zMin, zMax, /*numThreads=*/1);
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

    // Percorsi SA per-cluster
    std::vector<PointsList> saClustersGPS = res.saPerCluster;
    for (auto& cl : saClustersGPS) GeoUtils::toGPS(cl, origin);
    PointsListExporter::writeKMLClusters(saClustersGPS, "../output/iceland_sa_clusters.kml");

    // Percorsi DRSTASA per-cluster
    std::vector<PointsList> drstasaClustersGPS = res.drstasaPerCluster;
    for (auto& cl : drstasaClustersGPS) GeoUtils::toGPS(cl, origin);
    PointsListExporter::writeKMLClusters(drstasaClustersGPS, "../output/iceland_drstasa_clusters.kml");

    std::cout << "OK.\n";
    std::cout << "      -> ../output/iceland_sa.kml\n";
    std::cout << "      -> ../output/iceland_drstasa.kml\n";
    std::cout << "      -> ../output/iceland_clusters.kml\n";
    std::cout << "      -> ../output/iceland_sa_clusters.kml\n";
    std::cout << "      -> ../output/iceland_drstasa_clusters.kml\n";

    return 0;
}
