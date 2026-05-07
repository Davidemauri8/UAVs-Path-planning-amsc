#include "structures/geo/geoUtils.hpp"
#include "structures/functions/fitnessUtilities.hpp"
#include "structures/segment/segmentOptimizer.hpp"
#include "structures/kmeans.hpp"
#include "structures/tsp/tspSA.hpp"
#include "exporters/pointsListExporter.hpp"
#include "exporters/pointsListReader.hpp"

#include "criterion/criterion.hpp"
#include "scheduler/scheduler.hpp"

#include <iostream>
#include <memory>

template<int NWaypoints>
void processClusters(
    PointsList&            allPoints,
    int                    K,
    const FitnessFunction& fitness,
    DRSTASA::Config&       drsCfg,
    double                 zMin,
    double                 zMax,
    PointsList&            saPath,
    PointsList&            drstasaPath,
    double&                totalSAFit,
    double&                totalDRSTASAFit)
{
    for (int k = 0; k < K; ++k) {
        PointsList cluster = allPoints.extractCluster(k);
        if (cluster.size() == 0) continue;

        auto tspCrit  = std::make_shared<MetropolisCriterion>();
        auto tspSched = std::make_shared<ExponentialScheduler>(100.0, 0.01, 200, 0.95);
        PointsList ordered = TspSA(fitness, tspCrit, tspSched, 500).run(cluster);

        if (ordered.size() < 2) {
            Point p(ordered.getX(0), ordered.getY(0), ordered.getZ(0), -1);
            saPath.addPoint(p); drstasaPath.addPoint(p);
            continue;
        }

        for (int i = 0; i + 1 < ordered.size(); ++i) {
            Point segStart(ordered.getX(i),   ordered.getY(i),   ordered.getZ(i),   -1);
            Point segEnd  (ordered.getX(i+1), ordered.getY(i+1), ordered.getZ(i+1), -1);

            SegmentBounds bounds  = computeBounds(ordered);
            drsCfg.xMin = bounds.xMin; drsCfg.xMax = bounds.xMax;
            drsCfg.yMin = bounds.yMin; drsCfg.yMax = bounds.yMax;

            SegmentResult res = optimizeSegment<NWaypoints>(
                segStart, segEnd, fitness, bounds, zMin, zMax, drsCfg);

            totalSAFit      += res.saFit;
            totalDRSTASAFit += res.drstasaFit;
            appendPath(saPath,      res.saPath);
            appendPath(drstasaPath, res.drstasaPath);

            std::cout << "  seg " << i << " SA=" << res.saFit
                      << "  DRSTASA=" << res.drstasaFit << "\n";
        }

        int last = ordered.size() - 1;
        saPath.addPoint(     Point(ordered.getX(last), ordered.getY(last), ordered.getZ(last), -1));
        drstasaPath.addPoint(Point(ordered.getX(last), ordered.getY(last), ordered.getZ(last), -1));
    }
}

int main() {
    constexpr int K          = 3;
    constexpr int NWaypoints = 4;
    const double  zMin = 100.0, zMax = 200.0;
    const double  lat0 =  45.4641, lon0 = 9.1919;

    // 1. Input
    PointsList allPoints = PointsListReader::readCSV("../data/input.csv");
    if (allPoints.size() == 0) { std::cerr << "Nessun punto letto.\n"; return 1; }
    GeoUtils::toMeters(allPoints, lat0, lon0);

    // 2. Fitness e configurazione DRSTASA
    FitnessFunction fitness = makeDefaultFitness(zMin, zMax);
    DRSTASA::Config drsCfg  = GetConfigurationDRST(NWaypoints);
    drsCfg.zMin = zMin; drsCfg.zMax = zMax;

    // 3. K-Means
    KMeans kmeans(K, 100);
    kmeans.run(allPoints);

    // 4. Loop sui cluster
    PointsList saPath, drstasaPath;
    double totalSAFit = 0.0, totalDRSTASAFit = 0.0;
    processClusters<NWaypoints>(allPoints, K, fitness, drsCfg, zMin, zMax,
                                saPath, drstasaPath, totalSAFit, totalDRSTASAFit);

    // 5. Riepilogo
    PointsListExporter::printComparison(totalSAFit, totalDRSTASAFit);

    // 6. Export KML
    GeoUtils::toGPS(saPath,      lat0, lon0);
    GeoUtils::toGPS(drstasaPath, lat0, lon0);
    PointsListExporter::writeDroneDAE("../output/drone.dae");
    PointsListExporter::writeKML(saPath,      "../output/sa_path.kml");
    PointsListExporter::writeKML(drstasaPath, "../output/drstasa_path.kml");
    PointsListExporter::writeKMLWithDroneTrack(drstasaPath, "../output/drstasa_animated.kml");

    return 0;
}
