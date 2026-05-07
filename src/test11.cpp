#include "structures/kmeans.hpp"
#include "structures/tsp/tspSA.hpp"
#include "structures/geo/geoUtils.hpp"
#include "structures/segment/segmentOptimizer.hpp"
#include "structures/segment/segmentAppender.hpp"
#include "structures/functions/fitnessUtilities.hpp"
#include "exporters/pointsListExporter.hpp"
#include "exporters/pointsListReader.hpp"
#include <iostream>

int main() {
    //Set parameters
    constexpr int K = 3, NWaypoints = 4;
    const double zMin = 100.0, zMax = 200.0;

    //Load Target Points and conversion
    PointsList allPoints = PointsListReader::readCSV("../data/earthquake_sim_200.csv");
    if (allPoints.size() == 0) { std::cerr << "Errore: nessun punto letto.\n"; return 1; }
    GeoOrigin origin{45.4641, 9.1919};
    GeoUtils::toMeters(allPoints, origin);

    //Fitness function setting zmin and zmax
    FitnessFunction fitness = makeDefaultFitness(zMin, zMax);
    //Configure Kmeans with numbere of cluster and nMax iterations
    KMeans kmeans(K, 100);
    kmeans.run(allPoints);

    //Configure Drstasa an SA
    const DRSTASA::Config drsCfg(NWaypoints, zMin, zMax);
    PointsList saPath, drstasaPath;
    double totalSAFit = 0.0, totalDRSTASAFit = 0.0;

    for (int k = 0; k < K; ++k) {
        PointsList cluster = allPoints.extractCluster(k);
        if (cluster.size() == 0) continue;
        if (cluster.size() < 2) {
            Point p(cluster.getX(0), cluster.getY(0), cluster.getZ(0), -1);
            saPath.addPoint(p); drstasaPath.addPoint(p); continue;
        }
        PointsList ordered = TspSA(fitness, std::make_shared<MetropolisCriterion>(),
            std::make_shared<ExponentialScheduler>(100.0, 0.01, 200, 0.95), 500).run(cluster);
        //All'interno di ogni cluster iteriamo su ogni coppia di punti(cioe un segmento)
            for (int i = 0; i + 1 < ordered.size(); ++i) {
            Point segStart(ordered.getX(i),   ordered.getY(i),   ordered.getZ(i),   -1);
            Point segEnd  (ordered.getX(i+1), ordered.getY(i+1), ordered.getZ(i+1), -1);
            //Definiamo i bounds del percorso con un margine del 20%(guarda segmenteOptimazer)    
            SegmentBounds b = computeBounds(ordered);
            //Lanciamo sa per ogni segmento, auto definisce in modo intellinge se il risultato è int o double    
            auto saRes = runSegmentSA<NWaypoints>(
                segStart, segEnd, fitness,
                b.xMin, b.xMax, b.yMin, b.yMax, zMin, zMax);
            totalSAFit += saRes.fitness;
            appendSASegment<NWaypoints>(saPath, saRes, segStart);

            DRSTASA::Config localCfg = drsCfg;
            localCfg.xMin = b.xMin; localCfg.xMax = b.xMax;
            localCfg.yMin = b.yMin; localCfg.yMax = b.yMax;
            DRSTASA drstasa(fitness, localCfg);
            PointsList drstasaSeg = drstasa.run(segStart, segEnd);
            totalDRSTASAFit += drstasa.lastBestFit();
            appendDRSTASASegment(drstasaPath, drstasaSeg);
        }

        Point lastPt(ordered.getX(ordered.size()-1), ordered.getY(ordered.size()-1), ordered.getZ(ordered.size()-1), -1);
        saPath.addPoint(lastPt); drstasaPath.addPoint(lastPt);
    }

    std::cout << "\nSA: " << totalSAFit << "  DRSTASA: " << totalDRSTASAFit
              << (totalDRSTASAFit < totalSAFit ? " >> DRSTASA migliore" : "") << "\n";
    GeoUtils::toGPS(saPath, origin); GeoUtils::toGPS(drstasaPath, origin);
    PointsListExporter::writeDroneDAE("../output/drone.dae");
    PointsListExporter::writeKML(saPath,      "../output/sa_path.kml");
    PointsListExporter::writeKML(drstasaPath, "../output/drstasa_path.kml");
    PointsListExporter::writeKMLWithDroneTrack(drstasaPath, "../output/drstasa_animated.kml");
    return 0;
}
