#ifndef POINTSLISTEXPORTER_HPP
#define POINTSLISTEXPORTER_HPP

#include <string>
#include <vector>
#include "pointsList.hpp"

class PointsListExporter {
public:
    struct ObstacleCircle {
        double centerLon, centerLat;
        double radiusDegLon, radiusDegLat;
    };

    // Percorso singolo — linea rossa (comportamento originale)
    static void writeKML(const PointsList& points,
                         const std::string& filename);

    // Percorsi multi-cluster — un colore per cluster, raggruppati in folder
    static void writeKMLClusters(const std::vector<PointsList>& clusters,
                                 const std::string& filename);

    // SA/DRSTASA paths: LineString only + text labels for original target points (no icons)
    static void writeKMLClustersWithTargets(const std::vector<PointsList>& paths,
                                            const std::vector<PointsList>& targets,
                                            const std::string& filename);

    // SA (dashed) + DRSTASA (solid) side-by-side per cluster, with obstacle circles
    static void writeKMLComparison(const std::vector<PointsList>& saPaths,
                                   const std::vector<PointsList>& drstasaPaths,
                                   const std::vector<PointsList>& targets,
                                   const std::vector<ObstacleCircle>& obstacles,
                                   const std::string& filename);

    // CSV: target (input TSP-ordered) + waypoint SA + waypoint DRSTASA per cluster
    static void writeCSVClusters(const std::vector<PointsList>& targets,
                                 const std::vector<PointsList>& saWaypoints,
                                 const std::vector<PointsList>& drstasaWaypoints,
                                 const std::string& filename);

    // Confronto fitness
    static void printComparison(double totalSAFit, double totalDRSTASAFit);
};

#endif