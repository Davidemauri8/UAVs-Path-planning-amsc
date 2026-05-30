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

    // Single path (red line)
    static void writeKML(const PointsList& points,
                         const std::string& filename);

    // Multi cluster path 
    static void writeKMLClusters(const std::vector<PointsList>& clusters,
                                 const std::string& filename);

    // SA and DRSTASA paths 
    static void writeKMLClustersWithTargets(const std::vector<PointsList>& paths,
                                            const std::vector<PointsList>& targets,
                                            const std::string& filename);

    // SA (dashed) and DRSTASA (solid) 
    static void writeKMLComparison(const std::vector<PointsList>& saPaths,
                                   const std::vector<PointsList>& drstasaPaths,
                                   const std::vector<PointsList>& targets,
                                   const std::vector<ObstacleCircle>& obstacles,
                                   const std::string& filename);

    // CSV target
    static void writeCSVClusters(const std::vector<PointsList>& targets,
                                 const std::vector<PointsList>& saWaypoints,
                                 const std::vector<PointsList>& drstasaWaypoints,
                                 const std::string& filename);

    // comparison fitness
    static void printComparison(double totalSAFit, double totalDRSTASAFit);
};

#endif