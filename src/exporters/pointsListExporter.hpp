#ifndef POINTSLISTEXPORTER_HPP
#define POINTSLISTEXPORTER_HPP

#include <string>
#include <vector>
#include "pointsList.hpp"

class PointsListExporter {
public:
    // Percorso singolo — linea rossa (comportamento originale)
    static void writeKML(const PointsList& points,
                         const std::string& filename);

    // Percorsi multi-cluster — un colore per cluster, raggruppati in folder
    static void writeKMLClusters(const std::vector<PointsList>& clusters,
                                 const std::string& filename);

    // Confronto fitness
    static void printComparison(double totalSAFit, double totalDRSTASAFit);
};

#endif