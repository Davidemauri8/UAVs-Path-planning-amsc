#ifndef POINTSLISTEXPORTER_HPP
#define POINTSLISTEXPORTER_HPP

#include <string>
#include "pointsList.hpp"

class PointsListExporter {
public:
    // Esporta percorso come linea + punti (KML standard).
    static void writeKML(const PointsList& points, const std::string& filename);

    // Esporta percorso con drone 3D animato lungo il percorso via gx:Track.
    // droneModelPath : percorso del .dae relativo al file KML (default: "drone.dae")
    // secondsPerWaypoint : tempo tra waypoint consecutivi (controlla la velocità animazione)
    // droneScale : scala del modello 3D (1.0 = dimensioni reali ~2m)
    static void writeKMLWithDroneTrack(const PointsList& points,
                                       const std::string& filename,
                                       const std::string& droneModelPath = "drone.dae",
                                       int    secondsPerWaypoint = 10,
                                       double droneScale = 3.0);

    // Genera il file drone.dae (modello COLLADA: corpo centrale + 4 bracci).
    static void writeDroneDAE(const std::string& filename);

    /// Stampa il riepilogo comparativo SA vs DRSTASA
    static void printComparison(double totalSAFit, double totalDRSTASAFit);
};

#endif
