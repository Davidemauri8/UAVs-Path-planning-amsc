#include "pointsListExporter.hpp"
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>

// ─── Palette colori per cluster (KML AABBGGRR) ───────────────────────────────

static const char* CLUSTER_COLORS[] = {
    "ff0000ff",   // C0 — rosso
    "ff00cc00",   // C1 — verde
    "ff00ffff",   // C2 — giallo
    "ff0088ff",   // C3 — arancio
    "ffff0088",   // C4 — viola
    "ff00ccff",   // C5 — oro
};
static const char* CLUSTER_NAMES[] = {
    "Cluster 0", "Cluster 1", "Cluster 2",
    "Cluster 3", "Cluster 4", "Cluster 5",
};
static constexpr int N_COLORS = 6;

// ─── printComparison ─────────────────────────────────────────────────────────

void PointsListExporter::printComparison(double totalSAFit, double totalDRSTASAFit) {
    std::cout << "\n=== CONFRONTO FINALE ===" << std::endl;
    std::cout << "Fitness totale SA:      " << totalSAFit      << std::endl;
    std::cout << "Fitness totale DRSTASA: " << totalDRSTASAFit << std::endl;
    if (totalDRSTASAFit < totalSAFit)
        std::cout << ">> DRSTASA migliore di SA del "
                  << 100.0 * (totalSAFit - totalDRSTASAFit) / totalSAFit
                  << "%" << std::endl;
    else
        std::cout << ">> SA migliore (o equivalente) di DRSTASA" << std::endl;
}

// ─── writeKML (singolo percorso, comportamento originale) ────────────────────

void PointsListExporter::writeKML(const PointsList& points,
                                  const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }
    file << std::fixed << std::setprecision(6);

    file << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    file << "<kml xmlns=\"http://www.opengis.net/kml/2.2\">\n";
    file << "  <Document>\n";
    file << "    <name>Percorso Drone Ottimizzato</name>\n\n";

    file << "    <Style id=\"lineaRossa\">\n";
    file << "      <LineStyle><color>ff0000ff</color><width>4</width></LineStyle>\n";
    file << "    </Style>\n\n";

    file << "    <Style id=\"puntoDiscreto\">\n";
    file << "      <IconStyle>\n";
    file << "        <scale>0.4</scale>\n";
    file << "        <Icon><href>http://maps.google.com/mapfiles/kml/shapes/shaded_dot.png</href></Icon>\n";
    file << "      </IconStyle>\n";
    file << "      <LabelStyle><scale>0</scale></LabelStyle>\n";
    file << "    </Style>\n\n";

    file << "    <Placemark>\n";
    file << "      <name>Traiettoria</name>\n";
    file << "      <styleUrl>#lineaRossa</styleUrl>\n";
    file << "      <LineString>\n";
    file << "        <tessellate>1</tessellate>\n";
    file << "        <altitudeMode>relativeToGround</altitudeMode>\n";
    file << "        <coordinates>\n";
    for (int i = 0; i < points.size(); ++i)
        file << "          " << points.getX(i) << ","
                             << points.getY(i) << ","
                             << points.getZ(i) << "\n";
    file << "        </coordinates>\n";
    file << "      </LineString>\n";
    file << "    </Placemark>\n\n";

    for (int i = 0; i < points.size(); ++i) {
        file << "    <Placemark>\n";
        file << "      <styleUrl>#puntoDiscreto</styleUrl>\n";
        file << "      <Point>\n";
        file << "        <altitudeMode>relativeToGround</altitudeMode>\n";
        file << "        <coordinates>"
             << points.getX(i) << ","
             << points.getY(i) << ","
             << points.getZ(i) << "</coordinates>\n";
        file << "      </Point>\n";
        file << "    </Placemark>\n";
    }

    file << "  </Document>\n";
    file << "</kml>\n";
    file.close();
}

// ─── writeKMLClusters ────────────────────────────────────────────────────────

void PointsListExporter::writeKMLClusters(const std::vector<PointsList>& clusters,
                                          const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }
    file << std::fixed << std::setprecision(6);

    const int nClusters = static_cast<int>(clusters.size());

    file << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    file << "<kml xmlns=\"http://www.opengis.net/kml/2.2\">\n";
    file << "<Document>\n";
    file << "  <name>Percorsi Droni — Multi-Cluster</name>\n";
    file << "  <open>1</open>\n\n";

    // Stili per ogni cluster
    for (int c = 0; c < nClusters; ++c) {
        const char* col = CLUSTER_COLORS[c % N_COLORS];

        file << "  <Style id=\"line" << c << "\">\n";
        file << "    <LineStyle><color>" << col << "</color><width>3</width></LineStyle>\n";
        file << "  </Style>\n";

        file << "  <Style id=\"pin" << c << "\">\n";
        file << "    <IconStyle>\n";
        file << "      <color>" << col << "</color>\n";
        file << "      <scale>0.9</scale>\n";
        file << "      <Icon><href>http://maps.google.com/mapfiles/kml/shapes/placemark_circle.png</href></Icon>\n";
        file << "    </IconStyle>\n";
        file << "    <LabelStyle><color>" << col << "</color><scale>0.75</scale></LabelStyle>\n";
        file << "  </Style>\n\n";
    }

    // Una folder per cluster
    for (int c = 0; c < nClusters; ++c) {
        const PointsList& cl = clusters[c];
        if (cl.size() == 0) continue;

        const char* col  = CLUSTER_COLORS[c % N_COLORS];
        const char* name = CLUSTER_NAMES[c % N_COLORS];

        file << "  <Folder>\n";
        file << "    <name>" << name << " (" << cl.size() << " waypoint)</name>\n";
        file << "    <open>0</open>\n\n";

        // Linea percorso
        file << "    <Placemark>\n";
        file << "      <name>" << name << " — traiettoria</name>\n";
        file << "      <styleUrl>#line" << c << "</styleUrl>\n";
        file << "      <LineString>\n";
        file << "        <tessellate>1</tessellate>\n";
        file << "        <altitudeMode>absolute</altitudeMode>\n";
        file << "        <coordinates>\n";
        for (int i = 0; i < cl.size(); ++i)
            file << "          " << cl.getX(i) << ","
                                 << cl.getY(i) << ","
                                 << cl.getZ(i) << "\n";
        file << "        </coordinates>\n";
        file << "      </LineString>\n";
        file << "    </Placemark>\n\n";

        // Pin numerati
        for (int i = 0; i < cl.size(); ++i) {
            file << "    <Placemark>\n";
            file << "      <name>C" << c << "-W" << (i + 1) << "</name>\n";
            file << "      <styleUrl>#pin" << c << "</styleUrl>\n";
            file << "      <description>"
                 << "Cluster " << c << " — Waypoint " << (i + 1) << " di " << cl.size()
                 << "&#10;Lon: " << cl.getX(i)
                 << "  Lat: "   << cl.getY(i)
                 << "  Alt: "   << cl.getZ(i) << " m"
                 << "</description>\n";
            file << "      <Point>\n";
            file << "        <altitudeMode>absolute</altitudeMode>\n";
            file << "        <coordinates>"
                 << cl.getX(i) << ","
                 << cl.getY(i) << ","
                 << cl.getZ(i) << "</coordinates>\n";
            file << "      </Point>\n";
            file << "    </Placemark>\n";
        }

        file << "  </Folder>\n\n";
    }

    file << "</Document>\n";
    file << "</kml>\n";
    file.close();

    std::cout << "KML multi-cluster scritto: " << filename
              << " (" << nClusters << " cluster)" << std::endl;
}