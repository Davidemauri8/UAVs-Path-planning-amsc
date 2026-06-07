#include "pointsListExporter.hpp"
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <cmath>

static const char* CLUSTER_COLORS[] = {
    "ffff0000",   // blue
    "ff00cc00",   // green
    "ff00ffff",   // Yellow
    "ff0088ff",   // orange
    "ffff0088",   // purple
    "ff00ccff",   // gold
};
static const char* CLUSTER_NAMES[] = {
    "Cluster 0", "Cluster 1", "Cluster 2",
    "Cluster 3", "Cluster 4", "Cluster 5",
};
static constexpr int N_COLORS = 6;

// Comparison
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

// writeCSVClusters 
void PointsListExporter::writeCSVClusters(const std::vector<PointsList>& targets,
                                          const std::vector<PointsList>& saWaypoints,
                                          const std::vector<PointsList>& drstasaWaypoints,
                                          const std::string& filename) {
    std::ofstream f(filename);
    if (!f.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }
    f << std::fixed << std::setprecision(7);
    f << "cluster,type,index,lon,lat,alt\n";

    const int K = static_cast<int>(targets.size());
    for (int c = 0; c < K; ++c) {
        const PointsList& tgt = targets[c];
        for (int i = 0; i < tgt.size(); ++i)
            f << c << ",target," << i << ","
              << tgt.getX(i) << "," << tgt.getY(i) << "," << tgt.getZ(i) << "\n";

        if (c < static_cast<int>(saWaypoints.size())) {
            const PointsList& sa = saWaypoints[c];
            for (int i = 0; i < sa.size(); ++i)
                f << c << ",waypoint_sa," << i << ","
                  << sa.getX(i) << "," << sa.getY(i) << "," << sa.getZ(i) << "\n";
        }

        if (c < static_cast<int>(drstasaWaypoints.size())) {
            const PointsList& dr = drstasaWaypoints[c];
            for (int i = 0; i < dr.size(); ++i)
                f << c << ",waypoint_drstasa," << i << ","
                  << dr.getX(i) << "," << dr.getY(i) << "," << dr.getZ(i) << "\n";
        }
    }
    f.close();
}

// writeKMLClustersWithTargets 
void PointsListExporter::writeKMLClustersWithTargets(
    const std::vector<PointsList>& paths,
    const std::vector<PointsList>& targets,
    const std::string& filename)
{
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }
    file << std::fixed << std::setprecision(6);

    const int nClusters = static_cast<int>(paths.size());

    file << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    file << "<kml xmlns=\"http://www.opengis.net/kml/2.2\">\n";
    file << "<Document>\n";
    file << "  <name>Percorsi Droni — Multi-Cluster</name>\n";
    file << "  <open>1</open>\n\n";

    // One line style per cluster; one label-only style per cluster
    for (int c = 0; c < nClusters; ++c) {
        const char* col = CLUSTER_COLORS[c % N_COLORS];

        file << "  <Style id=\"line" << c << "\">\n";
        file << "    <LineStyle><color>" << col << "</color><width>3</width></LineStyle>\n";
        file << "  </Style>\n";

        file << "  <Style id=\"label" << c << "\">\n";
        file << "    <IconStyle>\n";
        file << "      <color>ffffffff</color>\n";
        file << "      <scale>0.5</scale>\n";
        file << "      <Icon><href>http://maps.google.com/mapfiles/kml/shapes/shaded_dot.png</href></Icon>\n";
        file << "    </IconStyle>\n";
        file << "    <LabelStyle><color>" << col << "</color><scale>0.85</scale></LabelStyle>\n";
        file << "  </Style>\n\n";
    }

    for (int c = 0; c < nClusters; ++c) {
        if (c >= static_cast<int>(paths.size())) continue;
        const PointsList& path = paths[c];
        if (path.size() == 0) continue;

        const char* name = CLUSTER_NAMES[c % N_COLORS];

        file << "  <Folder>\n";
        file << "    <name>" << name << "</name>\n";
        file << "    <open>0</open>\n\n";

        // Path line — passes through all points 
        file << "    <Placemark>\n";
        file << "      <name>" << name << " — percorso</name>\n";
        file << "      <styleUrl>#line" << c << "</styleUrl>\n";
        file << "      <LineString>\n";
        file << "        <tessellate>1</tessellate>\n";
        file << "        <altitudeMode>absolute</altitudeMode>\n";
        file << "        <coordinates>\n";
        for (int i = 0; i < path.size(); ++i)
            file << "          " << path.getX(i) << ","
                                 << path.getY(i) << ","
                                 << path.getZ(i) << "\n";
        file << "        </coordinates>\n";
        file << "      </LineString>\n";
        file << "    </Placemark>\n\n";

        // Text labels only for original target points.
        if (c < static_cast<int>(targets.size())) {
            const PointsList& tgt = targets[c];
            for (int i = 0; i < tgt.size(); ++i) {
                file << "    <Placemark>\n";
                file << "      <name>TP " << (i + 1) << "</name>\n";
                file << "      <styleUrl>#label" << c << "</styleUrl>\n";
                file << "      <Point>\n";
                file << "        <altitudeMode>absolute</altitudeMode>\n";
                file << "        <coordinates>"
                     << tgt.getX(i) << ","
                     << tgt.getY(i) << ","
                     << tgt.getZ(i) << "</coordinates>\n";
                file << "      </Point>\n";
                file << "    </Placemark>\n";
            }
        }

        file << "  </Folder>\n\n";
    }

    file << "</Document>\n";
    file << "</kml>\n";
    file.close();

}

// writeKML
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

// writeKMLClusters 
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

    // style for each cluster
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

    // folder for each cluster
    for (int c = 0; c < nClusters; ++c) {
        const PointsList& cl = clusters[c];
        if (cl.size() == 0) continue;

        const char* col  = CLUSTER_COLORS[c % N_COLORS];
        const char* name = CLUSTER_NAMES[c % N_COLORS];

        file << "  <Folder>\n";
        file << "    <name>" << name << " (" << cl.size() << " waypoint)</name>\n";
        file << "    <open>0</open>\n\n";

        // line path
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

        // numbered pin
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

    //std::cout << "KML multi-cluster scritto: " << filename
    //          << " (" << nClusters << " cluster)" << std::endl;
}

// writeKMLComparison 
void PointsListExporter::writeKMLComparison(
    const std::vector<PointsList>& saPaths,
    const std::vector<PointsList>& drstasaPaths,
    const std::vector<PointsList>& targets,
    const std::vector<ObstacleCircle>& obstacles,
    const std::string& filename)
{
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }
    file << std::fixed << std::setprecision(7);

    const int nClusters = static_cast<int>(
        std::min(saPaths.size(), drstasaPaths.size()));

    file << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    file << "<kml xmlns=\"http://www.opengis.net/kml/2.2\">\n";
    file << "<Document>\n";
    file << "  <name>SA vs DRSTASA — Comparison</name>\n";
    file << "  <open>1</open>\n\n";

    // Obstacle style
    file << "  <Style id=\"obstacle\">\n";
    file << "    <PolyStyle><color>550000ff</color><fill>1</fill><outline>1</outline></PolyStyle>\n";
    file << "    <LineStyle><color>ff0000ff</color><width>2</width></LineStyle>\n";
    file << "  </Style>\n\n";

    // Per-cluster styles
    for (int c = 0; c < nClusters; ++c) {
        const char* col = CLUSTER_COLORS[c % N_COLORS];

        std::string colSA = std::string("99") + std::string(col + 2);

        file << "  <Style id=\"drstasa_line" << c << "\">\n";
        file << "    <LineStyle><color>" << col << "</color><width>4</width></LineStyle>\n";
        file << "  </Style>\n";

        file << "  <Style id=\"sa_line" << c << "\">\n";
        file << "    <LineStyle><color>" << colSA << "</color><width>2</width></LineStyle>\n";
        file << "  </Style>\n";

        file << "  <Style id=\"label" << c << "\">\n";
        file << "    <IconStyle>\n";
        file << "      <color>ffffffff</color>\n";
        file << "      <scale>0.5</scale>\n";
        file << "      <Icon><href>http://maps.google.com/mapfiles/kml/shapes/shaded_dot.png</href></Icon>\n";
        file << "    </IconStyle>\n";
        file << "    <LabelStyle><color>" << col << "</color><scale>0.85</scale></LabelStyle>\n";
        file << "  </Style>\n\n";
    }

    // Obstacle folder
    if (!obstacles.empty()) {
        constexpr int N_POLY = 64;
        file << "  <Folder>\n";
        file << "    <name>Obstacles</name>\n";
        file << "    <open>0</open>\n\n";
        for (const auto& obs : obstacles) {
            file << "    <Placemark>\n";
            file << "      <styleUrl>#obstacle</styleUrl>\n";
            file << "      <Polygon><tessellate>1</tessellate>\n";
            file << "        <outerBoundaryIs><LinearRing><coordinates>\n";
            for (int i = 0; i <= N_POLY; ++i) {
                double theta = 2.0 * M_PI * i / N_POLY;
                double pLon  = obs.centerLon + obs.radiusDegLon * std::cos(theta);
                double pLat  = obs.centerLat + obs.radiusDegLat * std::sin(theta);
                file << "          " << pLon << "," << pLat << ",0\n";
            }
            file << "        </coordinates></LinearRing></outerBoundaryIs>\n";
            file << "      </Polygon>\n";
            file << "    </Placemark>\n\n";
        }
        file << "  </Folder>\n\n";
    }

    // Per-cluster folders
    for (int c = 0; c < nClusters; ++c) {
        const char* name = CLUSTER_NAMES[c % N_COLORS];

        file << "  <Folder>\n";
        file << "    <name>" << name << "</name>\n";
        file << "    <open>0</open>\n\n";

        // DRSTASA — solid line
        if (c < static_cast<int>(drstasaPaths.size()) && drstasaPaths[c].size() > 0) {
            const PointsList& dp = drstasaPaths[c];
            file << "    <Placemark>\n";
            file << "      <name>DRSTASA</name>\n";
            file << "      <styleUrl>#drstasa_line" << c << "</styleUrl>\n";
            file << "      <LineString>\n";
            file << "        <tessellate>1</tessellate>\n";
            file << "        <altitudeMode>absolute</altitudeMode>\n";
            file << "        <coordinates>\n";
            for (int i = 0; i < dp.size(); ++i)
                file << "          " << dp.getX(i) << "," << dp.getY(i) << "," << dp.getZ(i) << "\n";
            file << "        </coordinates>\n";
            file << "      </LineString>\n";
            file << "    </Placemark>\n\n";
        }

        // SA
        if (c < static_cast<int>(saPaths.size()) && saPaths[c].size() > 0) {
            const PointsList& sp = saPaths[c];
            const int n = sp.size();
            for (int i = 0; i + 1 < n; i += 2) {
                file << "    <Placemark>\n";
                file << "      <styleUrl>#sa_line" << c << "</styleUrl>\n";
                file << "      <LineString>\n";
                file << "        <tessellate>1</tessellate>\n";
                file << "        <altitudeMode>absolute</altitudeMode>\n";
                file << "        <coordinates>\n";
                file << "          " << sp.getX(i)   << "," << sp.getY(i)   << "," << sp.getZ(i)   << "\n";
                file << "          " << sp.getX(i+1) << "," << sp.getY(i+1) << "," << sp.getZ(i+1) << "\n";
                file << "        </coordinates>\n";
                file << "      </LineString>\n";
                file << "    </Placemark>\n";
            }
            file << "\n";
        }

        // Target point labels 
        if (c < static_cast<int>(targets.size())) {
            const PointsList& tgt = targets[c];
            for (int i = 0; i < tgt.size(); ++i) {
                file << "    <Placemark>\n";
                file << "      <name>TP " << (i + 1) << "</name>\n";
                file << "      <styleUrl>#label" << c << "</styleUrl>\n";
                file << "      <Point>\n";
                file << "        <altitudeMode>absolute</altitudeMode>\n";
                file << "        <coordinates>"
                     << tgt.getX(i) << "," << tgt.getY(i) << "," << tgt.getZ(i)
                     << "</coordinates>\n";
                file << "      </Point>\n";
                file << "    </Placemark>\n";
            }
        }

        file << "  </Folder>\n\n";
    }

    file << "</Document>\n";
    file << "</kml>\n";
    file.close();

    //std::cout << "KML comparison scritto: " << filename
    //          << " (" << nClusters << " cluster)" << std::endl;
}// review
