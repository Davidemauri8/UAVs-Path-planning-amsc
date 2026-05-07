#include "pointsListExporter.hpp"
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>

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

// ─── writeKML ────────────────────────────────────────────────────────────────

void PointsListExporter::writeKML(const PointsList& points, const std::string& filename) {
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
        file << "          " << points.getX(i) << "," << points.getY(i) << "," << points.getZ(i) << "\n";
    file << "        </coordinates>\n";
    file << "      </LineString>\n";
    file << "    </Placemark>\n\n";

    for (int i = 0; i < points.size(); ++i) {
        file << "    <Placemark>\n";
        file << "      <styleUrl>#puntoDiscreto</styleUrl>\n";
        file << "      <Point>\n";
        file << "        <altitudeMode>relativeToGround</altitudeMode>\n";
        file << "        <coordinates>" << points.getX(i) << "," << points.getY(i) << "," << points.getZ(i) << "</coordinates>\n";
        file << "      </Point>\n";
        file << "    </Placemark>\n";
    }

    file << "  </Document>\n";
    file << "</kml>\n";
    file.close();
}

// ─── Utility: formatta timestamp ISO 8601 da secondi offset ──────────────────

static std::string formatTimestamp(int totalSeconds) {
    int h = (totalSeconds / 3600) % 24;
    int m = (totalSeconds % 3600) / 60;
    int s =  totalSeconds % 60;
    std::ostringstream ss;
    ss << "2024-01-01T"
       << std::setw(2) << std::setfill('0') << h << ":"
       << std::setw(2) << std::setfill('0') << m << ":"
       << std::setw(2) << std::setfill('0') << s << "Z";
    return ss.str();
}

// ─── writeKMLWithDroneTrack ───────────────────────────────────────────────────

void PointsListExporter::writeKMLWithDroneTrack(const PointsList& points,
                                                 const std::string& filename,
                                                 const std::string& droneModelPath,
                                                 int    secondsPerWaypoint,
                                                 double droneScale) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    file << std::fixed << std::setprecision(6);

    // KML con namespace gx per gx:Track
    file << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    file << "<kml xmlns=\"http://www.opengis.net/kml/2.2\"\n";
    file << "     xmlns:gx=\"http://www.google.com/kml/ext/2.2\">\n";
    file << "  <Document>\n";
    file << "    <name>Percorso Drone con Animazione</name>\n\n";

    // Stile traiettoria
    file << "    <Style id=\"lineaRossa\">\n";
    file << "      <LineStyle><color>ff0000ff</color><width>4</width></LineStyle>\n";
    file << "    </Style>\n\n";

    // ── 1. Traiettoria (LineString) ──────────────────────────────────────────
    file << "    <Placemark>\n";
    file << "      <name>Traiettoria</name>\n";
    file << "      <styleUrl>#lineaRossa</styleUrl>\n";
    file << "      <LineString>\n";
    file << "        <tessellate>1</tessellate>\n";
    file << "        <altitudeMode>relativeToGround</altitudeMode>\n";
    file << "        <coordinates>\n";
    for (int i = 0; i < points.size(); ++i)
        file << "          " << points.getX(i) << "," << points.getY(i) << "," << points.getZ(i) << "\n";
    file << "        </coordinates>\n";
    file << "      </LineString>\n";
    file << "    </Placemark>\n\n";

    // ── 2. Drone animato via gx:Track ────────────────────────────────────────
    // gx:Track associa timestamps a coordinate; Google Earth anima il modello
    // COLLADA (.dae) lungo il percorso interpolando posizione e orientamento.
    file << "    <Placemark>\n";
    file << "      <name>Drone</name>\n";
    file << "      <gx:Track>\n";
    file << "        <altitudeMode>relativeToGround</altitudeMode>\n\n";

    // Timestamps (un timestamp per waypoint)
    for (int i = 0; i < points.size(); ++i)
        file << "        <when>" << formatTimestamp(i * secondsPerWaypoint) << "</when>\n";

    file << "\n";

    // Coordinate (lon lat alt) — nota: gx:coord usa spazi, non virgole
    for (int i = 0; i < points.size(); ++i)
        file << "        <gx:coord>"
             << points.getX(i) << " "
             << points.getY(i) << " "
             << points.getZ(i) << "</gx:coord>\n";

    file << "\n";

    // Modello COLLADA allegato al track
    file << "        <Model>\n";
    file << "          <Link><href>" << droneModelPath << "</href></Link>\n";
    file << "          <scale>\n";
    file << "            <x>" << droneScale << "</x>\n";
    file << "            <y>" << droneScale << "</y>\n";
    file << "            <z>" << droneScale << "</z>\n";
    file << "          </scale>\n";
    file << "        </Model>\n";

    file << "      </gx:Track>\n";
    file << "    </Placemark>\n\n";

    file << "  </Document>\n";
    file << "</kml>\n";
    file.close();
}

// ─── writeDroneDAE ───────────────────────────────────────────────────────────
// Genera un drone COLLADA minimale: corpo centrale (box) + 4 bracci (box).
// Dimensioni in metri: apertura alare ~2m, corpo 0.5×0.5×0.2m.
// Scalare nel KML con <scale> per adattare alle dimensioni desiderate.

void PointsListExporter::writeDroneDAE(const std::string& filename) {
    std::ofstream f(filename);
    if (!f.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    // 40 vertici (5 box × 8), 60 triangoli (5 × 12)
    // Layout vertici: [corpo][braccio X+][braccio X-][braccio Y+][braccio Y-]
    // Ogni box: v0..v7 con z- = bottom, z+ = top.
    const char* positions =
        // corpo: ±0.25 XY, ±0.10 Z
        "-0.25 -0.25 -0.1  0.25 -0.25 -0.1  0.25  0.25 -0.1 -0.25  0.25 -0.1 "
        "-0.25 -0.25  0.1  0.25 -0.25  0.1  0.25  0.25  0.1 -0.25  0.25  0.1 "
        // braccio X+: da 0.25 a 1.0, spessore 0.1
        " 0.25 -0.05 -0.05  1.0 -0.05 -0.05  1.0  0.05 -0.05  0.25  0.05 -0.05 "
        " 0.25 -0.05  0.05  1.0 -0.05  0.05  1.0  0.05  0.05  0.25  0.05  0.05 "
        // braccio X-: da -1.0 a -0.25
        "-1.0 -0.05 -0.05 -0.25 -0.05 -0.05 -0.25  0.05 -0.05 -1.0   0.05 -0.05 "
        "-1.0 -0.05  0.05 -0.25 -0.05  0.05 -0.25  0.05  0.05 -1.0   0.05  0.05 "
        // braccio Y+: da 0.25 a 1.0
        "-0.05  0.25 -0.05  0.05  0.25 -0.05  0.05  1.0 -0.05 -0.05  1.0 -0.05 "
        "-0.05  0.25  0.05  0.05  0.25  0.05  0.05  1.0  0.05 -0.05  1.0  0.05 "
        // braccio Y-: da -1.0 a -0.25
        "-0.05 -1.0 -0.05  0.05 -1.0 -0.05  0.05 -0.25 -0.05 -0.05 -0.25 -0.05 "
        "-0.05 -1.0  0.05  0.05 -1.0  0.05  0.05 -0.25  0.05 -0.05 -0.25  0.05";

    // Pattern triangoli per ogni box (offset O, 12 triangoli per box):
    // bottom: O+0,O+1,O+2  O+0,O+2,O+3
    // top:    O+4,O+7,O+6  O+4,O+6,O+5
    // front:  O+0,O+1,O+5  O+0,O+5,O+4
    // back:   O+3,O+2,O+6  O+3,O+6,O+7
    // left:   O+0,O+3,O+7  O+0,O+7,O+4
    // right:  O+1,O+2,O+6  O+1,O+6,O+5
    const char* indices =
        // corpo (O=0)
        "0 1 2  0 2 3  4 7 6  4 6 5  0 1 5  0 5 4  3 2 6  3 6 7  0 3 7  0 7 4  1 2 6  1 6 5 "
        // braccio X+ (O=8)
        "8 9 10  8 10 11  12 15 14  12 14 13  8 9 13  8 13 12  11 10 14  11 14 15  8 11 15  8 15 12  9 10 14  9 14 13 "
        // braccio X- (O=16)
        "16 17 18  16 18 19  20 23 22  20 22 21  16 17 21  16 21 20  19 18 22  19 22 23  16 19 23  16 23 20  17 18 22  17 22 21 "
        // braccio Y+ (O=24)
        "24 25 26  24 26 27  28 31 30  28 30 29  24 25 29  24 29 28  27 26 30  27 30 31  24 27 31  24 31 28  25 26 30  25 30 29 "
        // braccio Y- (O=32)
        "32 33 34  32 34 35  36 39 38  36 38 37  32 33 37  32 37 36  35 34 38  35 38 39  32 35 39  32 39 36  33 34 38  33 38 37";

    f << "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n";
    f << "<COLLADA xmlns=\"http://www.collada.org/2005/11/COLLADASchema\" version=\"1.4.1\">\n";
    f << "  <asset>\n";
    f << "    <unit name=\"meter\" meter=\"1\"/>\n";
    f << "    <up_axis>Z_UP</up_axis>\n";
    f << "  </asset>\n";

    // Materiale grigio scuro per il corpo
    f << "  <library_effects>\n";
    f << "    <effect id=\"droneEffect\">\n";
    f << "      <profile_COMMON><technique sid=\"common\"><phong>\n";
    f << "        <diffuse><color>0.15 0.15 0.15 1</color></diffuse>\n";
    f << "        <specular><color>0.4 0.4 0.4 1</color></specular>\n";
    f << "      </phong></technique></profile_COMMON>\n";
    f << "    </effect>\n";
    f << "  </library_effects>\n";
    f << "  <library_materials>\n";
    f << "    <material id=\"droneMat\" name=\"Drone\">\n";
    f << "      <instance_effect url=\"#droneEffect\"/>\n";
    f << "    </material>\n";
    f << "  </library_materials>\n";

    // Geometria: 40 vertici, 60 triangoli
    f << "  <library_geometries>\n";
    f << "    <geometry id=\"droneGeom\" name=\"drone\">\n";
    f << "      <mesh>\n";
    f << "        <source id=\"pos\">\n";
    f << "          <float_array id=\"pos-arr\" count=\"120\">" << positions << "</float_array>\n";
    f << "          <technique_common>\n";
    f << "            <accessor source=\"#pos-arr\" count=\"40\" stride=\"3\">\n";
    f << "              <param name=\"X\" type=\"float\"/>\n";
    f << "              <param name=\"Y\" type=\"float\"/>\n";
    f << "              <param name=\"Z\" type=\"float\"/>\n";
    f << "            </accessor>\n";
    f << "          </technique_common>\n";
    f << "        </source>\n";
    f << "        <vertices id=\"verts\"><input semantic=\"POSITION\" source=\"#pos\"/></vertices>\n";
    f << "        <triangles count=\"60\" material=\"droneMat\">\n";
    f << "          <input semantic=\"VERTEX\" source=\"#verts\" offset=\"0\"/>\n";
    f << "          <p>" << indices << "</p>\n";
    f << "        </triangles>\n";
    f << "      </mesh>\n";
    f << "    </geometry>\n";
    f << "  </library_geometries>\n";

    f << "  <library_visual_scenes>\n";
    f << "    <visual_scene id=\"Scene\">\n";
    f << "      <node id=\"Drone\" name=\"Drone\">\n";
    f << "        <instance_geometry url=\"#droneGeom\">\n";
    f << "          <bind_material><technique_common>\n";
    f << "            <instance_material symbol=\"droneMat\" target=\"#droneMat\"/>\n";
    f << "          </technique_common></bind_material>\n";
    f << "        </instance_geometry>\n";
    f << "      </node>\n";
    f << "    </visual_scene>\n";
    f << "  </library_visual_scenes>\n";

    f << "  <scene><instance_visual_scene url=\"#Scene\"/></scene>\n";
    f << "</COLLADA>\n";
    f.close();
}
