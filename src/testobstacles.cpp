#include "structures/functions/fitnessUtilities.hpp"
#include "structures/obstacles/cylinderObstacle.hpp"
#include "structures/geo/geoUtils.hpp"
#include <fstream>
#include <iostream>
#include <iomanip>
#include <cmath>

static constexpr double kMPDLat = 111320.0;
static double mpdLon(double lat0) { return 111320.0 * std::cos(lat0 * M_PI / 180.0); }

//plot all the obstacles in a KML file

int main()
{
    const GeoOrigin origin = {63.985, -22.605};
    const double lat0 = origin.lat0;
    const double lon0 = origin.lon0;
    const double mpl  = mpdLon(lat0);

    FitnessFunction fitness = makeDefaultFitness(0.0, 1000.0);
    const auto& obstacles   = fitness.getObstacles();

    const char* outfile = "../output/iceland_obstacles.kml";
    std::ofstream f(outfile);
    if (!f.is_open()) {
        std::cerr << "impossible to open: " << outfile << "\n";
        return 1;
    }
    f << std::fixed << std::setprecision(7);

    f << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
      << "<kml xmlns=\"http://www.opengis.net/kml/2.2\">\n"
      << "<Document>\n"
      << "  <name>Iceland — zone no-fly</name>\n\n";

    f << "  <Style id=\"noFly\">\n"
      << "    <PolyStyle><color>550000ff</color><fill>1</fill><outline>1</outline></PolyStyle>\n"
      << "    <LineStyle><color>ff0000ff</color><width>2</width></LineStyle>\n"
      << "  </Style>\n\n";

    constexpr int N = 64;

    int idx = 0;
    for (const auto& obs : obstacles) {
        const auto* cyl = dynamic_cast<const CylinderObstacle*>(obs.get());
        if (!cyl) { ++idx; continue; }

        const double cx  = cyl->getX();
        const double cy  = cyl->getY();
        const double r   = cyl->getRadius();
        const double h   = cyl->getHeight();
        const double buf = cyl->getBuffer();

        const double cLat = lat0 + cy / kMPDLat;
        const double cLon = lon0 + cx / mpl;

        const std::string name = "Obstacle_" + std::to_string(idx);

        f << "  <Placemark>\n"
          << "    <styleUrl>#noFly</styleUrl>\n"
          << "    <Polygon>\n"
          << "      <extrude>1</extrude>\n"
          << "      <altitudeMode>absolute</altitudeMode>\n"
          << "      <outerBoundaryIs><LinearRing><coordinates>\n";

        for (int i = 0; i <= N; ++i) {
            double theta = 2.0 * M_PI * i / N;
            double pLon  = lon0 + (cx + r * std::cos(theta)) / mpl;
            double pLat  = lat0 + (cy + r * std::sin(theta)) / kMPDLat;
            f << "        " << pLon << "," << pLat << "," << h << "\n";
        }

        f << "      </coordinates></LinearRing></outerBoundaryIs>\n"
          << "    </Polygon>\n"
          << "  </Placemark>\n\n";

        ++idx;
    }

    f << "</Document>\n</kml>\n";
    std::cout << "Write: " << outfile << " (" << idx << " Obstacles)\n";
    return 0;
}
