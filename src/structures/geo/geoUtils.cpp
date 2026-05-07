#include "geoUtils.hpp"
#include <cmath>

namespace GeoUtils {

namespace {
    // Metres per degree of latitude (constant)
    constexpr double kMetersPerDegLat = 111320.0;

    // Metres per degree of longitude varies with latitude 
    inline double metersPerDegLon(double lat0) {
        return 111320.0 * std::cos(lat0 * M_PI / 180.0);
    }
}

// Shifts each point so that maps to the metric origin, then scales degrees to metres using the local conversion factors.
void toMeters(PointsList& points, double lat0, double lon0) {
    const double mpdLon = metersPerDegLon(lat0);
    for (int i = 0; i < points.size(); ++i) {
        Point& p = points.extractPoint(i);
        p.setX((p.getX() - lon0) * mpdLon);
        p.setY((p.getY() - lat0) * kMetersPerDegLat);
    }
}

// Inverse of toMeters
void toGPS(PointsList& points, double lat0, double lon0) {
    const double mpdLon = metersPerDegLon(lat0);
    for (int i = 0; i < points.size(); ++i) {
        Point& p = points.extractPoint(i);
        p.setX(lon0 + p.getX() / mpdLon);
        p.setY(lat0 + p.getY() / kMetersPerDegLat);
    }
}

} 
