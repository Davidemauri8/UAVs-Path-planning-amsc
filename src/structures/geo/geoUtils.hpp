#ifndef GEO_UTILS_HPP
#define GEO_UTILS_HPP

#include "structures/pointsList.hpp"

// Reference origin for GPS↔metric coordinate conversions.
struct GeoOrigin { double lat0; double lon0; };

namespace GeoUtils {

// Converts all points in-place from GPS (lon/lat degrees) to local metric coordinates
// (metres relative to the origin lat0/lon0).
void toMeters(PointsList& points, double lat0, double lon0);

// Converts all points in-place from local metric coordinates back to GPS (lon/lat degrees).
void toGPS   (PointsList& points, double lat0, double lon0);

inline void toMeters(PointsList& points, const GeoOrigin& origin) {
    toMeters(points, origin.lat0, origin.lon0);
}
inline void toGPS(PointsList& points, const GeoOrigin& origin) {
    toGPS(points, origin.lat0, origin.lon0);
}

} // namespace GeoUtils

#endif
