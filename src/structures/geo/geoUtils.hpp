#ifndef GEO_UTILS_HPP
#define GEO_UTILS_HPP

#include "structures/pointsList.hpp"

struct GeoOrigin { double lat0; double lon0; };

namespace GeoUtils {

void toMeters(PointsList& points, double lat0, double lon0);
void toGPS   (PointsList& points, double lat0, double lon0);

inline void toMeters(PointsList& points, const GeoOrigin& origin) {
    toMeters(points, origin.lat0, origin.lon0);
}
inline void toGPS(PointsList& points, const GeoOrigin& origin) {
    toGPS(points, origin.lat0, origin.lon0);
}

} // namespace GeoUtils

#endif
