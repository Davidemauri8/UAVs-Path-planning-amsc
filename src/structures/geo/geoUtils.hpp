#ifndef GEO_UTILS_HPP
#define GEO_UTILS_HPP

#include "structures/pointsList.hpp"

// Reference origin for GPS to coordinate 
struct GeoOrigin { double lat0; double lon0; };

namespace GeoUtils {

// Converts all points from GPS to local metric coordinates
void toMeters(PointsList& points, double lat0, double lon0);

// Converts all points in-place from local metric coordinates back to GPS
void toGPS   (PointsList& points, double lat0, double lon0);

inline void toMeters(PointsList& points, const GeoOrigin& origin) {
    toMeters(points, origin.lat0, origin.lon0);
}
inline void toGPS(PointsList& points, const GeoOrigin& origin) {
    toGPS(points, origin.lat0, origin.lon0);
}

} 

#endif
