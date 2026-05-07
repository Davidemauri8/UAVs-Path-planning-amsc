#ifndef SEGMENT_APPENDER_HPP
#define SEGMENT_APPENDER_HPP

#include "segmentOptimizer.hpp"

/// Appende segStart + i waypoints SA al path.
template<int NWaypoints>
inline void appendSASegment(PointsList& path,
                            const SegmentSAResult<NWaypoints>& res,
                            const Point& segStart)
{
    path.addPoint(segStart);
    for (int w = 0; w < NWaypoints; ++w)
        path.addPoint(Point(res.bestPoint[w*3], res.bestPoint[w*3+1], res.bestPoint[w*3+2], -1));
}

/// Appende tutti i punti di seg tranne l'ultimo (che è segEnd, aggiunto dal chiamante).
inline void appendDRSTASASegment(PointsList& path, const PointsList& seg)
{
    for (int w = 0; w < seg.size() - 1; ++w)
        path.addPoint(Point(seg.getX(w), seg.getY(w), seg.getZ(w), -1));
}

#endif
