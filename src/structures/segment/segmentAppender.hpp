#ifndef SEGMENT_APPENDER_HPP
#define SEGMENT_APPENDER_HPP

#include "segmentOptimizer.hpp"

// Appends segStart followed by the NWaypoints intermediate waypoints from a
// classic SA result to path
// the caller is responsible for adding it when processing the next segment
template<int NWaypoints>
inline void appendSASegment(PointsList& path,
                            const SegmentSAResult<NWaypoints>& res,
                            const Point& segStart)
{
    path.addPoint(segStart);
    for (int w = 0; w < NWaypoints; ++w)
        path.addPoint(Point(res.bestPoint[w*3], res.bestPoint[w*3+1], res.bestPoint[w*3+2], -1));
}

// Appends all points of a DRSTASA segment except the last one, which is added by the caller when starting the next segment
inline void appendDRSTASASegment(PointsList& path, const PointsList& seg)
{
    for (int w = 0; w < seg.size() - 1; ++w)
        path.addPoint(Point(seg.getX(w), seg.getY(w), seg.getZ(w), -1));
}

#endif
