#include "segmentOptimizer.hpp"
#include <algorithm>

// Computes the XY bounding box of ordered waypoints 
SegmentBounds computeBounds(const PointsList& ordered,
                            double marginFactor, double marginMin)
{
    SegmentBounds b;
    b.xMin = ordered.getXMin();
    b.xMax = ordered.getXMax();
    b.yMin = ordered.getYMin();
    b.yMax = ordered.getYMax();
    double mx = std::max(marginMin, (b.xMax - b.xMin) * marginFactor);
    double my = std::max(marginMin, (b.yMax - b.yMin) * marginFactor);
    b.xMin -= mx; b.xMax += mx;
    b.yMin -= my; b.yMax += my;
    return b;
}
