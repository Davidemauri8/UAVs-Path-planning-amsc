#include "segmentOptimizer.hpp"
#include <algorithm>

SegmentBounds computeBounds(const PointsList& ordered,
                            double marginFactor, double marginMin)
{
    SegmentBounds b;
    b.xMin = const_cast<PointsList&>(ordered).getXMin();
    b.xMax = const_cast<PointsList&>(ordered).getXMax();
    b.yMin = const_cast<PointsList&>(ordered).getYMin();
    b.yMax = const_cast<PointsList&>(ordered).getYMax();
    double mx = std::max(marginMin, (b.xMax - b.xMin) * marginFactor);
    double my = std::max(marginMin, (b.yMax - b.yMin) * marginFactor);
    b.xMin -= mx; b.xMax += mx;
    b.yMin -= my; b.yMax += my;
    return b;
}
