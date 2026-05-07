#ifndef POINTSLISTREADER_HPP
#define POINTSLISTREADER_HPP

#include <string>
#include "pointsList.hpp"

class PointsListReader {
public:
    static PointsList readCSV(const std::string& filename);
};

#endif