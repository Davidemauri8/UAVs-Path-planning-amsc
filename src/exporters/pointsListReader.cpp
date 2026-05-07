#include "pointsListReader.hpp"
#include <fstream>
#include <sstream>
#include <iostream>

PointsList PointsListReader::readCSV(const std::string& filename) {
    std::ifstream file(filename);
    PointsList points;

    if (!file.is_open()) {
        std::cerr << "Error opening file!" << std::endl;
        return points;
    }

    std::string line;

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string token;

        double x, y, z;

        // Legge X
        if (!std::getline(ss, token, ',')) continue;
        x = std::stod(token);

        // Legge Y
        if (!std::getline(ss, token, ',')) continue;
        y = std::stod(token);

        // Legge Z
        if (!std::getline(ss, token, ',')) continue;
        z = std::stod(token);

        points.addPoint(Point(x, y, z ,-1));
    }

    file.close();
    return points;
}