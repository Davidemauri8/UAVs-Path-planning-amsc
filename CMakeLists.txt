cmake_minimum_required(VERSION 3.16)
project(UAVPathPlanning CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Cartella degli header
include_directories(src/structures)
include_directories(src)
include_directories(src/simulatedAnnealing)


# Eseguibile test11: versione refactorizzata pre-parallelizzazione
set(SOURCES_TEST11
    src/test11.cpp
    src/structures/point.cpp
    src/structures/pointsList.cpp
    src/structures/drone.cpp
    src/structures/obstacles/cylinderObstacle.cpp
    src/structures/fitness/fitnessFunction.cpp
    src/structures/lhs/lhs.cpp
    src/structures/drstasa/drstasa.cpp
    src/structures/drstasa/stasaNeighbourhood.cpp
    src/structures/drstasa/reverseLearnStrategy.cpp
    src/structures/functions/fitnessUtilities.cpp
    src/structures/geo/geoUtils.cpp
    src/structures/segment/segmentOptimizer.cpp
    src/simulatedAnnealing/controller/controller.cpp
    src/exporters/pointsListExporter.cpp
    src/exporters/pointsListReader.cpp
)
add_executable(test11 ${SOURCES_TEST11})

# Eseguibile test12: versione parallelizzata con OpenMP
find_package(OpenMP REQUIRED)
set(SOURCES_TEST12
    src/test12.cpp
    src/structures/point.cpp
    src/structures/pointsList.cpp
    src/structures/drone.cpp
    src/structures/obstacles/cylinderObstacle.cpp
    src/structures/fitness/fitnessFunction.cpp
    src/structures/lhs/lhs.cpp
    src/structures/drstasa/drstasa.cpp
    src/structures/drstasa/stasaNeighbourhood.cpp
    src/structures/drstasa/reverseLearnStrategy.cpp
    src/structures/functions/fitnessUtilities.cpp
    src/structures/geo/geoUtils.cpp
    src/structures/segment/segmentOptimizer.cpp
    src/simulatedAnnealing/controller/controller.cpp
    src/exporters/pointsListExporter.cpp
    src/exporters/pointsListReader.cpp
)
add_executable(test12 ${SOURCES_TEST12})

# Eseguibile test13: benchmark serial vs parallel su K variabile
set(SOURCES_TEST13
    src/test13.cpp
    src/structures/point.cpp
    src/structures/pointsList.cpp
    src/structures/drone.cpp
    src/structures/obstacles/cylinderObstacle.cpp
    src/structures/fitness/fitnessFunction.cpp
    src/structures/lhs/lhs.cpp
    src/structures/drstasa/drstasa.cpp
    src/structures/drstasa/stasaNeighbourhood.cpp
    src/structures/drstasa/reverseLearnStrategy.cpp
    src/structures/functions/fitnessUtilities.cpp
    src/structures/geo/geoUtils.cpp
    src/structures/segment/segmentOptimizer.cpp
    src/simulatedAnnealing/controller/controller.cpp
    src/exporters/pointsListExporter.cpp
    src/exporters/pointsListReader.cpp
)
add_executable(test13 ${SOURCES_TEST13})
target_link_libraries(test13 OpenMP::OpenMP_CXX)
