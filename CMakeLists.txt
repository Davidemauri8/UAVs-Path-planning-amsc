cmake_minimum_required(VERSION 3.16)
project(UAVPathPlanning CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# header
include_directories(src/structures)
include_directories(src)
include_directories(src/simulatedAnnealing)

# Search OpenMP
find_package(OpenMP REQUIRED)

# Common Sources
set(COMMON_SOURCES
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

# TESTICELAND
add_executable(testiceland src/testiceland.cpp ${COMMON_SOURCES})
if(OpenMP_CXX_FOUND)
    target_link_libraries(testiceland PUBLIC OpenMP::OpenMP_CXX)
endif()

# TESTOBSTACLES
add_executable(testobstacles src/testobstacles.cpp ${COMMON_SOURCES})
if(OpenMP_CXX_FOUND)
    target_link_libraries(testobstacles PUBLIC OpenMP::OpenMP_CXX)
endif()

# TESTBENCHMARK
add_executable(testbenchmark src/testbenchmark.cpp ${COMMON_SOURCES})
if(OpenMP_CXX_FOUND)
    target_link_libraries(testbenchmark PUBLIC OpenMP::OpenMP_CXX)
endif()

# TESTKMEANS
add_executable(testkmeans src/testkmeans.cpp ${COMMON_SOURCES})
if(OpenMP_CXX_FOUND)
    target_link_libraries(testkmeans PUBLIC OpenMP::OpenMP_CXX)
endif()