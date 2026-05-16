cmake_minimum_required(VERSION 3.16)
project(UAVPathPlanning CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Cartelle degli header
include_directories(src/structures)
include_directories(src)
include_directories(src/simulatedAnnealing)

# 1. CERCA OPENMP (Fondamentale per parallelismo)
find_package(OpenMP REQUIRED)

# --- SORGENTI COMUNI ---
# Definiamo una variabile con i file comuni per evitare ripetizioni
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

<<<<<<< Updated upstream
# --- TARGET: TEST 14 ---
add_executable(test14 src/test14.cpp ${COMMON_SOURCES})
if(OpenMP_CXX_FOUND)
    target_link_libraries(test14 PUBLIC OpenMP::OpenMP_CXX)
endif()

# --- TARGET: TEST 15 (Quello che ti dava errore) ---
add_executable(test15 src/test15.cpp ${COMMON_SOURCES})
if(OpenMP_CXX_FOUND)
    # Questo comando aggiunge sia i flag di compilazione che il link alle librerie
    target_link_libraries(test15 PUBLIC OpenMP::OpenMP_CXX)
endif()

# --- TARGET ALTRI (Esempio test11, 12 se necessari) ---
add_executable(test11 src/test11.cpp ${COMMON_SOURCES})
add_executable(test12 src/test12.cpp ${COMMON_SOURCES})
if(OpenMP_CXX_FOUND)
    target_link_libraries(test12 PUBLIC OpenMP::OpenMP_CXX)
endif()
=======
# L'Aquila survey benchmark (200 pts, 5 natural clusters, 6 obstacles)
set(SOURCES_TEST14
    src/test14.cpp
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
add_executable(test14 ${SOURCES_TEST14})
target_link_libraries(test14 OpenMP::OpenMP_CXX)

# Strong-scaling benchmark: K=10 fixed, sweep threads 1..maxThreads -> benchmark16.csv
set(SOURCES_TEST16
    src/test16.cpp
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
add_executable(test16 ${SOURCES_TEST16})
target_link_libraries(test16 OpenMP::OpenMP_CXX)
>>>>>>> Stashed changes
