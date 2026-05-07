#ifndef KMEANS_HPP
#define KMEANS_HPP

#include <cmath>
#include <iostream>
#include <sstream>
#include <array>
#include <random>
#include <algorithm>
#include <cassert>
#include <vector>
#include <ctime>

#include "pointsList.hpp"

// K-Means clustering for 3D points.
// Groups waypoints into K spatial clusters so that each cluster can be
// optimised independently by a separate TSP+SA segment-optimiser.
class KMeans {
private:
    int K;
    int max_iterations;
    unsigned int seed_;
    PointsList centroids;

    // Selects K random points from the dataset as initial centroids.
    void initializeCentroids(PointsList& points) {
        std::mt19937 gen(seed_);
        std::uniform_int_distribution<> dis(0, points.size() - 1);

        for (int i = 0; i < K; ++i) {
            int randomIndex = dis(gen);
			Point randomP = points.extractPoint(randomIndex);
            centroids.addPoint({randomP.getX(), randomP.getY(), randomP.getZ(), randomP.getCluster()});
        }
    }


public:
    KMeans(int k, int iter, unsigned int seed = 42) : K(k), max_iterations(iter), seed_(seed) {}

    // Runs the K-Means algorithm: assigns each point to the nearest centroid,
    // recomputes centroids as cluster means, and repeats until convergence or max_iterations.
    void run(PointsList& points) {
        if (points.size() < K) return;

        initializeCentroids(points);

        for (int iter = 0; iter < max_iterations; ++iter) {
            bool changed = false;

            // Assign each point to the nearest centroid.
            for (int i = 0; i < points.size(); ++i) {
				Point& p = points.extractPoint(i);

                double minDist = std::numeric_limits<double>::max();
                int bestCluster = -1;

                for (int j = 0; j < K; ++j) {
                    double d = p.distance(centroids.extractPoint(j));
                    if (d < minDist) {
                        minDist = d;
                        bestCluster = j;
                    }
                }

                if (p.getCluster() != bestCluster) {
                    p.setCluster(bestCluster);
                    changed = true;
                }
            }

            // Recompute each centroid as the mean of its assigned points.
            std::vector<double> sumX(K, 0.0), sumY(K, 0.0), sumZ(K, 0.0);
            std::vector<int> counts(K, 0);

            for (int i = 0; i<points.size(); ++i) {
				Point p = points.extractPoint(i);
				int c = p.getCluster();

				if(c >= 0 && c<K){
                	sumX[c] += p.getX();
                	sumY[c] += p.getY();
                	sumZ[c] += p.getZ();
                	counts[c]++;
				}
            }

            for (int i = 0; i < K; ++i) {
                if (counts[i] > 0) {
					Point newCentroid(sumX[i] / counts[i], sumY[i] / counts[i], sumZ[i] / counts[i], i);
					centroids.replacePoint(i, newCentroid);
				}
            }

            if (!changed) {
                std::cout << "Convergence reached at iteration: " << iter << std::endl;
                break;
            }
        }
    }

    // Prints the final centroid coordinates for each cluster.
    void printResults() {
        std::cout << "\n Final Centroids " << std::endl;
        for (int i = 0; i < K; ++i) {
            std::cout << "Cluster " << i << ": ("
            << centroids.extractPoint(i).getX() << ", "
            << centroids.extractPoint(i).getY() << " , "
            << centroids.extractPoint(i).getZ() << ")" << std::endl;
        }
    }
};


#endif
