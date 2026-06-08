#include "lhs.hpp"
#include <algorithm>


Lhs::Lhs(double xmin, double xmax, double ymin, double ymax, double zmin, double zmax, int n, int idCluster){
     this->xMin=xmin;
     this->xMax=xmax;
     this->yMin=ymin;
     this->yMax=ymax;
     this->zMin=zmin;
     this->zMax=zmax;
     this->n=n;
     this->idCluster=idCluster;

}


void Lhs::generatePartitions(){
    // Interval width for each axis.
    double dx = (xMax-xMin)/n;
    double dy = (yMax-yMin)/n;
    double dz = (zMax-zMin)/n;

    // REVIEW: Reseeding from `random_device` here defeats the deterministic seed path
    // set by DRSTASA/SA callers, so runs are not reproducible even when a seed is provided.
    // Maybe it is intentional. But it is worth noting that this is a design choice 
    //that adds external entropy to the population generation process and can make debugging harder.
    gen.seed(std::random_device{}());

    // Draw one uniform random sample from each of the n intervals for x, y, z.
    for(int i=0; i<n; i++){
       double xBegin = xMin + i * dx;
       double xEnd   = xMin + (i + 1)*dx;
       std::uniform_real_distribution<double> distX(xBegin, xEnd);
       xSamples.push_back(distX(gen));

       double yBegin = yMin + i * dy;
       double yEnd   = yMin + (i + 1)*dy;
       std::uniform_real_distribution<double> distY(yBegin, yEnd);
       ySamples.push_back(distY(gen));

       double zBegin = zMin + i * dz;
       double zEnd   = zMin + (i + 1)*dz;
       std::uniform_real_distribution<double> distZ(zBegin, zEnd);
       zSamples.push_back(distZ(gen));
    }

}

void Lhs::shuffleSamples(){
    // REVIEW: This second reseed makes the initial population depend on external entropy twice
    // per call and adds overhead in a hot path; keep one caller-controlled generator instead.
    gen.seed(std::random_device{}());
    std::shuffle(xSamples.begin(), xSamples.end(), gen);
    std::shuffle(ySamples.begin(), ySamples.end(), gen);
    std::shuffle(zSamples.begin(), zSamples.end(), gen);

}


PointsList Lhs::getSample(){
    PointsList population;

    for(int i=0; i<n; i++){
        Point newPoint(xSamples[i],ySamples[i],zSamples[i], idCluster);
        population.addPoint(newPoint);
    }
    return population;

}

PointsList Lhs::generatePopulation() {
    xSamples.clear();
    ySamples.clear();
    zSamples.clear();

    generatePartitions();
    shuffleSamples();
    return getSample();
}
