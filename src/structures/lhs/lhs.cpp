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
    //Ampiezza di ogni intervallo per x y z
    double dx = (xMax-xMin)/n;
    double dy = (yMax-yMin)/n;
    double dz = (zMax-zMin)/n;

    //Inizializzo il generatore di numeri casuali
    gen.seed(std::random_device{}());

    //Prendo campione casuale per x y z
    for(int i=0; i<n; i++){
       //Intervalli per x
       double xBegin = xMin + i * dx;
       double xEnd = xMin + (i + 1)*dx;
       
       //Valore per ogni intervallo di x
       std::uniform_real_distribution<double> distX(xBegin, xEnd);
       xSamples.push_back(distX(gen));

       //Intervalli per y
       double yBegin = yMin + i * dy;
       double yEnd = yMin + (i + 1)*dy;
       
       //Valore per ogni intervallo di y
       std::uniform_real_distribution<double> distY(yBegin, yEnd);
       ySamples.push_back(distY(gen));

       //Intervalli per z
       double zBegin = zMin + i * dz;
       double zEnd = zMin + (i + 1)*dz;
       
       //Valore per ogni intervallo di z
       std::uniform_real_distribution<double> distZ(zBegin, zEnd);
       zSamples.push_back(distZ(gen));
    }

}

void Lhs::shuffleSamples(){
    gen.seed(std::random_device{}());
    std::shuffle(xSamples.begin(), xSamples.end(), gen);
    std::shuffle(ySamples.begin(), ySamples.end(), gen);
    std::shuffle(zSamples.begin(), zSamples.end(), gen);
    
}


PointsList Lhs::getSample( ){
    PointsList population;
    
    for(int i=0; i<n; i++){
        Point newPoint(xSamples[i],ySamples[i],zSamples[i], idCluster);

        population.addPoint(newPoint);
    }
    //pulizia vettori per prossima generazione

    //Ritorno la popolazione pronta per l'algoritmo
    return population;

}

PointsList Lhs::generatePopulation() {
    xSamples.clear();
    ySamples.clear();
    zSamples.clear();

    //Crea le partizioni negli intervalli
    generatePartitions();
    
    //Mischia i campioni per garantire la proprietà del Latin Hypercube
    shuffleSamples();
    
    //Costruisce la lista di punti e pulisce i vettori interni
    return getSample();
}

