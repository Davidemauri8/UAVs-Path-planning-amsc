#include "structures/fitness/fitnessFunction.hpp"
#include "structures/obstacles/cylinderObstacle.hpp"
#include "structures/pointsList.hpp"
#include "structures/kmeans.hpp"
#include "structures/lhs/lhs.hpp"
#include <iostream>
#include <tuple>
#include <cmath>
#include <vector>
#include <functional>
#include <map> // Aggiunto per il conteggio
#include "exporters/pointsListExporter.hpp"
#include "exporters/pointsListReader.hpp"

int main() {

    // Lettura del file di input
    PointsList testpath = PointsListReader::readCSV("../data/input.csv");
    
    // Configurazione ed esecuzione KMeans
    int k = 3;
    KMeans algoritmo(k, 50);
    algoritmo.run(testpath);

    // Stampa i risultati generali (centroidi, ecc.)
    algoritmo.printResults();
    
    // --- CONTEGGIO PUNTI PER CLUSTER ---
    std::map<int, int> clusterCounts;

    // Iteriamo su tutti i punti della lista originale
    for (int i = 0; i < testpath.size(); ++i) {
        // Supponendo che KMeans assegni l'ID cluster a ogni punto nella PointsList
        int clusterID = testpath.extractPoint(i).getCluster();
        clusterCounts[clusterID]++;
    }

    std::cout << "\n=== Conteggio punti per Cluster ===" << std::endl;
    for (int i = 0; i < k; ++i) {
        // Usiamo la mappa per stampare il numero di punti per ogni ID da 0 a k-1
        std::cout << "Cluster " << i << ": " << clusterCounts[i] << " punti" << std::endl;
    }
    std::cout << "==================================\n" << std::endl;

    // Esempio di estrazione e stampa di un cluster specifico (es. Cluster 1)
    PointsList cluster1 = testpath.extractCluster(1);
    std::cout << "Dettaglio Cluster 1 (estratto): " << cluster1.size() << " punti totali." << std::endl;
    
    // Se vuoi vedere i punti estratti nel terminale:
    /*
    for(int j=0; j < cluster1.size(); ++j) {
        std::cout << "Punto " << j << ": " << cluster1.getX(j) << ", " << cluster1.getY(j) << std::endl;
    }
    */

    return 0;
}