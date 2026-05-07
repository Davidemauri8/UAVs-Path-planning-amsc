#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include "structures/pointsList.hpp"
#include <cmath>
#include <random>

class Controller {
private:
    double C0;          // Soglia iniziale (tipicamente tra 1 e 3)
    int total_iters;    // T (Iterazioni totali)
    
    // Generatore di numeri casuali
    std::mt19937 gen;

    // Funzione di supporto per la distanza Euclidea tra due traiettorie (PointsList)
    // In questo contesto, calcoliamo la distanza media tra i punti corrispondenti
    double calculateDistance(const PointsList& a, const PointsList& b);

public:
    Controller(double initial_C, int iterations);

    /**
     * Applica il Disruption Operator alla soluzione corrente.
     * @param current Soluzione attuale (Xi)
     * @param best Migliore soluzione globale (Xbest)
     * @param neighbor Una soluzione vicina (Xj) per calcolare la perturbazione Ri,j
     * @param current_iter Iterazione attuale (iter)
     */
    void applyDisruption(PointsList& current, const PointsList& best, const PointsList& neighbor, int current_iter);
};

#endif