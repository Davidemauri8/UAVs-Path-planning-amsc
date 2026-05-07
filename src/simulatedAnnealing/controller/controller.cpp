#include "controller.hpp"
#include <algorithm>
#include <ctime>

Controller::Controller(double initial_C, int iterations) 
    : C0(initial_C), total_iters(iterations) {
    gen.seed(static_cast<unsigned int>(std::time(nullptr)));
}

double Controller::calculateDistance(const PointsList& a, const PointsList& b) {
    double sumDist = 0;
    int n = std::min(a.size(), b.size());
    if (n == 0) return 0.0;

    for (int i = 0; i < n; ++i) {
        double dx = a.getX(i) - b.getX(i);
        double dy = a.getY(i) - b.getY(i);
        double dz = a.getZ(i) - b.getZ(i);
        sumDist += std::sqrt(dx*dx + dy*dy + dz*dz);
    }
    return sumDist / n; // Distanza media tra i punti del percorso
}

void Controller::applyDisruption(PointsList& current, const PointsList& best, const PointsList& neighbor, int current_iter) {
    // 1. Calcolo dinamico della soglia C (Formula 16)
    double C = C0 * (1.0 - static_cast<double>(current_iter) / total_iters);

    // 2. Calcolo distanze Ri,j (vicinato) e Ri,best (ottimo) (Formula 15)
    double Ri_j = calculateDistance(current, neighbor);
    double Ri_best = calculateDistance(current, best);

    // Controllo condizione di attivazione (Formula 15)
    if (Ri_best == 0 || (Ri_j / Ri_best) > C) {
        return; // Non soddisfa i criteri di distruzione gravitazionale
    }

    // 3. Calcolo del Disruption Factor D (Formula 17)
    std::uniform_real_distribution<double> distD(-Ri_j / 2.0, Ri_j / 2.0);
    double randVal = distD(gen);
    double D;

    if (Ri_best >= 1.0) {
        D = randVal; // Esplorazione ampia
    } else {
        D = Ri_j + randVal; // Ricerca locale (perturbazione intorno alla distanza del vicino)
    }

    // 4. Aggiornamento della soluzione (Formula 18)
    // Applichiamo la perturbazione a ogni punto della PointsList
    double iter_ratio = static_cast<double>(current_iter) / total_iters;

    for (int i = 0; i < current.size(); ++i) {
        // x_new = (iter/T)*x_old + (1 - iter/T)*x_old*D
        double oldX = current.getX(i);
        double oldY = current.getY(i);
        double oldZ = current.getZ(i);

        double newX = (iter_ratio * oldX) + (1.0 - iter_ratio) * (oldX * D);
        double newY = (iter_ratio * oldY) + (1.0 - iter_ratio) * (oldY * D);
        double newZ = (iter_ratio * oldZ) + (1.0 - iter_ratio) * (oldZ * D);

        // Nota: Qui potresti voler aggiungere un controllo per ostacoli dopo l'update
        current.replacePoint(i, Point(newX, newY, newZ, current.extractPoint(i).getCluster()));
    }
}