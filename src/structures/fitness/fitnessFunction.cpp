#include "fitnessFunction.hpp"
#include <cmath>

static const double INF = std::numeric_limits<double>::infinity();

FitnessFunction::FitnessFunction(const std::vector<std::shared_ptr<Obstacle>>& obstacles,
                                 const FitnessWeights& weights)
    : obstacles_(obstacles), w_(weights) {}

// F1: lunghezza totale del percorso
double FitnessFunction::f1_pathLength(const PointsList& path) const {
    // PointsList ha già totalDistance()
    return const_cast<PointsList&>(path).totalDistance();
}

//F2: costo ostacoli 
double FitnessFunction::f2_threatCost(const PointsList& path) const {
    double total = 0.0;

    for (int i = 0; i + 1 < path.size(); ++i) {
        Point A = const_cast<PointsList&>(path).extractPoint(i);
        Point B = const_cast<PointsList&>(path).extractPoint(i + 1);

        for (const auto& obs : obstacles_) {
            double cost = obs->segmentCost(A, B);
            if (cost == INF) return INF;
            total += cost;
        }
    }
    return total;
}

//F3: costo altitudine 
double FitnessFunction::f3_altitudeCost(const PointsList& path) const {
    double total = 0.0;
    double hMid  = (w_.hMin + w_.hMax) / 2.0;

    for (int i = 0; i < const_cast<PointsList&>(path).size(); ++i) {
        double h = const_cast<PointsList&>(path).getZ(i);

        if (h < w_.hMin || h > w_.hMax)
            return INF; // fuori range → percorso illegale

        total += std::abs(h - hMid);
    }
    return total;
}

// ─── F4: smoothness (angoli di virata e salita) ───────────────────────────────
double FitnessFunction::f4_smoothness(const PointsList& path) const {
    if (const_cast<PointsList&>(path).size() < 3) return 0.0;

    double sumPhi  = 0.0;
    double sumPsi  = 0.0;
    double prevPsi = 0.0;

    for (int i = 0; i + 2 < const_cast<PointsList&>(path).size(); ++i) {
        Point p0 = const_cast<PointsList&>(path).extractPoint(i);
        Point p1 = const_cast<PointsList&>(path).extractPoint(i + 1);
        Point p2 = const_cast<PointsList&>(path).extractPoint(i + 2);

        Point v1 = p1 - p0;  // vettore primo segmento
        Point v2 = p2 - p1;  // vettore secondo segmento

        double n1 = v1.norm();
        double n2 = v2.norm();
        if (n1 < 1e-9 || n2 < 1e-9) continue; // punti sovrapposti, salta

        // φ: angolo di virata orizzontale tra v1 e v2
        double dot   = v1 * v2;                  // operator* = dot product
        double cross = v1.cross2D(v2);
        double phi   = std::atan2(std::abs(cross), dot);
        sumPhi += phi;

        // ψ: angolo di salita del secondo segmento
        double dz  = p2.getZ() - p1.getZ();
        double dxy = std::sqrt(v2.getX() * v2.getX() + v2.getY() * v2.getY());
        double psi = (dxy > 1e-9) ? std::atan2(dz, dxy) : 0.0;

        // sommiamo la variazione di ψ, non il valore assoluto
        if (i > 0) sumPsi += std::abs(psi - prevPsi);
        prevPsi = psi;
    }

    return w_.a1 * sumPhi + w_.a2 * sumPsi;
}

// ─── evaluate: assembla tutto ─────────────────────────────────────────────────
double FitnessFunction::evaluate(const PointsList& path) const {
    if (const_cast<PointsList&>(path).size() < 2) return INF;

    // F2 e F3 prima: se illegale, esci subito senza calcolare il resto
    double f2 = f2_threatCost(path);
    if (f2 == INF) return INF;

    double f3 = f3_altitudeCost(path);
    if (f3 == INF) return INF;

    double f1 = f1_pathLength(path);
    double f4 = f4_smoothness(path);

    return w_.b1 * f1 + w_.b2 * f2 + w_.b3 * f3 + w_.b4 * f4;
}