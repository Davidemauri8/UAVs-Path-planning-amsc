#pragma once
#ifndef NEIGHBOURHOOD_GEN
#define NEIGHBOURHOOD_GEN

#include <random>
#include <cmath>
#include <memory>
#include "../geometry/point.hpp"
#include "../geometry/measure.hpp"
#include "../geometry/lattice_point.hpp"

// 1. INTERFACCIA EVOLUTA: Da "Filtro" a "Generatore"
template <typename PtType>
class NeighbourhoodExtractor {
public:
    using PointType = PtType;

    // Riceve il punto precedente (x_k-1) e l'attuale (x_k)
    virtual void from(PtType prev, PtType curr) = 0; 
    
    // Genera attivamente il prossimo candidato x_k+1 (PUNTO 4 dello pseudocodice)
    virtual PtType generateNext() = 0;

    // Verifica se il punto generato rispetta i confini del dominio
    virtual bool contains(PtType p) const = 0; 
};

// 2. IMPLEMENTAZIONE DRSTASA: Le 4 Trasformazioni
template<domain_dim Dim>
class STASANeighbourhood : public NeighbourhoodExtractor<point_nd<Dim>> {
private:
    std::shared_ptr<RnDomain<Dim>> domain;
    point_nd<Dim> x_k;      // Centro attuale
    point_nd<Dim> x_prev;   // Punto precedente (per la traslazione)
    
    // Parametri Epsilon dalle tue immagini
    double eps_rot, eps_trans, eps_scale, eps_axis;

    // Generatore di numeri casuali moderno
    mutable std::mt19937 gen{std::random_device{}()};

public:
    STASANeighbourhood(std::shared_ptr<RnDomain<Dim>> _dom, double _er, double _et, double _es, double _ea)
        : domain(_dom), eps_rot(_er), eps_trans(_et), eps_scale(_es), eps_axis(_ea), 
          x_k(0.0), x_prev(0.0) {}

    void from(point_nd<Dim> prev, point_nd<Dim> curr) override {
        x_prev = prev;
        x_k = curr;
    }

    // PUNTO 4: Scelta e applicazione di un operatore
    point_nd<Dim> generateNext() override {
        std::uniform_int_distribution<> dist(0, 3);
        int op = dist(gen);

        switch(op) {
            case 0: return applyRotation();
            case 1: return applyTranslation();
            case 2: return applyScaling();
            case 3: return applyAxisTransformation();
            default: return x_k;
        }
    }

    // --- IMPLEMENTAZIONE OPERATORI (Basata sulle tue immagini) ---

    // Eq (11): Translation Operator (Ricerca Locale)
    point_nd<Dim> applyTranslation() {
        std::uniform_real_distribution<double> r_dist(0.0, 1.0);
        point_nd<Dim> diff = x_k - x_prev;
        double norm = diff.norm();
        if (norm < 1e-9) return applyScaling(); // Fallback se fermo

        return x_k + diff * (eps_trans * r_dist(gen) / norm);
    }

    // Eq (10): Rotation Operator (Ricerca attorno al punto)
    point_nd<Dim> applyRotation() {
        std::uniform_real_distribution<double> r_dist(-1.0, 1.0);
        point_nd<Dim> random_v;
        for(int i=0; i<Dim; ++i) random_v.raw_data[i] = r_dist(gen);
        
        double norm_xk = x_k.norm();
        if (norm_xk < 1e-9) return applyScaling();

        return x_k + random_v * (eps_rot / (Dim * norm_xk));
    }

    // Eq (12): Scaling Operator (Ricerca Globale)
    point_nd<Dim> applyScaling() {
        std::normal_distribution<double> g_dist(0.0, 1.0);
        point_nd<Dim> gaussian_v;
        for(int i=0; i<Dim; ++i) gaussian_v.raw_data[i] = g_dist(gen);

        return x_k + gaussian_v * eps_scale;
    }

    // Eq (13): Axis Transformation (Ricerca su singolo asse)
    point_nd<Dim> applyAxisTransformation() {
        std::uniform_int_distribution<> axis_dist(0, Dim - 1);
        std::uniform_real_distribution<double> r_dist(0.0, 1.0);
        
        point_nd<Dim> next = x_k;
        int target_axis = axis_dist(gen);
        next.raw_data[target_axis] += eps_axis * r_dist(gen);
        
        return next;
    }

    bool contains(point_nd<Dim> p) const override {
        return domain->contains(&p);
    }
};

#endif