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
#include <memory>

// Include del framework Simulated Annealing
#include "geometry/point.hpp"
#include "domain/domain.hpp"
#include "domain/prefabs.hpp"
#include "scheduler/scheduler.hpp"
#include "sampler/uniform_sampler.hpp"
#include "sampler/local_sampler.hpp"
#include "sampler/non_local_sampler.hpp"
#include "sampler/exception.hpp"
#include "functions/continuous.hpp"
#include "functions/discrete.hpp"
#include "algorithms/serial_simulated_annealing.hpp"

int main() {

    // ── 1. Definizione Ostacoli e Fitness ────────────────────────
    std::vector<Obstacle*> obstacles;
    // Ostacoli posizionati strategicamente tra 100 e 400 per testare il path planning
    obstacles.push_back(new CylinderObstacle(Point(250, 350, 0, -1), 40.0, 200.0, 50.0));
    obstacles.push_back(new CylinderObstacle(Point(350, 450, 0, -1), 40.0, 200.0, 50.0));

    FitnessWeights weights;
    weights.b1 = 10.0; // Peso ostacoli (alto per evitarli)
    weights.b2 = 1.0;  // Lunghezza percorso
    weights.b3 = 1.0;  // Curvatura
    weights.b4 = 5.0;  // Smoothness
    weights.a1 = 1.0; weights.a2 = 1.0; 
    weights.hMin = 100.0; weights.hMax = 200.0;

    FitnessFunction fitness(obstacles, weights);

    // ── 2. Parametri Algoritmo e DOMINIO ──────────────────────────
    constexpr const long max_iterations = 500; // Alzate un po' per miglior convergenza

    // Punto di partenza per l'ottimizzazione (inizializzato tra i due target)
    const lattice_nd<12> start_point = {
        150, 250, 125, 
        200, 300, 125, 
        300, 400, 125, 
        350, 450, 125  
    };

    // Limiti minimi (il "pavimento" della ricerca)
    const lattice_nd<12> min_bounds = {
        80, 180, 100, 
        80, 180, 100, 
        80, 180, 100, 
        80, 180, 100
    };

    // Creazione del dominio discreto: (Limiti Massimi, Limiti Minimi)
    // I limiti massimi sono leggermente oltre il punto di arrivo (400, 500)
    const auto disc_dom = NnDomainFactory::make_lattice_grid<12>(
        {450, 550, 180, 450, 550, 180, 450, 550, 180, 450, 550, 180}, 
        min_bounds
    );

    // ── 3. Configurazione Simulated Annealing ─────────────────────
    const std::shared_ptr<TransitionCriterion> metropolis = std::make_shared<MetropolisCriterion>();
    
    // Raggio di vicinato per lo spostamento dei punti (80 unità)
    auto cd = CircleDiscreteNeighbourhood<12>(disc_dom, 80);
    auto sampler = std::make_shared<UniformSampler<CircleDiscreteNeighbourhood<12>>>();
    
    // Scheduler: (Temp Iniziale, Iterazioni Totali, Passi di stabilizzazione)
    auto aggro_sched = std::make_shared<LinearScheduler>(150.0, max_iterations, 20);

    sampler->seed(42);
    metropolis->seed(42);

    using MyNeighborhood = CircleDiscreteNeighbourhood<12>;
    using MySampler = UniformSampler<MyNeighborhood>;

    AnnealingExecutionPolicy<MySampler> lower_policy(
        metropolis, sampler, aggro_sched, StoppingCriterion::threshold, cd
    );

    // ── 4. Funzione Obiettivo (ORDINE CORRETTO) ───────────────────
    auto drone_objective = [&](const lattice_nd<12>& p) -> double {
        PointsList path;
        
        // 1. START (Fisso)
        path.addPoint(Point(100, 200, 120, -1)); 

        // 2. WAYPOINTS OTTIMIZZABILI (Mobili)
        // Inseriti IN MEZZO ai due target
        for (int i = 0; i < 12; i += 3) {
            path.addPoint(Point(p[i], p[i+1], p[i+2], -1));
        }

        // 3. ARRIVO (Fisso) - Chiude il percorso
        path.addPoint(Point(400, 500, 130, -1));

        double cost = fitness.evaluate(path);
        
        if (std::isinf(cost) || std::isnan(cost)) return 1e12;
        return cost;
    };

    // ── 5. Esecuzione ─────────────────────────────────────────────
    SerialSimulatedAnnealing ssa(max_iterations);

    std::cout << "Avvio ottimizzazione traiettoria 3D..." << std::endl;
    std::cout << "Target: (100,200) -> (400,500)" << std::endl;
    
    std::function<double(const lattice_nd<12>&)> target_func = drone_objective;

    // Esecuzione passando lo start_point invece del vecchio 'low'
    const auto result_params = ssa.run(target_func, start_point, lower_policy);

    std::cout << "\n=== RISULTATO OTTIMIZZAZIONE ===" << std::endl;
    std::cout << "Coordinate Waypoints Ottimizzati:\n" << result_params << std::endl;
    std::cout << "\nIl drone passera' da (100,200,120), tocchera' i punti sopra, e finira' a (400,500,130)." << std::endl;

    // Pulizia memoria
    for (auto obs : obstacles) delete obs;

    return 0;
}