/*#include <iostream> 
#include "structures/point.hpp"
#include "structures/pointsList.hpp"
//#include "structures/penalties/obstacles.hpp"
//#include "structures/scene/scene.hpp"
//#include "structures/obstacles/cylinderObstacle.hpp"
#include "structures/lhs/lhs.hpp"
#include "structures/kmeans.hpp"


int main(){
    
    //Point A;
    //Point B(2.0, 2.0, 0.0);
    //Point C(0.0, 0.0, 2.0);
    //Point D(1.5, 1.5, 12.0);

    //std::vector<Point> v;
    //v = {A,B,C};
    //PointList P(v);

    //std::cout << "distanza A-B: " << A.distance(B) << std::endl; 
    //std::cout << "distanza B-c: " << B.distance(C) << std::endl;
    //std::cout << "distanza C-A: " << C.distance(A) << std::endl;

    //std::cout << "getX di A " << P.getX(0) << std::endl;
    //std::cout << "distance TOT " << P.totalDistance() << std::endl;


    //Scene scene;
    //Definizione di un ostacolo dichiarato in modo statitico
    //CylinderObstacle obstacleA(Point (1., 1., 1.), 0.5, 2., 0.5);
    //scene.addObstacle(&obstacleA);
    //Definizione di un ostacolo dichiarato in modo dinamico
    //CylinderObstacle* obstacleB = new CylinderObstacle(Point (1.5, 1.5, 1.5), 0.7, 1.5, 0.7);
    //scene.addObstacle(obstacleB); //usando new, bisogna poi usare delete

    //delete obstacleB;//richiama il distruttore

    //P.exctractPoint(2);


    //Creiamo due gruppi di punti molto distanti tra loro
    PointsList dataset;

    // Gruppo A: Vicino all'origine (0, 0, 0)
    dataset.addPoint(Point(0.1, 0.2, 0.1, -1));
    dataset.addPoint(Point(1.2, 1.1, 0.9, -1));
    //dataset.addPoint(Point(0.3, 0.2, 0.2, -1));

    // Gruppo B: Molto lontano (100, 100, 100)
    dataset.addPoint(Point(100.5, 100.2, 80.1, -1));
    dataset.addPoint(Point(102.1, 104.8, 80.3, -1));
    //dataset.addPoint(Point(100.9, 100.4, 80.2, -1));

    //
    dataset.addPoint(Point(50.5, 50.2, 50.1, -1));
    dataset.addPoint(Point(51.1, 50.8, 49.3, -1));
    //dataset.addPoint(Point(48.9, 50.4, 51.2, -1));

    std::cout << "Dataset caricato: " << dataset.size() << " punti." << std::endl;

    // 2. CONFIGURAZIONE K-MEANS
    // Vogliamo trovare 2 cluster (K=2) in massimo 50 iterazioni
    int k=3;
    KMeans algoritmo(k, 50);

    // 3. ESECUZIONE
    std::cout << "Inizio calcolo..." << std::endl;
    algoritmo.run(dataset);
    std::cout << "Calcolo terminato." << std::endl;

    // 4. VERIFICA DEI RISULTATI
    algoritmo.printResults();

    // 5. TEST DI LOGICA
    // Se funziona, il punto 0 e il punto 3 DEVONO avere cluster diversi
    int clusterPuntoA = dataset.extractPoint(0).getCluster();
    int clusterPuntoB = dataset.extractPoint(3).getCluster();

    std::cout << "\n--- TEST DI VERIFICA ---" << std::endl;
    if (clusterPuntoA != clusterPuntoB && clusterPuntoA != -1 && clusterPuntoB != -1) {
        std::cout << "[SUCCESS] L'algoritmo ha separato correttamente i due gruppi!" << std::endl;
    } else {
        std::cout << "[FAILURE] Qualcosa non va: i punti sono nello stesso cluster o non assegnati." << std::endl;
    }

    
// 1. Parametri: xMin, xMax, yMin, yMax, zMin, zMax, n_campioni

    for(int i=0; i<k; i++){

        PointsList df = dataset.extractCluster(i);
        //std::cout << "cluster " << df << std::endl;

        int n = 3;
        Lhs campionatore(0.0, 100.0, 0.0, 100.0, 0.0, 100.0, n, i);  //TODO dimensioni del LHS

        // 4. Ottieni la popolazione finale
        // Nota: Ho corretto PointList in PointsList come nel tuo codice
        PointsList popolazione = campionatore.generatePopulation();
 
        // 5. Stampa i risultati per verificare
        std::cout << "Punti generati con LHS (" << n << "):" << std::endl;
        for (int j = 0; j < n; j++) {
            Point p = popolazione.extractPoint(j); // Assumendo che PointsList abbia getPoint(i)
            std::cout << "Punto " << j << ": X=" << p.getX() 
                  << " Y=" << p.getY() << " Z=" << p.getZ() << std::endl;
        }


    }

}*/

#include "structures/fitness/fitnessFunction.hpp"
#include "structures/obstacles/cylinderObstacle.hpp"
#include "structures/pointsList.hpp"
#include "structures/kmeans.hpp"
#include "structures/lhs/lhs.hpp"
#include <iostream>


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
#include "algorithms/parallel_scattered_simulated_annealing.hpp"
#include <iostream>
#include <tuple>



int main() {

    // ── Ostacoli ──────────────────────────────────────────────
    std::vector<Obstacle*> obstacles;
    obstacles.push_back(new CylinderObstacle(Point(300, 400, 0, -1), 80.0, 200.0, 50.0));
    obstacles.push_back(new CylinderObstacle(Point(600, 200, 0, -1), 80.0, 200.0, 50.0));

    // ── Pesi della fitness (valori dal paper DRSTASA) ─────────
    FitnessWeights weights;
    weights.b1   = 5.0;    // distanza: peso alto, vogliamo percorsi corti
    weights.b2   = 1.0;    // ostacoli
    weights.b3   = 1.0;    // altitudine
    weights.b4   = 5.0;    // smoothness: peso alto, vogliamo traiettorie lisce
    weights.a1   = 1.0;    // angolo orizzontale
    weights.a2   = 1.0;    // angolo verticale
    weights.hMin = 100.0;  // quota minima in metri
    weights.hMax = 200.0;  // quota massima in metri

    // ── Fitness function ──────────────────────────────────────
    FitnessFunction fitness(obstacles, weights);

    // ── Test rapido su un percorso di esempio ─────────────────
    PointsList testPath;

    // Gruppo A: Vicino all'origine (0, 0, 0)
    testPath.addPoint(Point(10, 10, 120, -1)); 
    testPath.addPoint(Point(40, 50, 130, -1));

    // Gruppo B: Molto lontano (100, 100, 100)
    testPath.addPoint(Point(300, 310, 150, -1)); 
    testPath.addPoint(Point(350, 280, 140, -1));

    //
    testPath.addPoint(Point(700, 700, 160, -1));
    testPath.addPoint(Point(720, 680, 170, -1));

    //int k=3;
    //KMeans algoritmo(k, 50);
    //algoritmo.run(testPath);
    //algoritmo.printResults();

    /*for(int i = 0; i < k; i++) {
        PointsList df = testPath.extractCluster(i);
        if (df.size() == 0) continue;

        // --- Calcolo dei limiti del cluster per il Sampler ---
        double minX = 1e9, maxX = -1e9, minY = 1e9, maxY = -1e9, minZ = 1e9, maxZ = -1e9;
    
        for(int j = 0; j < df.size(); j++) {
            Point p = df.extractPoint(j);
            minX = std::min(minX, p.getX()); maxX = std::max(maxX, p.getX());
            minY = std::min(minY, p.getY()); maxY = std::max(maxY, p.getY());
            minZ = std::min(minZ, p.getZ()); maxZ = std::max(maxZ, p.getZ());
        }

        // Aggiungiamo un piccolo "margine" (buffer) di 50 unità attorno ai punti
        // per permettere all'LHS di esplorare l'area circostante
        double margin = 50.0;
    
        // Inizializzazione dinamica del Sampler LHS basata sui dati reali
        int n = 3; // Numero di campioni da generare
        Lhs campionatore(minX - margin, maxX + margin, 
                         minY - margin, maxY + margin, 
                         minZ - margin, maxZ + margin, 
                         n, i);

        PointsList popolazione = campionatore.generatePopulation();

        std::cout << "\n--- Cluster " << i << " (Target Area) ---" << std::endl;
        std::cout << "Punti generati con LHS:" << std::endl;
        for (int j = 0; j < n; j++) {
            Point p = popolazione.extractPoint(j);
            std::cout << "Campione " << j << ": X=" << p.getX() 
                      << " Y=" << p.getY() << " Z=" << p.getZ() << std::endl;
        
            // Validazione: valutiamo la fitness di ogni punto generato
            double score = fitness.evaluate(popolazione); 
            std::cout << "  -> Fitness del campione: " << score << std::endl;
        }


        // 1. Prendi un cluster (i target reali)
        PointsList targetCluster = testPath.extractCluster(i);

        // 2. Genera dei candidati (i droni "virtuali")
        PointsList candidatiLHS = campionatore.generatePopulation();

        // 3. Valuta i candidati
        for(int j = 0; j < n; j++) {
            // Estrai un singolo candidato per analizzarlo
            Point p_candidato = candidatiLHS.extractPoint(j);
    
            // Crei un "percorso temporaneo" che va dal primo punto del cluster al candidato
            PointsList miniPath;
            miniPath.addPoint(targetCluster.extractPoint(0)); 
            miniPath.addPoint(p_candidato);

            // VALUTAZIONE: Questo è il momento in cui i due mondi si toccano
            double score = fitness.evaluate(miniPath);
            std::cout << "Il candidato LHS " << j << " ha uno score di: " << score << std::endl;
        }
    }*/

    

//    for(int i=0; i<k; i++){0
//
//        double score = fitness.evaluate(testPath.extractCluster(i));
//        std::cout << "Fitness: " << score << std::endl;
//    }

//    double score = fitness.evaluate(testPath);
//    std::cout << "Fitness of the total paths: " << score << std::endl;

    
    // ── Cleanup ───────────────────────────────────────────────
    for (Obstacle* obs : obstacles) delete obs;

    return 0;

    constexpr const long max_iterations = 100;

	const lattice_nd<1> low = 0;

	const lattice_nd<1> l = -23;

	const std::shared_ptr<TransitionCriterion> metropolis = std::make_shared<MetropolisCriterion>();

	const auto disc_dom = NnDomainFactory::make_lattice_grid({ 200 }, l);
	
	auto cd = CircleDiscreteNeighbourhood<1>(disc_dom, 200);

	//auto sampler = std::make_shared<LocalSampler<CircleDiscreteNeighbourhood<1>>>(0.8);
	//auto sampler = std::make_shared<NonLocalSampler<CircleDiscreteNeighbourhood<1>>>(0.3);
	auto sampler = std::make_shared<UniformSampler<CircleDiscreteNeighbourhood<1>>>();

	// Two different schedulers are needed, they will be changed by each parallel
	// thread independently!
	auto aggro_sched = std::make_shared<LinearScheduler>(30.0, max_iterations, 20);

	sampler->seed(0xcafebabe);
	metropolis->seed(0xcafebabe);

	AnnealingExecutionPolicy lower_policy(metropolis, sampler, aggro_sched, threshold, cd);

    double score = fitness.evaluate(testPath);

	const auto quintic = TestFunctions::quintic_discrete;

	//SerialSimulatedAnnealing ssa(max_iterations);



	const auto point = ssa.run(quintic, low, lower_policy);
	std::cout << "Final result of the serial algorithm: " << point << std::endl;


}


