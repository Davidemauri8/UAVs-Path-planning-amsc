#include "structures/pipeline/pipelineRunner.hpp"
#include "structures/geo/geoUtils.hpp"
#include "structures/functions/fitnessUtilities.hpp"
#include "structures/obstacles/cylinderObstacle.hpp"
#include "structures/kmeans.hpp"
#include "exporters/pointsListReader.hpp"
#include "exporters/plotter.hpp"
#include <iostream>
#include <fstream>
#include <vector>
#include <omp.h>
#include <memory>
#include <limits>

// ─────────────────────────────────────────────────────────────────────────────
// Struttura che porta i dati geometrici degli ostacoli fino al plotter
// ─────────────────────────────────────────────────────────────────────────────
struct PlotData {
    double x, y, r, hBase, hTop;
};

// ─────────────────────────────────────────────────────────────────────────────
// Crea la fitness e popola il vettore di ostacoli per la visualizzazione 3D.
// Ogni cilindro ha:
//   - centro (x, y)   in metri rispetto all'origine GeoOrigin
//   - raggio r        in metri
//   - hBase           quota Z del piano inferiore (= z di partenza del volume vietato)
//   - hTop            quota Z del piano superiore (= z del tetto del cilindro)
// ─────────────────────────────────────────────────────────────────────────────
static FitnessFunction makeLAquilaFitness(double zMin, double zMax,
                                           std::vector<PlotData>& toPlot)
{
    std::vector<std::shared_ptr<Obstacle>> obs;
    toPlot.clear();

    // Helper: aggiunge il cilindro sia alla logica di fitness che alla lista plot
    // Parametri CylinderObstacle: centro, raggio, altezza, zBase
    auto addCylinder = [&](double x, double y, double r, double height, double zBase)
    {
        obs.push_back(std::make_shared<CylinderObstacle>(
            Point(x, y, 0.0, -1), r, height, zBase));
        toPlot.push_back({x, y, r, zBase, zBase + height});
    };

    // Ostacoli posizionati tra i waypoint per forzare routing attorno ad essi
    addCylinder(  200.0,  420.0, 120.0, 215.0,  75.0);  // nord (tra pt 5-6-7)
    addCylinder( -370.0,  160.0, 105.0, 185.0,  85.0);  // ovest (tra pt 7-8-9)
    addCylinder(  460.0, -260.0, 115.0, 200.0,  80.0);  // est-sud (tra pt 3-4)
    addCylinder( -120.0, -430.0,  95.0, 168.0,  75.0);  // sud (tra pt 1-2-9)

    return FitnessFunction(obs, sampleFitnessWeights(zMin, zMax));
}

// ─────────────────────────────────────────────────────────────────────────────
int main()
{
    // ── 1. Configurazione missione ────────────────────────────────────────────
    constexpr int NWaypoints = 4;
    const double zMin = 80.0, zMax = 300.0;
    GeoOrigin origin{42.370, 13.400};   // Centro L'Aquila

    // ── 2. Caricamento e conversione punti da CSV ─────────────────────────────
    PointsList allPointsBase =
        PointsListReader::readCSV("../data/input3.csv");
    if (allPointsBase.size() == 0) {
        std::cerr << "Errore: File CSV non trovato o vuoto.\n";
        return 1;
    }
    GeoUtils::toMeters(allPointsBase, origin);

    // ── 3. Fitness + raccolta dati per il plotter ─────────────────────────────
    std::vector<PlotData> visualData;
    FitnessFunction fitness = makeLAquilaFitness(zMin, zMax, visualData);

    // ── 4. Inizializzazione Plotter ───────────────────────────────────────────
    Plotter plotter;
    plotter.send("set title 'UAV Path Planning - L Aquila Survey'");
    plotter.setVRange(0.0, zMax + 50.0);

    // Accoda i cilindri al plotter (disegnati come superfici 3D)
    for (const auto& d : visualData) {
        plotter.addCylinder(d.x, d.y, d.r, d.hBase, d.hTop);
    }

    // ── 5. Ciclo di ottimizzazione e raccolta path ────────────────────────────
    const int maxThreads = omp_get_max_threads();
    const std::vector<int> kValues = {1};

    for (int K : kValues) {
        std::cout << "Ottimizzazione per K=" << K << "..." << std::flush;

        PointsList pts = allPointsBase;
        KMeans(K, 100).run(pts);

        // Esegue l'algoritmo in modalità parallela
        BenchmarkResult res = runPipelineOptimization<NWaypoints>(
            pts, K, fitness, zMin, zMax, maxThreads);

        plotter.addPath(res.saPath,      "SA (K=" + std::to_string(K) + ")",      "orange");
        plotter.addPath(res.drstasaPath, "DRSTASA (K=" + std::to_string(K) + ")", "blue");

        std::cout << " Completato in " << res.wallTime << "s\n";
    }

    // ── 7. Rendering finale ───────────────────────────────────────────────────
    plotter.send("set title 'Confronto Missioni UAV - L Aquila (K=1)'");
    plotter.send("set key top right");
    plotter.send("set view 55, 25");
    plotter.flush();   // unico splot con terreno + cilindri + path

    // Aspetta input prima di chiudere la finestra
    std::cout << "Premi INVIO per uscire...\n";
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::cin.get();

    return 0;
}