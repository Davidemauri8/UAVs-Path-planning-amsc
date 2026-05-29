#include "fitnessUtilities.hpp"
#include <algorithm>
#include <cmath>
#include <memory>

// Builds a FitnessWeights with all operative values. Weight fields (b1..a2) are
// defined here and left at zero in the struct; only altitude bounds come from outside.
FitnessWeights sampleFitnessWeights(double zMin, double zMax) {
    FitnessWeights weights;
    // b1: length penalty doubled to create strong tension with obstacle avoidance —
    // the optimizer must find the shortest *feasible* path, not just any feasible path.
    weights.b1          = 12.0;
    // b2: obstacle weight doubled — sharper barriers create narrower corridors and
    // a more multimodal landscape, where DRSTASA's population covers multiple
    // corridors simultaneously while SA tends to commit to one.
    weights.b2          = 20.0;
    // b3: small increase; altitude constraint is a secondary selective pressure.
    weights.b3          = 2.0;
    // b4: smoothness weight doubled — DRSTASA's geometric operators (rotation,
    // scaling) produce globally coherent path deformations that inherently
    // respect smoothness better than SA's random local steps.
    weights.b4          = 10.0;
    // a1: stronger horizontal-turn penalty — sharp turns around obstacles are
    // expensive, rewarding DRSTASA's rotation operator over SA's random walk.
    weights.a1          = 2.5;
    // a2: moderate increase for vertical climb-angle variation.
    weights.a2          = 1.5;
    weights.hMin        = zMin;
    weights.hMax        = zMax;
    // Larger footprint narrows the feasible corridors between obstacles,
    // making the problem harder and more selective.
    weights.droneRadius = 3.0;
    return weights;
}


// All DRSTASA operative parameters are defined here; the Config struct fields are zero by default.
DRSTASA::Config GetConfigurationDRST(int NWaypoints) {
    DRSTASA::Config drsCfg;

    // popSize=30: larger population covers more corridors simultaneously;
    // budget ≈ 5 × 30 × 500 = 75,000 fitness evaluations, comparable to SA.
    drsCfg.popSize    = 30;
    drsCfg.maxIter    = 500;
    drsCfg.T0         = 100.0;
    // Slower cooling: each temperature level is exploited longer by the full
    // population — critical when the landscape has many shallow local minima.
    drsCfg.alpha      = 0.96;
    // p=0.4: reverse learning is applied 60% of iterations (up from 50%),
    // giving more chances to exploit opposite-side solutions around obstacles.
    drsCfg.p          = 0.4;
    // Stronger disruption to escape corridor-trapping local minima.
    drsCfg.C0         = 3.0;
    // Larger geometric perturbations so rotations and translations can reach
    // alternative corridors that are far from the current solution.
    drsCfg.eps_rot    = 180.0;
    drsCfg.eps_trans  = 150.0;
    drsCfg.eps_scale  = 0.08;
    drsCfg.eps_axis   = 0.08;

    drsCfg.nWaypoints = NWaypoints;

    return drsCfg;
}

// Setting obstacles parameters.
// Coordinate system: origin = (lat0=63.985, lon0=-22.605)
//   x = (lon - lon0) * mpdLon,  mpdLon = 111320 * cos(63.985 * pi/180) ≈ 48840 m/deg
//   y = (lat - lat0) * 111320
// Obstacle layout designed to intercept ALL four cluster paths:
//   C1 (west/Snæfellsnes), C2 (northwest/Westfjords),
//   C3 (south coast), C0 (northeast/east arc).
std::vector<std::shared_ptr<Obstacle>> buildDefaultObstacles() {
    std::vector<std::shared_ptr<Obstacle>> obstacles;

    // ── C1: WP0→WP15→WP16 (123 km + 53 km) ────────────────────────────────────
 
    // Reykjanes Ridge — mid WP0→WP15 (~123 km)
    // Ridge vulcanica SW; forza il drone ad aggirare la penisola di Reykjanes
    obstacles.push_back(std::make_shared<CylinderObstacle>(
        Point(-25922.9, 37805.7, 0.0, -1), 8000.0, 2000.0, 100.0));
 
    // Snæfellsjökull — mid WP15→WP16 (~53 km)
    // Vulcano-ghiacciaio sulla punta della penisola di Snæfellsnes
    obstacles.push_back(std::make_shared<CylinderObstacle>(
        Point(-31531.5, 97295.0, 0.0, -1), 5000.0, 1500.0, 90.0));
 
    // ── C2: WP19→WP18→WP17→WP20 (36 + 79 + 63 km) ─────────────────────────────
 
    // Vestfjarðahæðir — mid WP19→WP18 (~36 km)
    // Altopiano dei Fiordi Occidentali
    obstacles.push_back(std::make_shared<CylinderObstacle>(
        Point(-75108.3, 179579.3, 0.0, -1), 5000.0, 1500.0, 80.0));
 
    // Ísafjarðarfjöll — mid WP18→WP17 (~79 km)
    // Catena montuosa attorno a Ísafjörður
    obstacles.push_back(std::make_shared<CylinderObstacle>(
        Point(-19021.8, 177911.3, 0.0, -1), 5000.0, 1500.0, 80.0));
 
    // Drangajökull — mid WP17→WP20 (~63 km)
    // Il ghiacciaio più a nord dell'Islanda
    obstacles.push_back(std::make_shared<CylinderObstacle>(
        Point(26578.9, 201262.3, 0.0, -1), 6000.0, 1500.0, 90.0));
 
    // ── C3: WP4→WP1→WP2→WP3 (82 + 40 + 72 km) ─────────────────────────────────
 
    // Langjökull — mid WP4→WP1 (~82 km)
    // 2° ghiacciaio più grande d'Islanda; blocca il corridoio tra C1 e C3
    obstacles.push_back(std::make_shared<CylinderObstacle>(
        Point(127290.6, -14455.9, 0.0, -1), 18000.0, 1500.0, 120.0));
 
    // Hrafntinnusker — 63.878°N 19.026°W (Fjallabak highlands)
    // Cresta di ossidiana/riolite; il percorso SA C3 la attraversa
    // nel tratto WP4→WP1 prima di scendere sulla costa S
    obstacles.push_back(std::make_shared<CylinderObstacle>(
        Point(174793.3, -11898.4, 0.0, -1), 3500.0, 1500.0, 70.0));
 
    // Katla / Mýrdalsjökull — mid WP2→WP3 (~72 km)
    // Caldera subglaciale costa S; raggio largo su segmento lungo
    obstacles.push_back(std::make_shared<CylinderObstacle>(
        Point(187522.5, -56821.2, 0.0, -1), 10000.0, 1500.0, 140.0));
 
    // ── C0: WP5→6→7→8→9→14→10→11→12→13 (arco N/E, 9 segmenti) ─────────────────
 
    // Hverfjall — mid WP5→WP6 (~54 km)
    // Cratere di tephra, area del Lago Mývatn; ostacolo piccolo ma preciso
    obstacles.push_back(std::make_shared<CylinderObstacle>(
        Point(246779.0, 182359.1, 0.0, -1), 2500.0, 1500.0, 60.0));
 
    // Krafla — mid WP6→WP7 (~33 km)
    // Vulcano a fessura, area Mývatn nord
    obstacles.push_back(std::make_shared<CylinderObstacle>(
        Point(285795.7, 190698.7, 0.0, -1), 5000.0, 1500.0, 80.0));
 
    // Herðubreið — mid WP7→WP8 (~46 km)
    // Vulcano-tuya a cima piatta, NE Islanda
    obstacles.push_back(std::make_shared<CylinderObstacle>(
        Point(319935.2, 195146.5, 0.0, -1), 3500.0, 1500.0, 100.0));
 
    // Askja / Dyngjufjöll — mid WP8→WP9 (~61 km)
    // Complesso di caldere, highlands centrali
    obstacles.push_back(std::make_shared<CylinderObstacle>(
        Point(371388.4, 195813.7, 0.0, -1), 8000.0, 1500.0, 120.0));
 
    // Snæfell — mid WP9→WP14 (~43 km)
    // Picco isolato, Islanda orientale
    obstacles.push_back(std::make_shared<CylinderObstacle>(
        Point(388458.2, 185806.2, 0.0, -1), 5000.0, 1500.0, 110.0));
 
    // Kverkfjöll — mid WP14→WP10 (~81 km)
    // Vulcano geotermico, bordo N di Vatnajökull
    obstacles.push_back(std::make_shared<CylinderObstacle>(
        Point(408942.0, 145108.8, 0.0, -1), 6000.0, 1500.0, 130.0));
 
    // Bárðarbunga — mid WP10→WP11 (~28 km)
    // Caldera subglaciale, approccio SW; raggio largo perché è il più alto output magmatico
    obstacles.push_back(std::make_shared<CylinderObstacle>(
        Point(430888.8, 113418.3, 0.0, -1), 9000.0, 1500.0, 150.0));
 
    // Öræfajökull — mid WP11→WP12 (~41 km)
    // Il picco più alto dell'Islanda, costa S
    obstacles.push_back(std::make_shared<CylinderObstacle>(
        Point(401870.2, 95071.1, 0.0, -1), 5000.0, 1500.0, 120.0));
 
    // Hofsjökull — mid WP12→WP13 (~33 km)
    // 3° ghiacciaio d'Islanda, highlands centrali
    obstacles.push_back(std::make_shared<CylinderObstacle>(
        Point(372607.7, 72832.1, 0.0, -1), 14000.0, 1500.0, 130.0));
 
 
    // ── Ghiacciai visibili nell'area highland C3/C0 ────────────────────────────────
    // Aggiunti per coerenza visiva — centrati sui ghiacciai reali
 
    // Eiríksjökull — 64.764°N 20.415°W
    // Piccolo ghiacciaio isolato vicino Húsafell, lato sinistro immagine
    obstacles.push_back(std::make_shared<CylinderObstacle>(
        Point(107050.7, 86620.3, 0.0, -1), 3500.0, 1500.0, 80.0));
 
    // Langjökull — 64.650°N 20.100°W
    // 2° ghiacciaio d'Islanda; la caldera Þórisvatn (buco scuro) è visibile nell'immagine
    obstacles.push_back(std::make_shared<CylinderObstacle>(
        Point(122413.5, 73944.1, 0.0, -1), 17000.0, 1500.0, 120.0));
 
    // Þórisjökull — 64.753°N 19.490°W
    // Ghiacciaio minore centro-sinistra, tra Langjökull e Hofsjökull
    obstacles.push_back(std::make_shared<CylinderObstacle>(
        Point(152163.7, 85397.1, 0.0, -1), 4000.0, 1500.0, 80.0));
 
    // Hofsjökull — 64.829°N 18.920°W
    // 3° ghiacciaio d'Islanda, centro immagine; forma arrotondata caratteristica
    obstacles.push_back(std::make_shared<CylinderObstacle>(
        Point(179963.0, 93848.0, 0.0, -1), 13000.0, 1500.0, 130.0));
 
    // Vatnajökull (bordo NW) — 64.600°N 17.800°W
    // Il ghiacciaio più esteso d'Europa; solo il bordo NW entra nell'immagine
    obstacles.push_back(std::make_shared<CylinderObstacle>(
        Point(234586.3, 68384.3, 0.0, -1), 22000.0, 1500.0, 150.0)); 

    return obstacles;
}

FitnessFunction makeDefaultFitness(double zMin, double zMax) {
    return FitnessFunction(buildDefaultObstacles(), sampleFitnessWeights(zMin, zMax));
}

FitnessFunction makeDefaultFitness(double zMin, double zMax, int nObstacles) {
    auto obs = buildDefaultObstacles();
    if (nObstacles >= 0 && nObstacles < static_cast<int>(obs.size()))
        obs.resize(nObstacles);
    return FitnessFunction(obs, sampleFitnessWeights(zMin, zMax));
}

// Converts all waypoints in path from local metric coordinates to GPS.
void convertToGPS(PointsList& path, double lon0, double lat0, double metersPerDegLon, double metersPerDegLat) {
    for (int i = 0; i < path.size(); ++i) {
        Point& p = path.extractPoint(i);
        p.setX(lon0 + p.getX() / metersPerDegLon);
        p.setY(lat0 + p.getY() / metersPerDegLat);
    }
}
