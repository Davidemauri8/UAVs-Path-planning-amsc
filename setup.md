# Guida ai parametri dell'algoritmo

Questo documento elenca **tutti i parametri configurabili** del progetto, indicando per ciascuno il file in cui viene impostato, la funzione/struttura di riferimento e il significato.

---

## 1. Parametri globali dello scenario

**File:** `src/testiceland.cpp` — funzione `main()`

| Parametro | Valore attuale | Descrizione |
|-----------|---------------|-------------|
| `NWaypoints` | `4` | Numero di waypoint intermedi per ogni segmento di percorso |
| `K` | `4` | Numero di droni / cluster K-Means |
| `zMin` | `800.0` | Quota minima di volo (metri, coordinate locali) |
| `zMax` | `900.0` | Quota massima di volo (metri, coordinate locali) |

`NWaypoints` è un parametro template (`constexpr int`): viene propagato a `runPipelineOptimization<NWaypoints>` e quindi a tutti i sotto-algoritmi (SA, DRSTASA, LHS).

---

## 2. Origine geografica (scenario Iceland)

**File:** `src/scenarios/iceland.hpp` — funzione `icelandOrigin()`

| Parametro | Valore | Descrizione |
|-----------|--------|-------------|
| `lat0` | `63.985` | Latitudine dell'origine GPS (Aeroporto di Keflavik) |
| `lon0` | `-22.605` | Longitudine dell'origine GPS |

Tutti i punti del CSV e le coordinate degli ostacoli sono espressi in metri rispetto a questa origine. La conversione GPS↔metri usa `GeoUtils::toMeters` / `GeoUtils::toGPS`.

---

## 3. Fitness function — pesi e vincoli

**File:** `src/structures/functions/fitnessUtilities.cpp` — funzione `sampleFitnessWeights(zMin, zMax)`

Questi pesi vengono usati da **entrambi** gli ottimizzatori (SA e DRSTASA). La struttura `FitnessWeights` è dichiarata in `src/structures/fitness/fitnessFunction.hpp` con tutti i campi a `0.0` di default: i valori effettivi vivono **solo** in `fitnessUtilities.cpp`.

| Parametro | Valore | Descrizione |
|-----------|--------|-------------|
| `b1` | `5.0` | Peso del costo di **lunghezza percorso** (F1) |
| `b2` | `10.0` | Peso del costo di **prossimità agli ostacoli** (F2) |
| `b3` | `1.0` | Peso del costo di **deviazione di quota** (F3) |
| `b4` | `5.0` | Peso del costo di **smoothness del percorso** (F4) |
| `a1` | `1.0` | Sotto-peso per l'**angolo di svolta orizzontale** (dentro F4) |
| `a2` | `1.0` | Sotto-peso per la **variazione di angolo di salita** (dentro F4) |
| `hMin` | `= zMin` | Quota minima ammessa (preso dallo scenario) |
| `hMax` | `= zMax` | Quota massima ammessa (preso dallo scenario) |
| `droneRadius` | `1.5` | Raggio fisico del drone in metri (gonfia il raggio degli ostacoli) |

La fitness totale è: `F = b1·F1 + b2·F2 + b3·F3 + b4·F4`.
Violazioni di quota o collisioni restituiscono `+∞`, scartando la soluzione.

---

## 4. Ostacoli cilindrici

### 4a. Scenario Iceland
**File:** `src/scenarios/iceland.hpp` — funzione `makeIcelandFitness(zMin, zMax)`

Ogni ostacolo è un `CylinderObstacle(Point(x, y, 0, -1), radius, height, buffer)`.

| Ostacolo | x (m) | y (m) | radius (m) | height (m) | buffer (m) | Note |
|----------|-------|-------|-----------|-----------|-----------|------|
| Fagradalsfjall | 16212 | −8880 | 5000 | 385 | 2000 | Eruzioni 2021-23 |
| Hekla | 141480 | −555 | 15000 | 1491 | 5000 | Vulcano più attivo |
| Eyjafjallajökull | 145715 | −39405 | 12000 | 1651 | 4000 | Eruzione 2010 |
| Krafla | 284906 | 192252 | 10000 | 818 | 3000 | Adiacente a Mývatn |

### 4b. Scenario di default / test
**File:** `src/structures/functions/fitnessUtilities.cpp` — funzione `makeDefaultFitness(zMin, zMax)`

| Ostacolo | center (x, y, z) | radius (m) | height (m) | buffer (m) |
|----------|-----------------|-----------|-----------|-----------|
| Ostacolo 1 | (200, 300, 0) | 50 | 200 | 60 |
| Ostacolo 2 | (−300, −200, 0) | 50 | 200 | 60 |

### 4c. Scenario sample / unit test
**File:** `src/structures/functions/fitnessUtilities.cpp` — funzione `sampleFitnessFunction(zMin, zMax)`

| Ostacolo | center (x, y, z) | radius (m) | height (m) | buffer (m) |
|----------|-----------------|-----------|-----------|-----------|
| Ostacolo 1 | (250, 350, 0) | 40 | 200 | 50 |
| Ostacolo 2 | (350, 450, 0) | 40 | 200 | 50 |

**Significato dei campi `CylinderObstacle`:**
- `radius` — raggio di collisione dura (il drone non può entrare, pena fitness = ∞)
- `height` — altezza del cilindro; sopra questa quota l'ostacolo non conta
- `buffer` — spessore della zona di penalità morbida attorno al raggio duro

---

## 5. K-Means

**File:** `src/testiceland.cpp` — riga `KMeans(K, 100).run(allPoints)`

La classe `KMeans` è dichiarata in `src/structures/kmeans.hpp`.

| Parametro | Valore | Descrizione |
|-----------|--------|-------------|
| `K` | `4` (da `testiceland.cpp`) | Numero di cluster (= numero di droni) |
| `max_iterations` | `100` | Iterazioni massime per la convergenza dell'algoritmo |
| `seed` | `42` (default nel costruttore) | Seed per l'inizializzazione casuale dei centroidi |

---

## 6. TSP-SA (ordinamento dei waypoint nel cluster)

**File:** `src/structures/pipeline/pipelineRunner.hpp` — dentro `runPipelineOptimization()`

```cpp
TspSA(fitness,
      std::make_shared<MetropolisCriterion>(),
      std::make_shared<ExponentialScheduler>(100.0, 0.01, 200, 0.95),
      500).run(cluster);
```

| Parametro | Valore | Descrizione |
|-----------|--------|-------------|
| `maxIter` | `500` | Iterazioni massime del SA per ordinare i waypoint |
| Criterio | `MetropolisCriterion` | Criterio di accettazione probabilistica standard |
| Scheduler | `ExponentialScheduler` | Raffreddamento esponenziale |
| T0 (scheduler) | `100.0` | Temperatura iniziale |
| Tmin (scheduler) | `0.01` | Temperatura minima |
| stab_it (scheduler) | `200` | Iterazioni di stabilizzazione per ogni livello T |
| alpha (scheduler) | `0.95` | Fattore di raffreddamento: T(k+1) = T(k) · α |

---

## 7. SA classico per segmento

**File:** `src/structures/segment/segmentOptimizer.hpp`

### 7a. `runSegmentSA()` — singola run SA

```cpp
SegmentSAResult<NWaypoints> runSegmentSA(
    ...,
    long maxIter = 6000)
```

| Parametro | Valore default | Descrizione |
|-----------|---------------|-------------|
| `maxIter` | `6000` | Iterazioni massime della singola run SA |
| Scheduler T0 | `100.0` | Temperatura iniziale |
| Scheduler Tmin | `0.01` | Temperatura minima |
| Scheduler stab_it | `1` | Iterazioni di stabilizzazione per livello T |
| Scheduler alpha | `0.95` | Fattore di raffreddamento |
| `domRadius · 0.1` | calcolato | Raggio del vicinato per il campionamento locale |
| LocalSamplerFixed k | `0.1` | Parametro di località: σ = k · neighbourhood_radius |

Il dominio di ricerca è una **sfera** centrata nel mezzo del bounding box del segmento, con raggio `√NWaypoints · √(Δx² + Δy² + Δz²)`.

### 7b. `runSegmentSAMultiStart()` — multi-start SA

```cpp
SegmentSAResult<NWaypoints> runSegmentSAMultiStart(
    ...,
    int  nRestarts        = 4,
    long maxIterPerRestart = 6000)
```

| Parametro | Valore default | Descrizione |
|-----------|---------------|-------------|
| `nRestarts` | `4` | Numero di run SA indipendenti; viene tenuto il risultato migliore |
| `maxIterPerRestart` | `6000` | Iterazioni per ciascuna run (passato a `runSegmentSA`) |

### 7c. Bounds del segmento — `computeBounds()`

**File:** `src/structures/segment/segmentOptimizer.hpp`

```cpp
SegmentBounds computeBounds(const PointsList& ordered,
                            double marginFactor = 0.2,
                            double marginMin    = 50.0);
```

| Parametro | Valore default | Descrizione |
|-----------|---------------|-------------|
| `marginFactor` | `0.2` | Il bounding box XY viene espanso del 20% su ciascun lato |
| `marginMin` | `50.0` m | Margine assoluto minimo garantito (evita domini degenerati) |

---

## 8. LHS (Latin Hypercube Sampling)

**File:** usato in `src/structures/segment/segmentOptimizer.hpp` e `src/structures/drstasa/drstasa.cpp`

```cpp
Lhs lhs(xMin, xMax, yMin, yMax, zMin, zMax, NWaypoints, idCluster);
```

La classe è dichiarata in `src/structures/lhs/lhs.hpp`.

| Parametro | Sorgente | Descrizione |
|-----------|---------|-------------|
| `xMin, xMax` | bounds del segmento / DRSTASA::Config | Limiti sull'asse X per il campionamento |
| `yMin, yMax` | bounds del segmento / DRSTASA::Config | Limiti sull'asse Y per il campionamento |
| `zMin, zMax` | scenario (800–900 m per Iceland) | Limiti di quota per il campionamento |
| `n` | = `NWaypoints` | Numero di campioni generati (uno per waypoint) |
| `idCluster` | `-1` (neutro) | ID cluster da assegnare ai punti generati |

LHS è usato **solo** per generare il punto di partenza (`startPoint`) di ogni run SA e della popolazione iniziale di DRSTASA.

---

## 9. DRSTASA

**File:** `src/structures/functions/fitnessUtilities.cpp` — funzione `GetConfigurationDRST(NWaypoints)`

La struttura `DRSTASA::Config` è dichiarata in `src/structures/drstasa/drstasa.hpp` con tutti i campi a `0.0` di default. I valori effettivi vivono **solo** in `fitnessUtilities.cpp`.

```cpp
DRSTASA::Config GetConfigurationDRST(int NWaypoints)
```

| Parametro | Valore | Descrizione |
|-----------|--------|-------------|
| `popSize` | `20` | Dimensione della popolazione |
| `maxIter` | `300` | Iterazioni massime dell'algoritmo DRSTASA |
| `T0` | `100.0` | Temperatura iniziale del SA interno |
| `alpha` | `0.93` | Fattore di raffreddamento: T(k+1) = T(k) · α |
| `p` | `0.5` | Soglia di probabilità per la **reverse learning strategy**: se `rand > p` si applica il reverse learning |
| `C0` | `2.0` | Forza iniziale dell'operatore di **disruption**: C(iter) = C0 · (1 − iter/maxIter) |
| `eps_rot` | `150.0` | Step dell'operatore di **rotazione** (Eq. 10 del paper) |
| `eps_trans` | `100.0` | Step dell'operatore di **traslazione** (Eq. 11) |
| `eps_scale` | `0.05` | Perturbazione dell'operatore di **scaling** (Eq. 12) |
| `eps_axis` | `0.05` | Perturbazione dell'operatore di **axis-transform** (Eq. 13) |
| `nWaypoints` | `= NWaypoints` | Numero di waypoint intermedi (passato dal chiamante) |
| `xMin, xMax` | bounds del segmento | Limiti X del dominio di ricerca (impostati in `pipelineRunner.hpp`) |
| `yMin, yMax` | bounds del segmento | Limiti Y del dominio di ricerca (impostati in `pipelineRunner.hpp`) |
| `zMin, zMax` | scenario | Limiti di quota (impostati in `pipelineRunner.hpp`) |

I bounds XY vengono scritti in `pipelineRunner.hpp` subito prima di creare l'istanza DRSTASA:
```cpp
DRSTASA::Config localCfg = drsCfg;   // copia la config base
localCfg.xMin = b.xMin; localCfg.xMax = b.xMax;
localCfg.yMin = b.yMin; localCfg.yMax = b.yMax;
DRSTASA drstasa(fitness, localCfg);
```

### 9a. Reverse Learning Strategy

**File:** `src/structures/drstasa/reverseLearnStrategy.hpp`

Il punto di apprendimento inverso è calcolato come:
- `xBar[d] = r5 · (dimMin[d] + dimMax[d] − x[d])`
- `xNew[d] = (1 − r6) · x[d] + r6 · xBar[d]`

I limiti usati sono quelli di `DRSTASA::Config` (xMin/xMax, yMin/yMax, zMin/zMax). Il parametro `p` controlla con che frequenza si esegue il reverse learning.

### 9b. Controller (Disruption Operator)

**File:** `src/simulatedAnnealing/controller/controller.hpp`

| Parametro | Sorgente | Descrizione |
|-----------|---------|-------------|
| `C0` | `DRSTASA::Config.C0 = 2.0` | Forza iniziale della disruption |
| `total_iters` | `DRSTASA::Config.maxIter = 300` | Totale iterazioni per scalare C nel tempo |

Formula: `C(iter) = C0 · (1.0 − current_iter / total_iters)`

---

## 10. Scheduler di temperatura (SA classico e DRSTASA)

**File:** `src/simulatedAnnealing/scheduler/scheduler.hpp`

### LinearScheduler
```cpp
LinearScheduler(double T0, double delta, double Tmin, long stab_it)
```
T(k+1) = T(k) − delta

| Parametro | Descrizione |
|-----------|-------------|
| `T0` | Temperatura iniziale |
| `delta` | Decremento fisso per ogni step |
| `Tmin` | Temperatura minima (pavimento) |
| `stab_it` | Iterazioni di stabilizzazione a ogni livello T |

### ExponentialScheduler *(quello usato nel progetto)*
```cpp
ExponentialScheduler(double T0, double Tmin, long stab_it, double alpha = 0.95)
```
T(k+1) = T(k) · alpha

| Parametro | Descrizione |
|-----------|-------------|
| `T0` | Temperatura iniziale |
| `Tmin` | Temperatura minima |
| `stab_it` | Iterazioni di stabilizzazione a ogni livello T |
| `alpha` | Fattore di raffreddamento ∈ (0, 1) |

---

## 11. Riepilogo: dove toccare cosa

| Cosa si vuole cambiare | File da modificare | Funzione/riga |
|------------------------|-------------------|---------------|
| Numero di droni K | `src/testiceland.cpp` | `constexpr int K` |
| Waypoint per segmento | `src/testiceland.cpp` | `constexpr int NWaypoints` |
| Quota di volo | `src/testiceland.cpp` | `const double zMin, zMax` |
| Pesi fitness (b1–b4, a1–a2) | `src/structures/functions/fitnessUtilities.cpp` | `sampleFitnessWeights()` |
| Raggio drone | `src/structures/functions/fitnessUtilities.cpp` | `sampleFitnessWeights()` → `droneRadius` |
| Ostacoli scenario Iceland | `src/scenarios/iceland.hpp` | `makeIcelandFitness()` |
| Ostacoli scenario default | `src/structures/functions/fitnessUtilities.cpp` | `makeDefaultFitness()` |
| Parametri DRSTASA | `src/structures/functions/fitnessUtilities.cpp` | `GetConfigurationDRST()` |
| Iterazioni SA per segmento | `src/structures/segment/segmentOptimizer.hpp` | `runSegmentSAMultiStart()` → `maxIterPerRestart`, `nRestarts` |
| Scheduler SA per segmento | `src/structures/segment/segmentOptimizer.hpp` | `runSegmentSA()` → `ExponentialScheduler(...)` |
| Iterazioni TSP-SA | `src/structures/pipeline/pipelineRunner.hpp` | `TspSA(..., 500)` |
| Scheduler TSP-SA | `src/structures/pipeline/pipelineRunner.hpp` | `ExponentialScheduler(100.0, 0.01, 200, 0.95)` |
| K-Means max iterazioni | `src/testiceland.cpp` | `KMeans(K, 100)` |
| Margini bounding box | `src/structures/segment/segmentOptimizer.hpp` | `computeBounds()` → `marginFactor`, `marginMin` |
| Numero di thread OMP | `src/testiceland.cpp` | `runPipelineOptimization<NWaypoints>(..., /*numThreads=*/1)` |
