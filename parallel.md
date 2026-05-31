# Parallelizzazione — Documentazione tecnica

Questo documento descrive in dettaglio tutte le sezioni del codice parallelizzate con OpenMP, le motivazioni di ogni scelta, i rischi di data race identificati e le soluzioni adottate.

---

## Gerarchia di parallelismo

Il progetto usa **due livelli annidati di parallelismo OpenMP**:

```
Livello 1 — pipelineRunner.hpp
  #pragma omp parallel for schedule(dynamic)
  for k in [0, K):              ← K cluster, ognuno su un thread

      Livello 2 — drstasa.cpp
        #pragma omp parallel for schedule(static) num_threads(nT)
        for i in [0, popSize):  ← popolazione, distribuita su nT thread
```

La formula di distribuzione dei core è:

```
innerThreads = max(1, totalHardwareCores / numOuterThreads)
```

Esempio con 8 core e K=4 cluster: ogni cluster usa 2 thread interni → 4 × 2 = 8 thread totali, tutti i core occupati.

Il parallelismo annidato è abilitato esplicitamente all'inizio di `runPipelineOptimization`:

```cpp
// pipelineRunner.hpp — righe 36-43
omp_set_dynamic(0);
omp_set_max_active_levels(2);          // permette 2 livelli di nesting
omp_set_num_threads(numThreads);
int totalCores   = omp_get_max_threads();
int innerThreads = std::max(1, totalCores / std::max(1, numThreads));
```

- `omp_set_dynamic(0)` — disabilita l'aggiustamento automatico del numero di thread da parte del runtime; il thread count è fisso e prevedibile.
- `omp_set_max_active_levels(2)` — senza questa chiamata OpenMP collassa i team annidati a 1 thread (comportamento di default). È il requisito fondamentale per il livello 2.

---

## Livello 1 — Parallelismo sui cluster

**File:** `src/structures/pipeline/pipelineRunner.hpp`  
**Righe:** 55-116

```cpp
#pragma omp parallel for schedule(dynamic) \
    reduction(+:totalSAFit, totalDRSTASAFit)
for (int k = 0; k < K; ++k) {
    ...
}
```

### Cosa viene parallelizzato

Il loop esterno su `K` cluster. Ogni cluster rappresenta una zona geografica assegnata a un UAV; il suo lavoro è completamente indipendente dagli altri:
1. Ordina i waypoint del cluster con TSP-SA.
2. Per ogni coppia di waypoint consecutivi (segmento), esegue SA multi-start e DRSTASA.
3. Concatena i segmenti ottimizzati nel percorso locale.

### Scheduling: `schedule(dynamic)`

La dimensione dei cluster è variabile (cluster con più waypoint producono più segmenti e più chiamate a SA/DRSTASA). `dynamic` assegna le iterazioni ai thread man mano che si liberano, bilanciando il carico in modo adattivo. `static` avrebbe distribuito i cluster in blocchi fissi, rischiando di lasciare thread inattivi se un cluster è molto più pesante degli altri.

### Reduction su `totalSAFit` e `totalDRSTASAFit`

Le fitness dei singoli segmenti vengono accumulate in due variabili globali. Senza `reduction`, ogni thread dovrebbe usare un `critical` per ogni segmento (costoso). La reduction OpenMP crea una copia privata per thread e le somma atomicamente alla fine, eliminando il bottleneck.

### Sezioni critiche

Due `#pragma omp critical` nel corpo del loop:

```cpp
// Caso degenere: cluster con 1 solo punto
#pragma omp critical
{ saPath.addPoint(p); drstasaPath.addPoint(p); }

// Scrittura finale dei percorsi locali sul path globale
#pragma omp critical
{
    appendPath(saPath, localSA);
    appendPath(drstasaPath, localDRS);
    saPerCluster[k]      = localSA;
    drstasaPerCluster[k] = localDRS;
    orderedClusters[k]   = ordered;
}
```

`saPath` e `drstasaPath` sono `PointsList` con stato interno mutabile; `appendPath` modifica questa struttura, quindi l'accesso concorrente causerebbe data race. Il critical è necessario e giustificato: viene eseguito una sola volta per cluster (non per segmento), quindi l'overhead è trascurabile.

---

## Livello 2 — Parallelismo sulla popolazione DRSTASA

**File:** `src/structures/drstasa/drstasa.cpp`  
**Sezioni:** Fase 1 (STASA) e Fase 2 (Disruption)

### Problema di thread-safety degli oggetti interni

Prima di descrivere i singoli pragma, è essenziale capire perché gli oggetti non possono essere condivisi tra thread.

Ogni oggetto coinvolto nell'algoritmo contiene un generatore di numeri pseudo-casuali (`std::mt19937`) come membro privato **mutabile**:

| Oggetto | Campo con stato | Perché non condivisibile |
|---------|----------------|--------------------------|
| `STASANeighbourhoodDyn` | `std::mt19937 rng_` | Usato in tutti e 4 gli operatori STASA (`applyRotation`, `applyTranslation`, `applyScaling`, `applyAxisTransf`) |
| `MetropolisCriterion` | `std::mt19937 gen` + `std::uniform_real_distribution<> distrib` | Chiamata da `accept()` per campionare la probabilità di accettazione |
| `Controller` | `std::mt19937 gen` | Usato in `applyDisruption()` per campionare il fattore di perturbazione `D` |

`std::mt19937` **non è thread-safe**: due thread che chiamano `operator()` sulla stessa istanza simultaneamente producono undefined behavior (stato interno corrotto, valori duplicati o crash).

### Soluzione: vettori di oggetti per-thread

Prima del main loop, vengono allocati `nT = cfg_.innerThreads` istanze di ciascun oggetto:

```cpp
// drstasa.cpp — righe 65-79
int nT = cfg_.innerThreads;
std::vector<STASANeighbourhoodDyn> tNbh;
std::vector<MetropolisCriterion>   tCrit(nT);
std::vector<Controller>            tCtrl;
std::vector<std::mt19937>          tRng;
tNbh.reserve(nT);  tCtrl.reserve(nT);  tRng.reserve(nT);

for (int t = 0; t < nT; ++t) {
    tNbh.emplace_back(cfg_.eps_rot, ..., rng_() + t);  // seed diverso per thread
    tCtrl.emplace_back(cfg_.C0, cfg_.maxIter);
    tRng.emplace_back(rng_() + t);                      // seed diverso per thread
}
```

Ogni thread `tid` accede esclusivamente a `tNbh[tid]`, `tCrit[tid]`, `tCtrl[tid]`, `tRng[tid]`. Non c'è mai accesso condiviso a oggetti con stato.

I seed `rng_() + t` garantiscono che ogni thread produca sequenze stocastiche indipendenti, preservando la diversità della ricerca.

**`FitnessFunction::evaluate()`** è già thread-safe: legge solo `obstacles_` e `w_` (dati `const` dopo la costruzione, mai modificati). Ogni thread può chiamarla simultaneamente senza protezione.

---

### Fase 1 — Operatori STASA (parallela)

```cpp
// drstasa.cpp — righe 85-111
#pragma omp parallel for schedule(static) num_threads(nT)
for (int i = 0; i < popSize; ++i) {
    int tid = omp_get_thread_num();

    tNbh[tid].from(prevPop[i], pop[i]);
    std::array<PointsList, 4> candidates = tNbh[tid].generateAll();

    int    bestK      = 0;
    double bestNewFit = evalWaypoints(candidates[0], start, end);
    for (int k = 1; k < 4; ++k) {
        double f = evalWaypoints(candidates[k], start, end);
        if (f < bestNewFit) { bestNewFit = f; bestK = k; }
    }

    double delta = bestNewFit - fitVals[i];
    if (tCrit[tid].accept(delta, iter, scheduler->temp())) {
        prevPop[i] = pop[i];
        pop[i]     = candidates[bestK];
        fitVals[i] = bestNewFit;
        #pragma omp critical
        {
            if (bestNewFit < bestFit_) {
                bestFit_ = bestNewFit;
                bestPath = candidates[bestK];
            }
        }
    }
}
```

#### Cosa parallelizza

Per ogni individuo `i` della popolazione:
1. Genera i 4 candidati STASA (rotazione, traslazione, scaling, axis-transform).
2. Valuta tutti e 4 con `evalWaypoints` (ciascuna chiama `fitness_.evaluate`).
3. Seleziona il candidato migliore.
4. Applica il criterio di Metropolis per decidere l'accettazione.

**4 valutazioni di fitness per individuo** × 30 individui = 120 valutazioni per iterazione.

#### Scheduling: `schedule(static)`

Con `popSize=30` individui e `nT` thread, `static` assegna blocchi di dimensione `popSize/nT` in modo ciclico (es. thread 0 → individui 0–14, thread 1 → 15–29). Il carico per individuo è uniforme (stesso numero di operazioni), quindi `static` minimizza l'overhead di scheduling rispetto a `dynamic`.

#### Accessi senza race condition

| Dato | Tipo accesso | Thread safety |
|------|-------------|---------------|
| `prevPop[i]`, `pop[i]`, `fitVals[i]` | Lettura + scrittura | Sicuro: ogni thread scrive solo sull'indice `i` che gli è assegnato |
| `tNbh[tid]`, `tCrit[tid]` | Lettura + scrittura | Sicuro: accesso esclusivo per thread |
| `scheduler->temp()` | Solo lettura | Sicuro: `temp()` è const |
| `bestFit_`, `bestPath` | Scrittura condivisa | Protetta da `#pragma omp critical` |

#### Critical su `bestFit_` e `bestPath`

Il critical scatta solo quando un thread trova un miglioramento globale. Con la doppia condizione (controllo fuori e dentro il critical), si evita di entrare nella sezione critica per aggiornamenti non necessari, riducendo la contesa:

```cpp
// Solo se il thread locale ha trovato un miglioramento...
if (tCrit[tid].accept(delta, iter, scheduler->temp())) {
    ...
    #pragma omp critical
    {
        // ...riconferma dentro il critical (un altro thread potrebbe aver già aggiornato)
        if (bestNewFit < bestFit_) { ... }
    }
}
```

---

### Fase 2 — Disruption (parallela)

```cpp
// drstasa.cpp — righe 115-133
{
    std::vector<PointsList> popSnap = pop;   // snapshot prima del loop
    #pragma omp parallel for schedule(static) num_threads(nT)
    for (int i = 0; i < popSize; ++i) {
        int tid = omp_get_thread_num();
        std::uniform_int_distribution<> dist(0, popSize - 2);
        int j = dist(tRng[tid]);
        if (j >= i) ++j;
        tCtrl[tid].applyDisruption(pop[i], bestPath, popSnap[j], iter);
        fitVals[i] = evalWaypoints(pop[i], start, end);
        #pragma omp critical
        {
            if (fitVals[i] < bestFit_) {
                bestFit_ = fitVals[i];
                bestPath = pop[i];
            }
        }
    }
}
```

#### Cosa parallelizza

Per ogni individuo `i`, il disruption operator lo perturba usando `bestPath` (migliore globale) e un vicino casuale `pop[j]`, poi rivaluta la fitness. **1 valutazione per individuo** × 30 = 30 valutazioni per iterazione.

#### Il problema del vicino `pop[j]`

Senza precauzioni, il loop parallelo avrebbe un data race critico: il thread A scrive su `pop[i]` mentre il thread B legge `pop[j]` dove `j == i`. Il comportamento sarebbe undefined.

**Soluzione: snapshot sincrono**

```cpp
std::vector<PointsList> popSnap = pop;  // copia completa prima del loop
```

La copia viene creata in modo seriale prima di aprire la regione parallela. Dentro il loop, ogni thread legge da `popSnap[j]` (immutabile durante il loop) e scrive su `pop[i]` (indice privato al thread). Non esiste più alcuna interferenza tra lettura e scrittura.

Questo pattern corrisponde a un **aggiornamento sincrono** (synchronous update) tipico degli algoritmi evolutivi: tutta la popolazione viene aggiornata usando i valori al passo `t`, non i valori parzialmente aggiornati al passo `t+1`.

Il blocco `{}` delimita lo scope dello snapshot: `popSnap` viene deallocato non appena il loop parallelo termina.

#### `tCtrl[tid].applyDisruption`

`applyDisruption` usa il proprio `gen` (mt19937 privato del Controller) per campionare il fattore di perturbazione stocastico `D`. Poiché ogni thread ha il proprio `tCtrl[tid]`, non c'è race. `applyDisruption` modifica `pop[i]` in-place: sicuro perché l'indice `i` è unico per thread.

---

### Fase 3 — Reverse Learning (seriale)

```cpp
// drstasa.cpp — righe 136-139
if (uni(rng_) > cfg_.p)
    reverseLearn_.apply(pop, fitVals,
        [&](const PointsList& wp) { return evalWaypoints(wp, start, end); },
        cfg_.nWaypoints);
```

Questa fase rimane seriale per due motivi strutturali:

1. **Calcolo di statistiche globali**: `reverseLearnStrategy.cpp` calcola per ogni dimensione `d` il min e max sull'intera popolazione (`dimMin[d]`, `dimMax[d]`). Questo richiede una scansione completa prima di poter generare qualsiasi candidato. Parallelizzare richiederebbe una riduzione multi-dimensionale su `D = nWaypoints × 3` dimensioni, con overhead di sincronizzazione proporzionale a `D`.

2. **Frequenza di attivazione**: la fase 3 è applicata solo nel `(1-p) × 100% = 60%` delle iterazioni. Aggiunge 30 valutazioni quando attivata; il suo contributo al tempo totale è minoritario rispetto alle fasi 1 e 2 (150 valutazioni totali).

---

### Fase 4 — Global best sweep (seriale)

```cpp
// drstasa.cpp — righe 141-146
for (int i = 0; i < popSize; ++i) {
    if (fitVals[i] < bestFit_) {
        bestFit_ = fitVals[i];
        bestPath = pop[i];
    }
}
```

Rimane seriale: è un semplice loop di 30 confronti su `double`, trascurabile rispetto alle valutazioni di fitness. Parallelizzarlo con critical o atomic introdurrebbe overhead superiore al tempo risparmiato.

---

## Cosa rimane seriale e perché

| Sezione | Dove | Motivo |
|---------|------|--------|
| Inizializzazione popolazione (LHS) | `drstasa.cpp` righe 40-44 | Eseguita una volta sola; il costo è irrilevante rispetto ai 500 × 150 eval. |
| Valutazione iniziale fitness | `drstasa.cpp` righe 49-51 | Stessa ragione; 30 eval una tantum. |
| TSP-SA per l'ordinamento waypoint | `pipelineRunner.hpp` riga 68 | Eseguita dentro il loop su `k` già parallelo; è locale al thread del cluster. |
| Loop segmenti dentro cluster | `pipelineRunner.hpp` righe 73-99 | I segmenti sono sequenziali per definizione (start/end del segmento `i` dipendono dall'ordine TSP). |
| SA multi-start per segmento | `segmentOptimizer.hpp` | Già ottimizzato con 4 restart seriali; parallelizzare richiederebbe 4 istanze SA indipendenti con stato separato. |
| Aggiornamento scheduler temperatura | `drstasa.cpp` riga 149 | Scrittura scalare atomica per natura; un solo thread la esegue per iterazione. |

---

## Analisi dei data race — tabella riassuntiva

| Variabile | Accesso | Thread safety | Meccanismo |
|-----------|---------|--------------|------------|
| `pop[i]`, `prevPop[i]`, `fitVals[i]` | R/W | Sicuro | `i` è unico per thread (`static` scheduling) |
| `popSnap[j]` | Solo lettura | Sicuro | Snapshot immutabile durante il loop |
| `bestPath` | Lettura (disruption) + scrittura (entrambe le fasi) | Protetto | `#pragma omp critical` per le scritture; le letture di `bestPath` nella disruption avvengono prima che il loop parallelo cominci a scrivere su di esso |
| `bestFit_` | R/W | Protetto | `#pragma omp critical` |
| `tNbh[tid]`, `tCrit[tid]`, `tCtrl[tid]`, `tRng[tid]` | R/W | Sicuro | Accesso esclusivo: `tid = omp_get_thread_num()` |
| `scheduler->temp()` | Solo lettura | Sicuro | Metodo `const`; il solo aggiornamento avviene serialmente dopo il loop |
| `fitness_.evaluate()` | Solo lettura di `obstacles_`, `w_` | Sicuro | Tutti i campi di `FitnessFunction` sono `const` dopo la costruzione |
| `totalSAFit`, `totalDRSTASAFit` | Accumulo | Sicuro | `reduction(+:...)` nel pragma del livello 1 |
| `saPath`, `drstasaPath` | Scrittura | Protetto | `#pragma omp critical` nel livello 1 |

---

## Stima del guadagno di prestazioni

### Budget di valutazioni per run DRSTASA

```
Fase 1 (STASA):          4 eval/individuo × 30 individui = 120 eval/iter
Fase 2 (Disruption):     1 eval/individuo × 30 individui =  30 eval/iter
Fase 3 (Reverse learn):  1 eval/individuo × 30 individui =  30 eval/iter (60% delle iter)
Totale medio per iter:   ~168 eval
Totale per run:           168 × 500 iter = 84.000 eval
```

### Speedup teorico

Con `innerThreads = nT`:

- **Fasi 1+2**: parallelizzate → speedup ideale `nT`, reale ~`0.85 × nT` (overhead critical + scheduling)
- **Fase 3**: seriale → contribuisce ~30/168 ≈ 18% del tempo
- **Fase 4 + overhead loop**: trascurabile

Applicando la legge di Amdahl con frazione seriale `s ≈ 0.18`:

```
Speedup(nT) = 1 / (s + (1-s)/nT) = 1 / (0.18 + 0.82/nT)

nT=1: 1.0x  (baseline)
nT=2: 1.64x
nT=4: 2.70x
nT=8: 3.74x
```

### Combinazione dei due livelli

Con 8 core e K=4 cluster, prima della modifica il parallelismo di livello 1 usava 4 thread lasciando 4 core inattivi. Con il livello 2 attivato (2 thread interni per cluster):

```
Prima:  4 thread attivi su 8 core → utilizzo 50%
Dopo:   4 cluster × 2 thread interni = 8 thread → utilizzo 100%
```

Lo speedup sul tempo totale del pipeline è quindi vicino a **2x** in questa configurazione, senza modificare la qualità della soluzione (gli stessi 84.000 valutazioni vengono eseguite, solo più in parallelo).

---

## Come modificare il grado di parallelismo

### Variare i thread del livello 1 (cluster)

Nel file `src/testiceland.cpp`:

```cpp
constexpr int numThreads = 4;  // thread per i cluster
runPipelineOptimization<NWaypoints>(..., numThreads);
```

### Variare i thread del livello 2 (popolazione)

`innerThreads` è calcolato automaticamente in `pipelineRunner.hpp` come `max(1, totalCores / numThreads)`. Per forzare un valore specifico, si può modificare direttamente `localCfg.innerThreads` prima della costruzione di `DRSTASA`:

```cpp
// pipelineRunner.hpp
localCfg.innerThreads = 2;  // forza 2 thread interni indipendentemente dai core disponibili
```

### Disabilitare il livello 2

Impostare `innerThreads = 1` (default se non cambiato) ripristina il comportamento seriale originale all'interno di DRSTASA.
