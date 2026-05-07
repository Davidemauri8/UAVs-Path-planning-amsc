#ifndef TSP_SA_HPP
#define TSP_SA_HPP

#include "tspNeighbourhood.hpp"
#include "tspSampler.hpp"
#include "../fitness/fitnessFunction.hpp"
#include "criterion/criterion.hpp"
#include "scheduler/scheduler.hpp"
#include <memory>

// Simulated Annealing solver for the Travelling Salesman ordering of cluster waypoints.
// Finds a visit order that minimises the path fitness, providing a good starting
// permutation before the per-segment SA/DRSTASA optimisation.
class TspSA {
public:
    TspSA(const FitnessFunction& fitness,
          std::shared_ptr<TransitionCriterion> criterion,
          std::shared_ptr<ParamScheduler> scheduler,
          long maxIter = 1000)
        : fitness_(fitness)
        , criterion_(criterion)
        , scheduler_(scheduler)
        , maxIter_(maxIter) {}

    // Runs the SA loop starting from initialPath.
    // At each temperature level, stab_it() candidate permutations are evaluated
    // and accepted/rejected via the Metropolis criterion.
    // Returns the best permutation found across all iterations.
    PointsList run(const PointsList& initialPath) {

        // A cluster with a single point has no ordering to optimise.
        if (initialPath.size() < 2) return initialPath;

        PointsList current  = initialPath;
        PointsList best     = initialPath;
        double currentFit   = fitness_.evaluate(current);
        double bestFit      = currentFit;

        TspNeighbourhood neigh(current);
        TspSampler sampler;

        int iter = 0;
        while (iter < maxIter_ && scheduler_->temp() > 1e-6) {

            for (int r = 0; r < scheduler_->stab_it(); ++r) {

                neigh.from(current, current);
                PointsList candidate = sampler.sample(neigh);

                double candidateFit = fitness_.evaluate(candidate);
                double delta = candidateFit - currentFit;

                if (criterion_->accept(delta, iter, scheduler_->temp())) {
                    current    = candidate;
                    currentFit = candidateFit;

                    if (currentFit < bestFit) {
                        best    = current;
                        bestFit = currentFit;
                    }
                }
            }

            ++iter;
            scheduler_->update(iter);
        }

        return best;
    }

private:
    const FitnessFunction& fitness_;
    std::shared_ptr<TransitionCriterion> criterion_;
    std::shared_ptr<ParamScheduler> scheduler_;
    long maxIter_;
};

#endif
