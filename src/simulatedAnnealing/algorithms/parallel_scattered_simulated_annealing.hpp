#pragma once
#ifndef PARALLEL_SCATTERED_SIMULATED_ANNEALING
#define PARALLEL_SCATTERED_SIMULATED_ANNEALING

// OpenMP 
#include <omp.h>

#include <stdexcept>
#include <float.h>
#include "simulated_annealing.hpp"

class ParallelScatteredSimulatedAnnealing : public SimulatedAnnealing {

public:

	ParallelScatteredSimulatedAnnealing(long max_iter,
		long n_threads) :
		SimulatedAnnealing(max_iter), algo_num_threads(n_threads) {
	}

	template <typename Sampler>
	auto run(
		const std::function<double(typename Sampler::Neighbourhood::PointType)>& func,
		std::vector<typename Sampler::Neighbourhood::PointType> p0s,
		std::vector<AnnealingExecutionPolicy<Sampler>> policies

	) -> typename Sampler::Neighbourhood::PointType {

		if (policies.size() != algo_num_threads || policies.size() == 0) {
			throw std::invalid_argument("Each running thread must be assigned an individual policy");
		}

		// Unwrap the type templeting hierarchy
		typedef typename Sampler::Neighbourhood Neighbourhood;
		typedef typename Neighbourhood::PointType PointType;

		PointType ret_val = p0s[0];
		double min_energy = +DBL_MAX; // akin to -inf but avoiding all troubles

#pragma omp parallel num_threads(algo_num_threads)
		{
			// Each thread geets its own starting point in its own domain.
			PointType pt = p0s[omp_get_thread_num()], prev = 0.0;
			const auto& policy = policies[omp_get_thread_num()];

			policy.neighbourhood().from(prev, pt);

			if (!policy.neighbourhood().contains(pt)) {
				throw std::invalid_argument("One of the domains is incorrectly configured so that the initial point of" 
				" a thread was not cointained in its own domain");
			}
			
			double energy, new_energy;
			int iter;

			const auto param = policy.scheduler();
			energy = func(pt);
			iter = 0;
			while (iter < this->get_max_iter()) {

				for (int repeat = 0; repeat < param->stab_it(); ++repeat) {

					policy.neighbourhood().from(prev, pt);
					auto new_p = policy.sampler()->sample(
						policy.neighbourhood(), 1000);

					new_energy = func(new_p);
					// Accept the new value stochastically based on the energy
					bool do_accept = policy.criterion()->accept(
						new_energy - energy, iter,
						param->temp());

					if (do_accept) {
						prev = pt;
						pt = new_p;
						energy = new_energy;
					}

				}
				++iter;
				param->update(iter);
				// Check the zero temperature stopping criterion
				if (policy.stopping() == temp_zero && std::abs(param->temp()) < 1e-6)
					break;
				// Check the threshold stopping criterion
				if (policy.stopping() == threshold && std::abs(energy) < this->get_threshold())
					break;
			}


#pragma omp critical
			{
				if (energy < min_energy) {
					min_energy = energy;
					ret_val = pt;
				}
			}
		}
		return ret_val;
	}

	template <typename Sampler>
	auto run(
		const std::function<double(typename Sampler::Neighbourhood::PointType)>& func,
		std::initializer_list<typename Sampler::Neighbourhood::PointType> p0s,
		// The main object containing the backbone of our simulated annealing
		// algorithm
		std::initializer_list<AnnealingExecutionPolicy<Sampler>> policies

	) -> typename Sampler::Neighbourhood::PointType {
		return run(func, std::vector<typename Sampler::Neighbourhood::PointType>(p0s),
			std::vector<AnnealingExecutionPolicy<Sampler>>(policies));
	}


protected:

	long algo_num_threads;

};

#endif