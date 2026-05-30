#pragma once
#ifndef PARALLEL_COMMON_SIMULATED_ANNEALING
#define PARALLEL_COMMON_SIMULATED_ANNEALING

// OpenMP is required for the parallelization
#include <omp.h>

#include <stdexcept>
#include <float.h>
#include "simulated_annealing.hpp"

class ParallelCommonSimulatedAnnealing : public SimulatedAnnealing {

public:

	ParallelCommonSimulatedAnnealing(long max_iter,
		long n_threads) :
		SimulatedAnnealing(max_iter), algo_num_threads(n_threads) { }

	template <typename Sampler>
	auto run(
		const std::function<double(typename Sampler::Neighbourhood::PointType)>& func,
		const typename Sampler::Neighbourhood::PointType& p0,
		// The main object containing the backbone of our simulated annealing
		std::vector<AnnealingExecutionPolicy<Sampler>> policies
		
	) -> typename Sampler::Neighbourhood::PointType {

		if (policies.size() != algo_num_threads) {
			throw std::invalid_argument("Each running thread must be assigned an individual policy");
		}

		// Unwrap the type templeting hierarchy
		typedef typename Sampler::Neighbourhood Neighbourhood;
		typedef typename Neighbourhood::PointType PointType;

		PointType ret_val = p0;
		double min_energy = +DBL_MAX; 

		#pragma omp parallel num_threads(algo_num_threads)
		{
			// In a common parallel implementation, all threads start
			// from the same point and may end up in different regions of the
			// space due to the stochasticity. 

			PointType pt = p0, prev = 0.0;
			double energy, new_energy;
			int iter;

			const auto& policy = policies[omp_get_thread_num()];
			const auto param = policy.scheduler();
			energy = func(p0);
			iter = 0;
			while (iter < this->get_max_iter()) {

				for (int repeat = 0; repeat < param->stab_it(); ++repeat) {

					policy.neighbourhood().from(prev, p0);
					auto new_p = policy.sampler()->sample(
						policy.neighbourhood(), 1000);

					new_energy = func(new_p);
					// Accept the new value stochastically based on the energy
					// delta and the current temperature
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

			// atomically update the minimum value.
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
		const typename Sampler::Neighbourhood::PointType& p0,
		// The main object containing the backbone of our simulated annealing
		std::initializer_list<AnnealingExecutionPolicy<Sampler>> policies

	) -> typename Sampler::Neighbourhood::PointType {
		return run(func, p0, std::vector<AnnealingExecutionPolicy<Sampler>>(policies));
	}

protected:

	long algo_num_threads;

};

#endif