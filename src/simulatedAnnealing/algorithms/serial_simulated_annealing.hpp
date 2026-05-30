#pragma once
#ifndef SERIAL_SIMULATED_ANNEALING
#define SERIAL_SIMULATED_ANNEALING

#include "simulated_annealing.hpp"
#include "../sampler/exception.hpp"

class SerialSimulatedAnnealing : public SimulatedAnnealing {

public:

	SerialSimulatedAnnealing(long max_iter) : SimulatedAnnealing(max_iter) {}

	template <typename Sampler>
	auto run(
		const std::function<double(typename Sampler::Neighbourhood::PointType)>& func,
		const typename Sampler::Neighbourhood::PointType& p0,
		// The main object containing the backbone of our simulated annealing

		AnnealingExecutionPolicy<Sampler>& policy
	) -> typename Sampler::Neighbourhood::PointType {

		// Unwrap the type templeting hierarchy
		typedef typename Sampler::Neighbourhood Neighbourhood;
		typedef typename Neighbourhood::PointType PointType;

		PointType pt = p0, prev = 0.0;
		double energy, new_energy;
		int iter;

		const auto param = policy.scheduler();
		energy = func(p0);
		iter = 0;
		while (iter < this->get_max_iter()) {

			for (int repeat = 0; repeat < param->stab_it(); ++repeat) {

				policy.neighbourhood().from(prev, pt);

				PointType new_p = pt;
				try {
					new_p = policy.sampler()->sample(
						policy.neighbourhood(), 1000);
				} catch (const sa::SamplerFailure&) {
					// Domain and step-size mismatch 
					continue;
				}

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
			//std::cout << "Energy: " << energy << std::endl;
		}

		return pt;
	}

};

#endif