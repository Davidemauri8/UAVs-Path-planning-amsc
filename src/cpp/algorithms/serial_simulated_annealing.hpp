#pragma once
#ifndef SERIAL_SIMULATED_ANNEALING
#define SERIAL_SIMULATED_ANNEALING

#include "simulated_annealing.hpp"

template <typename PointType, typename Neigh>
class SerialSimulatedAnnealing : public SimulatedAnnealing<PointType, Neigh> {

public:
	using typename SimulatedAnnealing<PointType, Neigh>::min_func;

	template <typename Sampler>
	SerialSimulatedAnnealing(
		const std::shared_ptr<Sampler> _sa,
		typename Sampler::Neighbourhood& _ne,
		const std::shared_ptr<TransitionCriterion>& _tc,
		const StoppingCriterion _sc = temp_zero
	) : SimulatedAnnealing<PointType, Neigh>(_sa, _ne, _tc, _sc) {}

	template <typename ParamScheduler>
	void run(const min_func& func, const PointType& p0, ParamScheduler& param) {
		PointType pt = p0, prev = 0.0;
		double energy, new_energy;
		int iter;

		// Raw skeleton just to see if everything fits together nicely...
		energy = func(p0);
		iter = 0;
		std::cout << "Point0: " << p0 << std::endl;
		while (iter < 300) {

			for (int repeat = 0; repeat < param.stab_it(); ++repeat) {

				this->ne.from(p0, prev);
				auto new_p = this->sa->sample(this->ne);

				new_energy = func(new_p);
				bool do_accept = this->tc->accept(new_energy - energy, iter, param.temp());
				if (do_accept) {
					prev = pt;
					pt = new_p;
					energy = new_energy;
				}
				//std::cout << "Point: " << pt << std::endl;
				//std::cout << "Energy now: " << energy << " temperature: " << param.temp();
			}
			++iter;
			param.update(iter);
		}

		std::cout << "Done!";
	}

	bool success() override {
		return true;
	}

};

template <typename Sampler>
SerialSimulatedAnnealing(
	const std::shared_ptr<Sampler>,
	const typename Sampler::Neighbourhood&,
	const std::shared_ptr<TransitionCriterion>&,
	const StoppingCriterion)
	-> SerialSimulatedAnnealing<typename Sampler::Neighbourhood::PointType, typename Sampler::Neighbourhood>;

#endif