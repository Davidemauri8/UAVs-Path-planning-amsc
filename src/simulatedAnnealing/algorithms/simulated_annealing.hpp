#pragma once
#ifndef ALGO_SIMULATED_ANNEALING
#define ALGO_SIMULATED_ANNEALING

#include <type_traits>

#include "../sampler/uniform_sampler.hpp"
#include "../neighbourhoods/neighbourhood_gen.hpp"
#include "../criterion/criterion.hpp"
#include "../scheduler/scheduler.hpp"
#include "annealing_policy.hpp"

class SimulatedAnnealing {

public:

	SimulatedAnnealing(
		long max_it
	) :
		max_iter(max_it) {
	}

	void set_max_iter(long max_it) { max_iter = max_it; };
	long get_max_iter() const { return max_iter; };

	void set_threshold(double t) {
		threshold = t;
	}

	double get_threshold() const { return threshold; }

protected:

	long max_iter;
	double threshold;
};

#endif 