#pragma once
#ifndef ALGO_SIMULATED_ANNEALING
#define ALGO_SIMULATED_ANNEALING


#include <type_traits>

#include "../sampler/uniform_sampler.hpp"
#include "../neighbourhoods/neighbourhood_gen.hpp"
#include "../criterion/criterion.hpp"
#include "../scheduler/scheduler.hpp"

enum StoppingCriterion {
	threshold,
	temp_zero,
	max_iter
};

template <typename PointType, typename Neigh>
class SimulatedAnnealing {

public:
	
	typedef std::function<double(PointType)> min_func;

	virtual bool success() = 0;

	template <typename Sampler>
	SimulatedAnnealing(
		const std::shared_ptr<Sampler> _sa,
		typename Sampler::Neighbourhood& _ne,
		const std::shared_ptr<TransitionCriterion>& _tc,
		const StoppingCriterion _sc = temp_zero
	) : tc(_tc), sc(_sc), ne(_ne), sa(_sa) {
		static_assert(std::is_base_of<SamplerBase<typename Sampler::Neighbourhood>, Sampler>::value);
	}

public:

	// Note: the neighbourhood may change when transitioning from points, so it is 
	// not to be marked as const!
	Neigh& ne;
	const std::shared_ptr<TransitionCriterion> tc;
	const std::shared_ptr<SamplerBase<Neigh>> sa;
	const StoppingCriterion sc;
	const std::shared_ptr<ParamScheduler> ps;

};

#endif //! ALGO_SIMULATED_ANNEALING