#pragma once
#ifndef ANNEALING_POLICY_HPP
#define ANNEALING_POLICY_HPP

#include "../neighbourhoods/neighbourhood_gen.hpp"
#include "../sampler/sampler_base.hpp"
#include "../geometry/point_base.hpp"
#include "../domain/domain.hpp"
#include "../criterion/criterion.hpp"

enum StoppingCriterion {
	threshold,
	temp_zero
};

template <typename Sampler>
class AnnealingExecutionPolicy {

	typedef typename Sampler::Neighbourhood Neighbourhood;
	// Notice that the neighbourhood is not constant as it will 
	// mutate when the annealing process keeps creating 
	// new neighbourhoods. 
	const std::shared_ptr<TransitionCriterion> tc;
	const std::shared_ptr<Sampler> sa;
	std::shared_ptr<ParamScheduler> ps;
	const StoppingCriterion sc;
	Neighbourhood& ne;

public:
	AnnealingExecutionPolicy(
		const std::shared_ptr<TransitionCriterion> _tc,
		const std::shared_ptr<Sampler> _sa,
		const std::shared_ptr<ParamScheduler> _ps,
		const StoppingCriterion _sc,
		Neighbourhood& _ne
	): tc(_tc), sa(_sa), ps(_ps), sc(_sc), ne(_ne) {}

	StoppingCriterion stopping() const {
		return sc;
	}

	std::shared_ptr<TransitionCriterion> criterion() const {
		return tc;
	}

	std::shared_ptr<Sampler> sampler() const {
		return sa;
	}

	std::shared_ptr<ParamScheduler> scheduler() const {
		return ps;
	}

	Neighbourhood& neighbourhood() const {
		return ne;
	}

};

#endif
