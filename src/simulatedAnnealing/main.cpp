#include "geometry/point.hpp"
#include "domain/domain.hpp"
#include "domain/prefabs.hpp"

#include "scheduler/scheduler.hpp"
#include "sampler/uniform_sampler.hpp"
#include "sampler/local_sampler.hpp"
#include "sampler/non_local_sampler.hpp"


#include "sampler/exception.hpp"
#include "functions/continuous.hpp"
#include "functions/discrete.hpp"

#include "algorithms/serial_simulated_annealing.hpp"
#include "algorithms/parallel_scattered_simulated_annealing.hpp"
#include <iostream>
#include <tuple>


int main() {

	constexpr const long max_iterations = 100;

	const lattice_nd<1> low = 0;

	const lattice_nd<1> l = -23;

	const std::shared_ptr<TransitionCriterion> metropolis = std::make_shared<MetropolisCriterion>();

	const auto disc_dom = NnDomainFactory::make_lattice_grid({ 200 }, l);
	
	auto cd = CircleDiscreteNeighbourhood<1>(disc_dom, 200);

	//auto sampler = std::make_shared<LocalSampler<CircleDiscreteNeighbourhood<1>>>(0.8);
	//auto sampler = std::make_shared<NonLocalSampler<CircleDiscreteNeighbourhood<1>>>(0.3);
	auto sampler = std::make_shared<UniformSampler<CircleDiscreteNeighbourhood<1>>>();

	// Two different schedulers are needed, they will be changed by each parallel
	// thread independently!
	auto aggro_sched = std::make_shared<LinearScheduler>(30.0, max_iterations, 20);

	sampler->seed(0xcafebabe);
	metropolis->seed(0xcafebabe);

	AnnealingExecutionPolicy lower_policy(metropolis, sampler, aggro_sched, threshold, cd);

	const auto quintic = TestFunctions::quintic_discrete;

	SerialSimulatedAnnealing ssa(/* Max iterations */ max_iterations);

	const auto point = ssa.run(quintic, low, lower_policy);
	std::cout << "Final result of the serial algorithm: " << point << std::endl;
}