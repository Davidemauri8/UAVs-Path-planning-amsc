
#include "../geometry/point.hpp"
#include "../domain/domain.hpp"
#include "../domain/prefabs.hpp"

#include "../scheduler/scheduler.hpp"
#include "../sampler/uniform_sampler.hpp"

#include "../sampler/exception.hpp"
#include "../functions/continuous.hpp"

#include "../algorithms/serial_simulated_annealing.hpp"
#include "../algorithms/parallel_scattered_simulated_annealing.hpp"
#include <iostream>
#include <tuple>


int main() {

	constexpr const long max_iterations = 100;

	const point_nd<1> low_end = 0.25;
	const point_nd<1> high_end = 0.75;

	const auto low_interval = RnDomainFactory::make_sphere(0.25, low_end);
	const auto high_interval = RnDomainFactory::make_sphere(0.25, high_end);

	const std::shared_ptr<TransitionCriterion> metropolis = std::make_shared<MetropolisCriterion>();

	// Two different neighbours are needed, they will be changed by each parallel
	// thread independently!
	auto cn_low = CircleNeighbourhood<1>(low_interval, 3);
	auto cn_up = CircleNeighbourhood<1>(high_interval, 3);

	auto sampler = std::make_shared<UniformSampler<CircleNeighbourhood<1>>>();


	// Two different schedulers are needed, they will be changed by each parallel
	// thread independently!
	auto aggro_sched = std::make_shared<LinearScheduler>(30.0, max_iterations, 20);
	auto weak_sched = std::make_shared<LinearScheduler>(25.0, max_iterations, 20);

	sampler->seed(	0xcafebabe	);
	metropolis->seed(	0xcafebabe	);

	AnnealingExecutionPolicy lower_policy(metropolis, sampler, aggro_sched, threshold, cn_low);
	AnnealingExecutionPolicy upper_policy(metropolis, sampler, weak_sched, threshold, cn_up);

	// The forrester function is a function with a single global minimum, another local minimum and
	// local convexity changes
	const auto forrester = TestFunctions::forrester;

	ParallelScatteredSimulatedAnnealing ssa(/* Max iterations */ max_iterations, /* Num threads*/ 2);

	const auto point = ssa.run(forrester, { low_end, high_end }, { lower_policy, upper_policy });
	std::cout << "Final result of the parallel algorithm: " << point << std::endl;

}