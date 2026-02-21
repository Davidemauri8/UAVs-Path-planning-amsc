
#include "geometry/point.hpp"
#include "geometry/lattice_point.hpp"
#include "domain/domain.hpp"
#include "scheduler/scheduler.hpp"
#include "sampler/uniform_sampler.hpp"
#include "functions/continuous.hpp"
#include "functions/combinatorics.hpp"

#include "algorithms/serial_simulated_annealing.hpp"
#include <iostream>

int main() {

	// Test points
	{
		point_nd<3> p = 1.0;
		point_nd<3> w = p;
		point_nd<3> l(p);
		point_nd<4> uu = { 1.0 };
		point_nd<3> mm = p + w;
		point_nd<3> mz = -p + mm;
		point_nd<3> q = { 1.0, 2.0, 3.0 };
		q += mz;
	}
	const point_nd<1> c = 0.5;
	const lattice_nd<1> l = 0.5;

	const auto sphere = RnDomainFactory::make_sphere(0.5, c);
	// const auto disc_dom = NnDomainFactory::make_lattice_grid( {2}, l);
	// const auto domain = RnDomainFactory::make_sphere(1, p);
	// const auto domainrec = RnDomainFactory::make_hypercube(2, p);
	// const auto twospheres = make_domain_union(domain, domainrec);

	std::shared_ptr<TransitionCriterion> mc = std::make_shared<MetropolisCriterion>();
	StoppingCriterion sc = threshold;
	//auto cn = std::make_shared<SquareNeighbourhood<1>>(std::make_shared<point_nd<1>>(c), 0.1);
	auto cn = std::make_shared<CircleNeighbourhood<1>>(sphere, 0.1);
	auto us = std::make_shared<LocalSampler<CircleNeighbourhood<1>>>(0.02);
	
	//auto cnd = std::make_shared<CircleDiscreteNeighbourhood<1>>(disc_dom, 1);
	//auto usd = std::make_shared<UniformSampler<CircleDiscreteNeighbourhood<1>>>();

	auto sched = LinearScheduler();

	SerialSimulatedAnnealing ssa(us, *cn, mc, sc);
	
	point_nd<1> lll = 0.1;
	//lattice_nd<1> lll = 1;

	cn->from(lll, lll);
	for (int i = 0; i < 10; ++i)
		std::cout << us->sample(*cn) << std::endl;
	const auto forrester = TestFunctions::forrester;

	ssa.run(forrester, lll, sched);

}