#pragma once
#ifndef UNIFORM_SAMPLER_HPP
#define UNIFORM_SAMPLER_HPP

#include "../neighbourhoods/neighbourhood_gen.hpp"
#include "../geometry/combination.hpp"

#include "sampler_base.hpp"
#include "exception.hpp"

#include <random>

// Sampler that samples with equal probability an element from the neighbourhood
typedef unsigned int domain_dim;


template <typename Neigh>
class UniformSampler : public SamplerBase<Neigh> {
	using Neighbourhood = Neigh;
	
};

// ---------------------------------------------------------
// CONTINUOUS
// ---------------------------------------------------------


template<domain_dim Dim>
class UniformSampler<SquareNeighbourhood<Dim>> : public SamplerBase<SquareNeighbourhood<Dim>> {
public:
	using Neighbourhood = SquareNeighbourhood<Dim>;

	std::random_device rd;
	std::mt19937 gen;
	std::uniform_real_distribution<double> distrib;

public:

	UniformSampler()
		: rd(), gen(rd()), distrib(0, 1.0) {}

	point_nd<Dim> sample(const SquareNeighbourhood<Dim>& n, int max_it) override {

		point_nd<Dim> p = 0.0;

		// a and b will be used to rescale the normal distribution
		double a = 0.0;
		double b = 0.0;

		int it = 0;

		do {
			it += 1;
			for (int i = 0; i < Dim; ++i) {
				a = n.center[i] - n.radius;
				b = n.center[i] + n.radius;
				// rescaling the distribution
				p[i] = a + (b - a) * distrib(gen);

			}
		} while (!n.contains(p) && it < max_it);
		
		if ( it == max_it ) {
			throw sa::SamplerFailure("Iteration missed after " +
					std::to_string(max_it) + " attempts");
		}

		//return the point extracted
		return p;
	}

	void seed(long seed) override {
		gen.seed(seed);
	}

};

template<domain_dim Dim>
class UniformSampler<CircleNeighbourhood<Dim>> : public  SamplerBase<CircleNeighbourhood<Dim>> {

	std::random_device rd;
	std::mt19937 gen;
	std::normal_distribution<double> nd;

public:

	UniformSampler() 
		: rd(), gen(rd()), nd(0, 1.0) { }

	using Neighbourhood = CircleNeighbourhood<Dim>;

	point_nd<Dim> sample(const CircleNeighbourhood<Dim>& n, int max_it) override {

		point_nd<Dim> p = 0.0;
		std::uniform_real_distribution<> distrib(0, n.radius);

		int it = 0;

		do {

			it += 1;
			const double radius = distrib(gen);

			for (int i = 0; i < Dim; ++i) {
				p[i] = nd(gen);
			}
			// Only normalize if the sampled unitary point is nonzero... which should be everytime!
			if (p.norm())
				p /= p.norm();
			p *= radius;
			p += n.center;
		} while (!n.contains(p) && it < max_it); // Ensure that the domain condition is respected. 

		if (it == max_it) {
			throw sa::SamplerFailure("Iteration missed after " +
				std::to_string(max_it) + " attempts");
		}

		return p;
	}

	void seed(long seed) override {
		gen.seed(seed);
	}

};

// ---------------------------------------------------------
// DISCRETE
// ---------------------------------------------------------

template<domain_dim Dim>
class UniformSampler<DiamondDiscreteNeighbourhood<Dim>> : public  SamplerBase<DiamondDiscreteNeighbourhood<Dim>> {
	double k;
	std::random_device rd;
	std::mt19937 gen;
	
public:
	UniformSampler() :
		rd(), gen(rd()) {}

	using Neighbourhood = DiamondDiscreteNeighbourhood<Dim>;

	point_nd<Dim> sample(const DiamondDiscreteNeighbourhood<Dim>& n) override {

		point_nd<Dim> p = 0.0;
		std::uniform_int_distribution<int> nd{0, n.manhattan_dist};
		do {

			for (int i = 0; i < Dim; ++i) {
				p[i] = n.center[i] + (nd(gen));
			}

		} while (!n.contains(p));

		return p;

	}

	void seed(long seed) override {
		gen.seed(seed);
	}

};


template<domain_dim Dim>
class UniformSampler<CircleDiscreteNeighbourhood<Dim>> : public  SamplerBase<CircleDiscreteNeighbourhood<Dim>> {
	
	std::random_device rd;
	std::mt19937 gen;
	std::normal_distribution<double> nd;

public:
	UniformSampler()
		: rd(), gen(rd()), nd(0, 1.0) { }

	using Neighbourhood = CircleDiscreteNeighbourhood<Dim>;

	lattice_nd<Dim> sample(const CircleDiscreteNeighbourhood<Dim>& n, int max_it) override {

		lattice_nd<Dim> p = 0.0;
		std::uniform_int_distribution<> distrib(0, n.radius);

		int it = 0;

		do {

			it += 1;
			const double radius = distrib(gen);

			for (int i = 0; i < Dim; ++i) {
				p[i] = nd(gen);
			}
			// Only normalize if the sampled unitary point is nonzero... which should be everytime!
			if (p.l1_norm())
				p /= p.l1_norm();
			p *= radius;
			p += n.center;
		} while (!n.contains(p) && it < max_it); // Ensure that the domain condition is respected.

		if (it == max_it) {
			throw sa::SamplerFailure("Iteration missed after " +
				std::to_string(max_it) + " attempts");
		}

		return p;
	}

	void seed(long seed) override {
		gen.seed(seed);
	}

};

// ---------------------------------------------------------
// COMBINATORY
// ---------------------------------------------------------


template<domain_dim Dim>
class UniformSampler<CombinationNeighbourhood<Dim>> : public  SamplerBase<CombinationNeighbourhood<Dim>> {
	double k;
	std::random_device rd;
	std::mt19937 gen;
	std::uniform_int_distribution<int> value;
public:
	UniformSampler()
		: rd(), gen(rd()), value(0, 1) { }

	using Neighbourhood = CombinationNeighbourhood<Dim>;

	combination_nd<Dim> sample(const CombinationNeighbourhood<Dim>& n, int max_it) override {

		// Default initialize to 0
		combination_nd<Dim> p = n.center;
		int it = 0;
		std::uniform_int_distribution<> iterations(0, n.hamming_distance);
		std::uniform_int_distribution<> distrib(0, Dim-1);

		do {
			it += 1;
			const int its = iterations(gen);
			for (int i = 0; i < its; ++i) {
				p.flip(distrib(gen));
			}
			
		} while (!n.contains(p) && it < max_it); // Ensure that the domain condition is respected.

		if (it == max_it) {
			throw sa::SamplerFailure("Iteration missed after " +
				std::to_string(max_it) + " attempts");
		}

		return p;
	}

	void seed(long seed) override {
		gen.seed(seed);
	}

};


#endif