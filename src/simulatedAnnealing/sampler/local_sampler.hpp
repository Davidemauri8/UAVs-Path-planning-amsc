#pragma once
#ifndef LOCAL_SAMPLER 
#define LOCAL_SAMPLER
#include "../neighbourhoods/neighbourhood_gen.hpp"
#include "exception.hpp"
#include "sampler_base.hpp"
#include <random>

#define THRESHOLD 0.5

// Sampler that samples with higher probability the values near the centre of the neighbourhood

template <typename Neigh>
class LocalSampler : public SamplerBase<Neigh> {
	double k;
public:
	using Neighbourhood = Neigh;

	LocalSampler(double n) {
		this->k = n;
	};

protected:
};

// ---------------------------------------------------------
// CONTINUOUS
// ---------------------------------------------------------

template<domain_dim Dim>
class LocalSampler<CircleNeighbourhood<Dim>> : public  SamplerBase<CircleNeighbourhood<Dim>> {
	
	double k; 
	std::random_device rd;
	std::mt19937 gen;
	std::normal_distribution<double> nd;
	int max_iterations;

public:

	LocalSampler(double n) : 
		rd(), gen(rd()), nd(0, 1.0) {
		this->k = n;
	}

	using Neighbourhood = CircleNeighbourhood<Dim>;

	point_nd<Dim> sample(const CircleNeighbourhood<Dim>& n , int max_it) override {

		// normal distribution to sample the radius

		double sigma = this->k * n.radius;
		std::normal_distribution nd_r{ 0.0, sigma * sigma };

		point_nd<Dim> p = 0.0;
		double radius = 0.0;

		int it = 0;

		do {
			it += 1;
			do {
				radius = (nd_r(gen));
			} while (radius > n.radius);

			for (int i = 0; i < Dim; ++i) {
				p[i] = (nd(gen));
			}

			if (p.norm())
				p /= p.norm();
			p *= radius;
			p += n.center;
		} while (!n.contains(p) && it < max_it);

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

template<domain_dim Dim>
class LocalSampler<SquareNeighbourhood<Dim>> : public  SamplerBase<SquareNeighbourhood<Dim>> {
	double k;
	std::random_device rd;
	std::mt19937 gen;

public:
	LocalSampler(double n) :
		rd(), gen(rd()) {
		this->k = n;
	}

	using Neighbourhood = SquareNeighbourhood<Dim>;

	point_nd<Dim> sample(const SquareNeighbourhood<Dim>& n, int max_it) override {

		double sigma = this->k * n.radius;
		std::normal_distribution nd{ 0.0, sigma * sigma };
		point_nd<Dim> p = 0.0;
		int it = 0;

		do {
			it += 1;
			for (int i = 0; i < Dim; ++i) {
				do {
					p[i] = (nd(gen));
				} while (p[i] > n.radius);
			}
		} while (!n.contains(p) && it < max_it);

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
class LocalSampler<CircleDiscreteNeighbourhood<Dim>> : public  SamplerBase<CircleDiscreteNeighbourhood<Dim>> {

	double probability;
	std::random_device rd;
	std::mt19937 gen;
	std::normal_distribution<double> nd;

public:
	LocalSampler(double k)
		: probability(k), rd(), gen(rd()), nd(0, 1.0) { }

	using Neighbourhood = CircleDiscreteNeighbourhood<Dim>;

	lattice_nd<Dim> sample(const CircleDiscreteNeighbourhood<Dim>& n, int max_it) override {

		if (probability < THRESHOLD) {
			throw sa::LocalWrongUse();
		}

		lattice_nd<Dim> p = 0.0;
		std::geometric_distribution<> geom(probability);

		int it = 0;

		do {

			it += 1;
			const double radius = geom(gen);

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



template<domain_dim Dim>
class LocalSampler<DiamondDiscreteNeighbourhood<Dim>> : public  SamplerBase<DiamondDiscreteNeighbourhood<Dim>> {
	double probability;
	std::random_device rd;
	std::mt19937 gen;

	public:
		LocalSampler(double prob) :
			rd(), gen(rd()), probability(prob) {}

		using Neighbourhood = DiamondDiscreteNeighbourhood<Dim>;

		lattice_nd<Dim> sample(const DiamondDiscreteNeighbourhood<Dim>& n) override {

		if (probability < THRESHOLD) {
			throw sa::LocalWrongUse();
		}

		lattice_nd<Dim> p = 0.0;
		std::geometric_distribution<> geom(probability); //to sample with more probability near the initial "point"
		int to_add = 0;
		do {

			for (int i = 0; i < Dim; ++i) {
				do {
					to_add = geom(gen);
					p[i] = n.center[i] + to_add;
				} while (to_add > n.manhattan_dist );
			}

		} while (!n.contains(p));

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
class LocalSampler<CombinationNeighbourhood<Dim>> : public  SamplerBase<CombinationNeighbourhood<Dim>> {
	double probability;
	std::random_device rd;
	std::mt19937 gen;
	std::uniform_int_distribution<int> value;
public:
	LocalSampler(double prob)
		: probability(prob), rd(), gen(rd()), value(0, 1) { }

	using Neighbourhood = CombinationNeighbourhood<Dim>;

	combination_nd<Dim> sample(const CombinationNeighbourhood<Dim>& n, int max_it) override {

		if (probability < THRESHOLD) {
			throw sa::LocalWrongUse();
		}

		combination_nd<Dim> p = 0.0;
		int it = 0;
		std::geometric_distribution<> iterations(probability); //to sample with more probability near the initial "point"
		std::uniform_int_distribution<> distrib(0, Dim - 1);

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