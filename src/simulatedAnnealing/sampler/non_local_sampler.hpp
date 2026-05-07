
#pragma once
#ifndef NON_LOCAL_SAMPLER 
#define NON_LOCAL_SAMPLER
#include "../neighbourhoods/neighbourhood_gen.hpp"
#include "exception.hpp"
#include "sampler_base.hpp"

#define THRESHOLD 0.5

#include <random>

// Sampler that samples with lower probability the values near the centre of the neighbourhood


template <typename Neigh>
class NonLocalSampler : public SamplerBase<Neigh> {
public:
	using Neighbourhood = Neigh;
	
	// k is a parameter that controls the degree of locality
	double k;
	NonLocalSampler(double n) { };
protected:
};

// ---------------------------------------------------------
// CONTINUOUS
// ---------------------------------------------------------

template<domain_dim Dim>
class NonLocalSampler<CircleNeighbourhood<Dim>> : public  SamplerBase<CircleNeighbourhood<Dim>> {
	double k;
	std::random_device rd;
	std::mt19937 gen;
	std::normal_distribution<double> nd;
public:

	NonLocalSampler(double n) :
		rd(), gen(rd()), nd(0, 1.0) {
		this->k = n;
	}

	using Neighbourhood = CircleNeighbourhood<Dim>;

	point_nd<Dim> sample(const CircleNeighbourhood<Dim>& n, int max_it) override {

		std::gamma_distribution<> d(2, n.radius*0.1);
		point_nd<Dim> p = 0.0;
		double radius = 0.0;

		int it = 0;

		do {

			it += 1;

			do {
				radius = d(gen);

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
class NonLocalSampler<SquareNeighbourhood<Dim>> : public  SamplerBase<SquareNeighbourhood<Dim>> {
	double k;
	std::random_device rd;
	std::mt19937 gen;
public:
	NonLocalSampler(double n) :
		rd(), gen(rd()) {
		this->k = n;
	}

	using Neighbourhood = SquareNeighbourhood<Dim>;

	point_nd<Dim> sample(const SquareNeighbourhood<Dim>& n, int max_it) override {

		point_nd<Dim> p = 0.0;
		// TODO: Implement this square sampler!
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
class NonLocalSampler<CircleDiscreteNeighbourhood<Dim>> : public  SamplerBase<CircleDiscreteNeighbourhood<Dim>> {

	double probability;
	std::random_device rd;
	std::mt19937 gen;
	std::normal_distribution<double> nd;

public:
	NonLocalSampler(double prob)
		: rd(), gen(rd()), nd(0, 1.0), probability(prob) { }

	using Neighbourhood = CircleDiscreteNeighbourhood<Dim>;

	lattice_nd<Dim> sample(const CircleDiscreteNeighbourhood<Dim>& n, int max_it) override {

		//if (probability > THRESHOLD) {
		//	throw sa::NonLocalWrongUse();
		//}

		lattice_nd<Dim> p = 0.0;
		std::geometric_distribution<> geom(probability);


		int it = 0;


		do {

			it += 1;
			
			int g = geom(gen);
			int radius = n.radius - g;
			radius = std::max(0, radius);
			//int radius = geom(gen);

			for (int i = 0; i < Dim; ++i) {
				p[i] = nd(gen);
			}
			// Only normalize if the sampled unitary point is nonzero... 
			// which should be everytime!
			if (p.l1_norm())
				p /= p.l1_norm();
			else
				p += 1;
			p *= radius;
			for (int i = 0; i < Dim; ++i) {
				p[i] += n.center[i];
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


template<domain_dim Dim>
class NonLocalSampler<DiamondDiscreteNeighbourhood<Dim>> : public  SamplerBase<DiamondDiscreteNeighbourhood<Dim>> {

	double probability;
	std::random_device rd;
	std::mt19937 gen;

public:
	NonLocalSampler(double prob) :
		rd(), gen(rd()), probability(prob) { }

	using Neighbourhood = DiamondDiscreteNeighbourhood<Dim>;

	lattice_nd<Dim> sample(const DiamondDiscreteNeighbourhood<Dim>& n, int max_it) override {
		
		/*if (probability > THRESHOLD)
			// When a probability that goes against the notion of locality of the 
			// sampling is used, we consider this a bug. 
			throw sa::NonLocalWrongUse();*/

		lattice_nd<Dim> p = 0.0;
		std::geometric_distribution<> geom(probability); //to sample with more probability near the initial "point"

		int it = 0;
		do {
			for (int i = 0; i < Dim; ++i) {
				int to_add = 0;
				do {
					// Although its a bit impractical...
					to_add = n.manhattan_dist-geom(gen);
					to_add = std::max(0, to_add);
				} while (to_add > n.manhattan_dist);
				p[i] = n.center[i] + to_add;
			}
			it++;
		} while (!n.contains(p) && it < max_it);

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
class NonLocalSampler<CombinationNeighbourhood<Dim>> : public  SamplerBase<CombinationNeighbourhood<Dim>> {
	double k;
	std::random_device rd;
	std::mt19937 gen;
	std::uniform_int_distribution<int> value;
public:

	NonLocalSampler(double prob)
		: k(prob), rd(), gen(rd()), value(0, 1) { }

	using Neighbourhood = CombinationNeighbourhood<Dim>;

	combination_nd<Dim> sample(const CombinationNeighbourhood<Dim>& n, int max_it) override {

		//if (k > THRESHOLD) {
		//	throw sa::NonLocalWrongUse();
		//}

		combination_nd<Dim> p = 0.0;
		int it = 0;
		std::geometric_distribution<> iterations(k); //to sample with more probability far from the initial "point"
		std::uniform_int_distribution<> distrib(0, Dim - 1);

		do {
			it += 1;
			const int its = Dim - iterations(gen);
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