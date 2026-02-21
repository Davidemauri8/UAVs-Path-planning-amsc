#pragma once
#ifndef UNIFORM_SAMPLER 
#define UNIFORM_SAMPLER
#include "../neighbourhoods/neighbourhood_gen.hpp"

#include <random>

template <typename Neigh>
class SamplerBase {
public:
	using Neighbourhood = Neigh;

	virtual typename Neigh::PointType sample(const Neigh& n) = 0;
	//virtual typename Neigh::LatticeType sample(const Neigh& n) = 0;
	
};


template <typename Neigh>
class UniformSampler : public SamplerBase<Neigh> {
	using Neighbourhood = Neigh;
	
};

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

	point_nd<Dim> sample(const SquareNeighbourhood<Dim>& n) override {

		point_nd<Dim> p = 0.0;

		// a and b will be used to rescale the normal distribution
		double a = 0.0;
		double b = 0.0;

		for (int i = 0; i < Dim; ++i) {
			a = n.center[i] - n.radius;
			b = n.center[i] + n.radius;
			// rescaling the distribution
			p[i] = a + (b-a) * distrib(gen);

		}

		//return the point extracted
		return p;
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

	point_nd<Dim> sample(const CircleNeighbourhood<Dim>& n) override {

		point_nd<Dim> p = 0.0;
		std::uniform_real_distribution<> distrib(0, n.radius);
		do {
			const double radius = distrib(gen);

			for (int i = 0; i < Dim; ++i) {
				p[i] = nd(gen);
			}
			// Only normalize if the sampled unitary point is nonzero... which should be everytime!
			if (p.norm())
				p /= p.norm();
			p *= radius;
			p += n.center;
		} while (!n.contains(p)); // Ensure that the domain condition is respected. 
		return p;
	}
};


template <typename Neigh>
class LocalSampler  : public SamplerBase<Neigh> {
	double k;
public:
	using Neighbourhood = Neigh;
	LocalSampler(double n) {
		this->k = n;
	};
	virtual typename Neigh::PointType sample(const Neigh& n) = 0;

protected:
};


template<domain_dim Dim>
class LocalSampler<CircleNeighbourhood<Dim>> : public  SamplerBase<CircleNeighbourhood<Dim>> {
	double k;
	std::random_device rd;
	std::mt19937 gen;
	std::normal_distribution<double> nd;
public:

	LocalSampler(double n) : 
		rd(), gen(rd()), nd(0, 1.0)  { this->k = n; }

	using Neighbourhood = CircleNeighbourhood<Dim>;

	point_nd<Dim> sample(const CircleNeighbourhood<Dim>& n) override {

		// normal distribution to sample the radius

		double sigma = this->k * n.radius;
		std::normal_distribution nd_r{ 0.0, sigma * sigma };

		point_nd<Dim> p = 0.0;
		double radius = 0.0;

		do {
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
		} while (!n.contains(p));
		
		return p;

	}
};

template<domain_dim Dim>
class LocalSampler<SquareNeighbourhood<Dim>> : public  SamplerBase<SquareNeighbourhood<Dim>> {
	double k;
	std::random_device rd;
	std::mt19937 gen;
public:
	LocalSampler(double n) :
		rd(), gen(rd()) { this->k = n; }

	using Neighbourhood = SquareNeighbourhood<Dim>;

	point_nd<Dim> sample(const SquareNeighbourhood<Dim>& n) override {

		double sigma = this->k * n.radius;
		std::normal_distribution nd{ 0.0, sigma*sigma };
		point_nd<Dim> p = 0.0;

		do {
			for (int i = 0; i < Dim; ++i) {
				do {
					p[i] = (nd(gen));;
				} while (p[i] > n.radius);
			}
		} while (!n.contains(p));
		

		//return the point extracted
		return p;
		
	}
};


template <typename Neigh>
class NonLocalSampler : public SamplerBase<Neigh> {
public:
	double k;
	using Neighbourhood = Neigh;
	NonLocalSampler(double n) {};
	virtual typename Neigh::PointType sample(const Neigh& n) = 0;

protected:
};


template<domain_dim Dim>
class NonLocalSampler<CircleNeighbourhood<Dim>> : public  SamplerBase<CircleNeighbourhood<Dim>> {
	double k;
	std::random_device rd;
	std::mt19937 gen;
	std::uniform_real_distribution<double> nd_r;
	std::normal_distribution<double> nd;
public:

	NonLocalSampler(double n) :
		rd(), gen(rd()), nd_r(0, 1.0), nd(0, 1.0) {
		this->k = n;
	}

	using Neighbourhood = CircleNeighbourhood<Dim>;

	point_nd<Dim> sample(const CircleNeighbourhood<Dim>& n) override {

		point_nd<Dim> p = 0.0;
		double radius = 0.0;
		k = k + Dim; // k needs to be > n-1
		do {
			do {
				radius = n.radius * pow((nd_r(gen)), 1.0/(this->k +1.0));
			} while (radius > n.radius);

			for (int i = 0; i < Dim; ++i) {
				p[i] = (nd(gen));
			}

			if (p.norm())
				p /= p.norm();
			p *= radius;
			p += n.center;
		} while (!n.contains(p));

		return p;

	}
};

template<domain_dim Dim>
class NonLocalSampler<SquareNeighbourhood<Dim>> : public  SamplerBase<SquareNeighbourhood<Dim>> {
	double k;
	std::random_device rd;
	std::mt19937 gen;
public:
	NonLocalSampler(double n) :
		rd(), gen(rd()) { this->k = n; }

	using Neighbourhood = SquareNeighbourhood<Dim>;

	point_nd<Dim> sample(const SquareNeighbourhood<Dim>& n) override {

		point_nd<Dim> p = 0.0;
		
		return p;

	}
};


/*
template<domain_dim Dim>
class UniformSampler<SquareNeighbourhood<Dim>> : public  SamplerBase<SquareNeighbourhood<Dim>> {
	double k;
	std::random_device rd;
	std::mt19937 gen;
	
public:
	UniformDiscreteSampler() :
		rd(), gen(rd()) {}

	using Neighbourhood = SquareNeighbourhood<Dim>;

	point_nd<Dim> sample(const SquareNeighbourhood<Dim>& n) override {

		point_nd<Dim> p = 0.0;
		std::uniform_int_distribution<double> nd{-n.radius, n.radius};
		do {

			for (int i = 0; i < Dim; ++i) {
				p[i] = n.center[i] + (nd(gen));
			}

		} while (!n.contains(p));

		return p;

	}
};
*/


template<domain_dim Dim>
class UniformSampler<CircleDiscreteNeighbourhood<Dim>> : public  SamplerBase<CircleDiscreteNeighbourhood<Dim>> {
	double k;
	std::random_device rd;
	std::mt19937 gen;
	std::normal_distribution<double> nd;

public:
	UniformSampler()
		: rd(), gen(rd()), nd(0, 1.0) { }

	using Neighbourhood = CircleDiscreteNeighbourhood<Dim>;

	lattice_nd<Dim> sample(const CircleDiscreteNeighbourhood<Dim>& n) override {

		lattice_nd<Dim> p = 0.0;
		std::uniform_int_distribution<> distrib(0, n.radius);
		do {
			const double radius = distrib(gen);

			for (int i = 0; i < Dim; ++i) {
				p[i] = nd(gen);
			}
			// Only normalize if the sampled unitary point is nonzero... which should be everytime!
			if (p.l1_norm())
				p /= p.l1_norm();
			p *= radius;
			p += n.center;
		} while (!n.contains(p)); // Ensure that the domain condition is respected.
		return p;
	}
};



#endif