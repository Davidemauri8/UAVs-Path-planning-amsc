#pragma once
#ifndef LOCAL_SAMPLER_FIXED_HPP
#define LOCAL_SAMPLER_FIXED_HPP

#include "../neighbourhoods/neighbourhood_gen.hpp"
#include "exception.hpp"
#include "sampler_base.hpp"
#include <random>
#include <cmath>

// LocalSampler con distribuzione corretta:
//   radius ~ |N(0, sigma)|  dove sigma = k * n.radius
// std::normal_distribution prende la deviazione standard (non la varianza),
// quindi si passa sigma, non sigma*sigma.

template <typename Neigh>
class LocalSamplerFixed : public SamplerBase<Neigh> {
    double k;
public:
    using Neighbourhood = Neigh;
    LocalSamplerFixed(double n) { this->k = n; }
protected:
};

template<domain_dim Dim>
class LocalSamplerFixed<CircleNeighbourhood<Dim>>
    : public SamplerBase<CircleNeighbourhood<Dim>> {

    double k;
    std::random_device rd;
    std::mt19937 gen;
    std::normal_distribution<double> nd;

public:
    LocalSamplerFixed(double n) : rd(), gen(rd()), nd(0, 1.0) { this->k = n; }

    using Neighbourhood = CircleNeighbourhood<Dim>;

    point_nd<Dim> sample(const CircleNeighbourhood<Dim>& n, int max_it) override {

        double sigma = this->k * n.radius;
        std::normal_distribution<double> nd_r{ 0.0, sigma };  // stddev = sigma, varianza = sigma²

        point_nd<Dim> p = 0.0;
        double radius   = 0.0;
        int it = 0;

        do {
            it += 1;
            do {
                radius = nd_r(gen);
            } while (std::abs(radius) > n.radius);   // rigetta sia grandi positivi che negativi

            for (int i = 0; i < Dim; ++i)
                p[i] = nd(gen);

            if (p.norm())
                p /= p.norm();
            p *= radius;
            p += n.center;
        } while (!n.contains(p) && it < max_it);

        if (it == max_it)
            throw sa::SamplerFailure("Iteration missed after " +
                std::to_string(max_it) + " attempts");

        return p;
    }

    void seed(long seed) override { gen.seed(seed); }
};

template<domain_dim Dim>
class LocalSamplerFixed<SquareNeighbourhood<Dim>>
    : public SamplerBase<SquareNeighbourhood<Dim>> {

    double k;
    std::random_device rd;
    std::mt19937 gen;

public:
    LocalSamplerFixed(double n) : rd(), gen(rd()) { this->k = n; }

    using Neighbourhood = SquareNeighbourhood<Dim>;

    point_nd<Dim> sample(const SquareNeighbourhood<Dim>& n, int max_it) override {

        double sigma = this->k * n.radius;
        std::normal_distribution<double> nd{ 0.0, sigma };
        point_nd<Dim> p = 0.0;
        int it = 0;

        do {
            it += 1;
            for (int i = 0; i < Dim; ++i) {
                do {
                    p[i] = nd(gen);
                } while (std::abs(p[i]) > n.radius);
            }
        } while (!n.contains(p) && it < max_it);

        if (it == max_it)
            throw sa::SamplerFailure("Iteration missed after " +
                std::to_string(max_it) + " attempts");

        return p;
    }

    void seed(long seed) override { gen.seed(seed); }
};

#endif
