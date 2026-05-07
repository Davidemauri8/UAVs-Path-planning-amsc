#pragma once
#ifndef SAMPLER_BASE_HPP
#define SAMPLER_BASE_HPP

template <typename Neigh>
class SamplerBase {
public:
	using Neighbourhood = Neigh;

	virtual typename Neigh::PointType sample(const Neigh& n, int max_it) = 0;

	virtual void seed(long seed) = 0;

};

#endif

