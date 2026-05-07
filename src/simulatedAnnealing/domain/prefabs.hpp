#pragma once
#ifndef PREFABS_HPP
#define PREFABS_HPP
#include "domain.hpp"

class ContinuousDomainRepository {

public:

	// A unitary circle in 2 dimensions
	static const std::shared_ptr<RnDomain<2>> unitary_2d_circle;
	// A unitary sphere in 3 dimensions
	static const std::shared_ptr<RnDomain<3>> unitary_3d_sphere;
	// A unitary cube in 3 dimensions
	static const std::shared_ptr<RnDomain<3>> unitary_3d_cube;

};

// Static initialization of the domains in the repository;
const std::shared_ptr<RnDomain<2>> ContinuousDomainRepository::unitary_2d_circle =
	RnDomainFactory::make_sphere(1.0, point_nd<2>(0.0));
const std::shared_ptr<RnDomain<3>> ContinuousDomainRepository::unitary_3d_sphere = 
	RnDomainFactory::make_sphere(1.0, point_nd<3>(0.0));
const std::shared_ptr<RnDomain<3>> ContinuousDomainRepository::unitary_3d_cube = 
	RnDomainFactory::make_hypercube(1.0, point_nd<3>(0.0));

class DiscreteDomainRepository {


};

#endif