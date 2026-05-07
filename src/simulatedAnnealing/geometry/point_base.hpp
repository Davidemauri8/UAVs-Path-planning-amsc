#pragma once
#ifndef POINT_BASE_HPP
#define POINT_BASE_HPP

typedef unsigned int domain_dim;

template <domain_dim PointDim>
class PointBase {
public:

	// Necessary to allow point-templetized functions to access the point
	// dimension through PointType::Dim
	static constexpr const domain_dim Dim = PointDim;

};
#endif