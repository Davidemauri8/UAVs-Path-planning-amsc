
#pragma once
#ifndef _DOMAIN_DOMAIN
#define _DOMAIN_DOMAIN

#include <memory>
#include <stdexcept>
#include <vector>
#include <array>
#include <functional>
#include <map>

#include "../geometry/lattice_point.hpp"
#include "../geometry/combination.hpp"
#include "../geometry/point.hpp"


typedef unsigned int domain_dim;

template <domain_dim Dim = 2u>
class Domain {
public:

	static constexpr auto Dimension = Dim;

	// Note: We have to use a void* to overcome the difficulty
	// of having templetized virtual functions (e.g. we reinterpret
	// the pointer as being one of lattice or points depending on what happens
	virtual bool contains(const void* point) const = 0;

protected:

	virtual void output_print(std::ostream& os) const = 0;

	// Friend function to allow output of domains
	template <domain_dim FriendDim>
	friend std::ostream& operator<<(std::ostream& os, const Domain<FriendDim>& d);

};

template <domain_dim Dim>
std::ostream& operator<<(std::ostream& os, const Domain<Dim>& d) {
	d.output_print(os);
	return os;
}

template <domain_dim Dim>
class RnDomain : public Domain<Dim> {

};

template <domain_dim Dim>
class NnDomain : public Domain<Dim> {

};

template <domain_dim Dim>
class DiscreteGrid : public NnDomain<Dim> {

public:

	std::array<long, Dim> dimensions;
	const lattice_nd<Dim> reference;

	DiscreteGrid(
		std::initializer_list<long> init_dims, lattice_nd<Dim> ref
	) : reference(ref) {
		if (init_dims.size() != Dim)
			throw std::invalid_argument("Bad number of arguments");
		auto it = init_dims.begin();
		for (int i = 0; i < Dim; ++i, ++it) {
			if (*it <= 0)
				throw std::invalid_argument("Bad size argument");
			dimensions[i] = *it;
		}
	}

	bool contains(const void* p) const override {
		// TODO: warning
		const auto po = reinterpret_cast<const lattice_nd<Dim>*>(p);
		bool is_contained = 1;
		for (int i = 0; i < Dim; ++i)
			is_contained &= (reference[i] < (*po)[i] && (*po)[i] < reference[i] + dimensions[i]);
		return is_contained;
	}

	virtual void output_print(std::ostream& os) const {
		os << "Discrete grid of dimensions [";
		for (int i = 0; i < Dim - 1; ++i)
			os << dimensions[i] << ", ";
		os << dimensions[Dim - 1] << "] centered at " << reference;
	}

};

template <domain_dim Dim>
class BinaryString : public NnDomain<Dim> {

public:

	typedef std::function<bool(const combination_nd<Dim>&)> allowed_predicate;

	BinaryString() = delete;

	BinaryString(const allowed_predicate& allow)
		: allowed(allow) { }

	
	virtual bool contains(const void* p) const {
		// Notice: assuming the correct dimension is issued for the point p,
		// every binary string is contained in the range unless it is not allowed
		// by the specified rule (viz. illegal combinations)
		const auto po = reinterpret_cast<const combination_nd<Dim>*>(p);
		return allowed(*po);
	}

protected:

	allowed_predicate allowed;

	virtual void output_print(std::ostream& os) const {
		os << "Binary String of size " << Dim;
	}

};

template <domain_dim Dim>
class SphereDomainRn : public RnDomain< Dim> {

public:

	SphereDomainRn(const double radius, const point_nd<Dim>& p) :
		rad(radius), centre(p)
	{ }

	bool contains(const void* p) const override {
		// TODO: warning
		const auto po = reinterpret_cast<const point_nd<Dim>*>(p);
		return (*po - centre).norm() < rad;
	}

	void output_print(std::ostream& os) const override {
		os << "Sphere of radius " << rad << " centered in " << centre;
	}

	double get_radius() {
		return rad;
	}

protected:

	const point_nd<Dim> centre;
	const double rad;

};

template <domain_dim Dim>
class RectangleDomain : public RnDomain<Dim> {

protected:

	std::array<double, Dim> sides;
	point_nd<Dim> reference;

public:

	RectangleDomain() = delete;

	RectangleDomain( // Initializer constructor
		std::initializer_list<double> init_sides, point_nd<Dim> ref
	) : reference(ref) {
		if (init_sides.size() != Dim)
			throw std::invalid_argument("Bad number of arguments");
		auto it = init_sides.begin();
		for (int i = 0; i < Dim; ++i, ++it) {
			if (*it <= 0)
				throw std::invalid_argument("Bad size argument");
			sides[i] = *it;
		}
	}

	RectangleDomain(const point_nd<Dim>& init_sides, point_nd<Dim> ref)
		: reference(ref) {
		for (int i = 0; i < Dim; ++i)
			sides[i] = init_sides[i];
	}

	void output_print(std::ostream& os) const override {
		os << "Hyperrectangle of sides [";
		for (int i = 0; i < Dim - 1; ++i)
			os << sides[i] << ", ";
		os << sides[Dim - 1] << "] centered at " << reference;
	}

	bool contains(const void* p) const override {
		// TODO: warning
		const auto po = reinterpret_cast<const point_nd<Dim>*>(p);
		bool is_contained = 1;
		for (int i = 0; i < Dim; ++i)
			is_contained &= (reference[i] < (*po)[i] && (*po)[i] < reference[i] + sides[i]);
		return is_contained;
	}

};

template <domain_dim Dim>
class CubicDomain : public RnDomain<Dim> {

public:

	const double side;
	const point_nd<Dim> reference;

public:

	CubicDomain() = delete;

	CubicDomain( // Initializer constructor
		double _side, point_nd<Dim> ref
	) : side(_side), reference(ref) {
		if (side < 0)
			throw std::invalid_argument("Bad argument size");
	}

	void output_print(std::ostream& os) const override {
		os << "Hypercube of side " << side << " centered at " << reference;
	}

	bool contains(const void* p) const override {
		// TODO: warning
		const auto po = reinterpret_cast<const point_nd<Dim>*>(p);
		bool is_contained = 1;
		for (int i = 0; i < Dim; ++i)
			is_contained &= (reference[i] < (*po)[i] && (*po)[i] < reference[i] + side);
		return is_contained;
	}

};


template <domain_dim Dim>
class SimplexDomain : public RnDomain<Dim> {

public:

	std::vector<std::vector<double>> equations;

public:

	SimplexDomain() = delete;

	SimplexDomain( // Initializer constructor
		std::vector<std::vector<double>> s
	) : equations(s) {
		if (s.size() != Dim)
			throw std::invalid_argument("Bad argument size");
	}

	void output_print(std::ostream& os) const override {
		os << "Simplex of equations: [ ";
		for (int i = 0; i < Dim; ++i) {
			os << "[";
			for (int j = 0; j < Dim; ++j) os << equations[i][j] << ", ";
			os << equations[i][Dim - 1] << "]";
		}
		os << "]";
	}

	bool contains(const void* p) const override {
		// TODO: warning
		const auto po = reinterpret_cast<const point_nd<Dim>*>(p);
		bool is_contained = 1;
		double d;
		for (int i = 0; i < Dim; ++i) {
			d = 0;
			for (int j = 0; j < Dim; ++j)
				d += (*po)[j] * equations[i][j];
			is_contained &= (d <= 0);
		}
		return is_contained;
	}
};


class RnDomainFactory {

public:

	template <domain_dim Dim>
	static std::shared_ptr<RnDomain<Dim>> make_sphere(double radius, point_nd<Dim> centre) {
		return std::make_shared<SphereDomainRn<Dim>>(radius, centre);
	}

	template <domain_dim Dim>
	static std::shared_ptr<RnDomain<Dim>> make_hyperrectangle(
		std::initializer_list<double> list, point_nd<Dim> reference
	) {
		return std::make_shared<RectangleDomain<Dim>>(list, reference);
	}

	template <domain_dim Dim>
	static std::shared_ptr<RnDomain<Dim>> make_hyperrectangle(
		const point_nd<Dim>& sides, point_nd<Dim> reference
	) {
		return std::make_shared<RectangleDomain<Dim>>(sides, reference);
	}

	template <domain_dim Dim>
	static std::shared_ptr<RnDomain<Dim>> make_hypercube(double side, point_nd<Dim> reference) {
		return std::make_shared<CubicDomain<Dim>>(side, reference);
	}

	template <domain_dim Dim>
	static std::shared_ptr<RnDomain<Dim>> make_simplex(std::vector<std::vector<double>> s) {
		return std::make_shared<SimplexDomain<Dim>>(s);
	}


};

class NnDomainFactory {

public:

	template <domain_dim Dim>
	static std::shared_ptr<NnDomain<Dim>> make_binary_combination(
		typename BinaryString<Dim>::allowed_predicate allowed
	) { // allowed_predicate
		return std::make_shared<BinaryString<Dim>>(allowed);
	}

	template <domain_dim Dim>
	static std::shared_ptr<NnDomain<Dim>> make_lattice_grid(
		std::initializer_list<long> list, lattice_nd<Dim> reference) {
		return std::make_shared<DiscreteGrid<Dim>>(list, reference);
	}

};

template <domain_dim Dim, typename Dom_Type>
class DomainUnion : public RnDomain<Dim> {

protected:

	std::shared_ptr<Dom_Type> d1p;
	std::shared_ptr<Dom_Type> d2p;

public:

	// NOTE: 
	DomainUnion(
		const std::shared_ptr<Dom_Type>& d1, const std::shared_ptr<Dom_Type>& d2
	) : d1p(d1), d2p(d2) {
		static_assert(std::is_base_of<Domain<Dim>, Dom_Type>::value);
	}

	bool contains(const void* point) const override {
		return d1p->contains(point) || d2p->contains(point);
	}

protected:

	void output_print(std::ostream& os) const override { os << "Union of " << *d1p << " And " << *d2p; }

	template <typename Other_Dom>
	friend auto make_domain_union(const std::shared_ptr<Other_Dom>& d1, const std::shared_ptr<Other_Dom>& d2);

};

template <typename Dom>
std::shared_ptr<RnDomain<Dom::Dimension>> make_domain_union(
	const std::shared_ptr<Dom>& d1,
	const std::shared_ptr<Dom>& d2
) {
	static constexpr unsigned int Dim = Dom::Dimension;
	return std::make_shared<DomainUnion<Dim, Dom>>(d1, d2);
}

template <typename DomainType>
struct DomainCompositor {

	std::map<long, std::shared_ptr<DomainType>> domain_id_mapping;

	void join(long id, const std::vector<DomainType>& domains) {
		for (const auto& d : domains)
			this->join(id, d);
	}

	void join(long id, const std::shared_ptr<DomainType> sp) {

		domain_id_mapping.insert(std::pair(id, sp));
	}

	std::shared_ptr<DomainType> get(long id) {
		return domain_id_mapping[id];
	}

};

namespace Decomposition {


}

#endif