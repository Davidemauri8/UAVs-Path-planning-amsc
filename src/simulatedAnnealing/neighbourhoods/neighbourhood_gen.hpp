#pragma once
#ifndef NEIGHBOURHOOD_GEN
#define NEIGHBOURHOOD_GEN

#include "../geometry/point.hpp"
#include "../geometry/lattice_point.hpp"
#include "../geometry/measure.hpp"

template <typename PtType>
class NeighbourhoodExtractor {
public:
	using PointType = PtType;

	virtual void from(PtType prev, PtType curr) = 0; 
	// sets the center of the neighbourhood based on previous and current points

	virtual bool contains(PtType p) const = 0; 

};

template<domain_dim Dim>                                                                       
class SquareNeighbourhood : public NeighbourhoodExtractor<point_nd<Dim>> {

public:	
	SquareNeighbourhood(
		std::shared_ptr<point_nd<Dim>> _center,
		std::shared_ptr<RnDomain<Dim>> _dom,
		double _radius
	) 
		: center(*_center), domain(_dom), radius(_radius) 
		//Riceve un shared_ptr<lattice_nd<Dim>> _center e copia il valore in center (cioè center = *_center).
	{ }
	
	point_nd<Dim> center;
	double radius;
	std::shared_ptr<RnDomain<Dim>> domain;


	void from(point_nd <Dim> prev, point_nd<Dim> curr) override {
		center = curr;
		return;
	}

	bool contains(point_nd<Dim> p) const override {
		
		for (domain_dim i = 0; i < Dim; ++i) {
			if (std::abs(p.raw_data[i] - center.raw_data[i]) > radius) // Controlla se la distanza assoluta tra p.raw_data[i] e center.raw_data[i] è maggiore di radius.
				return false;  // se maggiore rifiutiamo il punto 
		}
		return domain->contains(&p);
		
	}

};

template<domain_dim Dim>
class CircleNeighbourhood : public NeighbourhoodExtractor<point_nd<Dim>> {

	
public:
	CircleNeighbourhood(
		std::shared_ptr<RnDomain<Dim>> _dom,
		double _radius
	)
		: domain(_dom), radius(_radius), center(0.0)
	{ }

	// definiamo centro e raggio
	point_nd<Dim> center;
	double radius;
	std::shared_ptr<RnDomain<Dim>> domain;

	void from(point_nd <Dim> prev, point_nd<Dim> curr) override {
		center = curr; // assegna il valore di curr a center
		return;
	}

	bool contains(point_nd<Dim> p) const override {
		return (p-center).norm() <= radius && domain->contains(&p); //verifica se il punot è dentro il dominio 
	}

 

};


template<domain_dim Dim>
class CircleDiscreteNeighbourhood : public NeighbourhoodExtractor<lattice_nd<Dim>> {

	std::shared_ptr<NnDomain<Dim>> domain;

public:

	CircleDiscreteNeighbourhood(
		std::shared_ptr<NnDomain<Dim>> _dom,
		double _radius
	): domain(_dom), radius(_radius), center(0.0)
	{ }

	// definiamo centro e raggio
	lattice_nd<Dim> center;
	double radius;

	void from(lattice_nd <Dim> prev, lattice_nd<Dim> curr) override {
		center = curr; // assegna il valore di curr a center
		return;
	}

	bool contains(lattice_nd<Dim> p) const override {
		double dist_sq = Distance::euclidean_distance(center, p);
		return (dist_sq <= radius && domain->contains(&p)); 
	}

};


template<domain_dim Dim>
class DiamondDiscreteNeighbourhood : public NeighbourhoodExtractor<lattice_nd<Dim>> {
	
	std::shared_ptr<NnDomain<Dim>> domain;

public:
	DiamondDiscreteNeighbourhood(
		std::shared_ptr<NnDomain<Dim>> _dom,
		double _manhattan_dist
	)
		: domain(_dom), manhattan_dist(_manhattan_dist), center(0.0)
	{  }


	// definiamo centro e raggio
	lattice_nd<Dim> center;
	double manhattan_dist;


	void from(lattice_nd <Dim> prev, lattice_nd<Dim> curr) override {
		center = curr; // assegna il valore di curr a center
		return;
	}

	bool contains(lattice_nd<Dim> p) const override {
		double dist_sq = Distance::manhattan_distance(center, p);
		return (dist_sq <= manhattan_dist && domain->contains(&p));
	}

};

template<domain_dim Dim>
class CombinationNeighbourhood : public NeighbourhoodExtractor<combination_nd<Dim>> {

	std::shared_ptr<NnDomain<Dim>> domain;

public:

	CombinationNeighbourhood(
		std::shared_ptr<NnDomain<Dim>> _dom,
		double _hd
	): domain(_dom), hamming_distance(_hd)
	{ }

	// definiamo centro e raggio
	combination_nd<Dim> center;
	double hamming_distance;

	void from(combination_nd <Dim> prev, combination_nd<Dim> curr) override {
		center = curr; // assegna il valore di curr a center
		return;
	}

	bool contains(combination_nd<Dim> p) const override {
		double dist_sq = Distance::hamming_distance(center, p);
		return (dist_sq <= hamming_distance && domain->contains(&p));
	}

};

#endif // ! NEIGHBOURHOOD_GEN