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

	virtual void from(PtType prev, PtType curr) = 0; // sets the center of the neighbourhood based on previous and current points

	virtual bool contains(PtType p) const = 0; 

};

template<domain_dim Dim>                                                                       
class SquareNeighbourhood : public NeighbourhoodExtractor<point_nd<Dim>> {

public:	
	SquareNeighbourhood(

		std::shared_ptr<point_nd<Dim>> _center,
		
		double _radius
	) 
		: center(*_center), radius(_radius) //Riceve un shared_ptr<lattice_nd<Dim>> _center e copia il valore in center (cioè center = *_center).

	{
	}
	
	point_nd<Dim> center;
	double radius;


	void from(point_nd <Dim> prev, point_nd<Dim> curr) override {

		for (domain_dim i = 0; i < Dim; ++i) { 
		
			center.raw_data[i] = static_cast<int>(std::round(curr.raw_data[i])); // Arrotonda il valore di curr.raw_data[i] e lo assegna a center.raw_data[i].
		
		}

	
		return;
	}

	bool contains(point_nd<Dim> p) const override {

		for(domain_dim i = 0; i < Dim; ++i) {

			if (std::abs(p.raw_data[i] - center.raw_data[i]) > radius) // Controlla se la distanza assoluta tra p.raw_data[i] e center.raw_data[i] è maggiore di radius.
			
				return false;  // se maggiore rifiutiamo il punto 
		}

		return true; //se passa il ciclo if accettiamo 
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
	{
	}

	// definiamo centro e raggio
	point_nd<Dim> center;
	double radius;
	std::shared_ptr<RnDomain<Dim>> domain;

	void from(point_nd <Dim> prev, point_nd<Dim> curr) override {

		center = curr; // assegna il valore di curr a center
		return;
	
	}

	bool contains(point_nd<Dim> p) const override {

		double dist_sq = 0.0; // distanza euclidea

		for (domain_dim i = 0; i < Dim; ++i) {

			double diff = p.raw_data[i] - center.raw_data[i]; // calcola la differenza tra le coordinate del punto p e del centro
			dist_sq += diff * diff; // somma il quadrato delle differenze

		}

		if (dist_sq > radius * radius) // confronta la distanza al quadrato con il raggio al quadrato
			return false; 

		return domain->contains(&p); //verifica se il punot è dentro il dominio 
	}

 

};


template<domain_dim Dim>
class CircleDiscreteNeighbourhood : public NeighbourhoodExtractor<lattice_nd<Dim>> {


public:
	CircleDiscreteNeighbourhood(
		std::shared_ptr<NnDomain<Dim>> _dom,
		double _radius
	)
		: domain(_dom), radius(_radius), center(0.0)
	{
	}

	// definiamo centro e raggio
	lattice_nd<Dim> center;
	double radius;
	std::shared_ptr<NnDomain<Dim>> domain;

	void from(lattice_nd <Dim> prev, lattice_nd<Dim> curr) override {
		center = curr; // assegna il valore di curr a center
		return;
	}

	bool contains(lattice_nd<Dim> p) const override {

		double dist_sq = Distance::manhattan_distance(center, p);

		return (dist_sq > radius); 
			 
	}



};

#endif // ! NEIGHBOURHOOD_GEN