#pragma once
#ifndef CONTINUOUS_TEST_FUNCTIONS
#define CONTINUOUS_TEST_FUNCTIONS

#include "../geometry/point.hpp"
#include <cmath>

typedef unsigned int domain_dim;

namespace TestFunctions {

	template <domain_dim Dim>
	std::function<double(const point_nd<Dim>&)> make_func_sum_of_different_powers() {
		// https://www.sfu.ca/~ssurjano/sumpow.html
		return [](const point_nd<Dim>& p) -> double {
			double s = 0.0;
			for (int i = 0; i < Dim; ++i) {
				double pow = 1.0;
				for (int k = 0; k < i + 2; ++k) pow *= p[i];
				s += pow;
			};
			return s;
		};
	}

	template <domain_dim Dim>
	std::function<double(const point_nd<Dim>&)> make_func_schwefel() {
		// https://www.sfu.ca/~ssurjano/schwef.html
		return [](const point_nd<Dim>& p) -> double {
			double s = 418.9829 * Dim;
			for (int i = 0; i < Dim; ++i) {
				s -= p[i] * std::sin(std::sqrt(std::abs(p[i])));
			};
			return s;
		};
	}

	double forrester(const double x) {
		// Note the use of implicit conversion to double for point_nd<1>!
		return (6 * x - 2) * (6 * x - 2) * std::sin(12 * x - 4);
	}

	double drop_wave_function(const point_nd<2>& x) {
		return -(1 + std::cos(12 * x.norm())) / (0.5 * x.norm_squared() + 2);
	}

	template <domain_dim Dim>
	std::function<double(const point_nd<Dim>&)> make_func_levy() {
	
		using namespace VectorMath;
		constexpr domain_dim Dimm1 = Dim - 1;
		constexpr double pi = 3.141592;

		return [](const point_nd<Dim>& x) -> double {
			const auto w = 1 + (x - 1) / 4.0;
			const auto w_r = w. template splice<0, Dim - 1>();
			const auto m = ((w_r - 1) ^ 2).elwise_prod(1 + 10 * (sin(pi * w_r + 1)) ^ 2).sum();
			m += std::sin(w[0] * pi) * std::sin(w[0] * pi);
			m += (w[Dim - 1] - 1) * (w[Dim - 1] - 1) * 
				(1 + std::sin(2 * pi * w[Dim - 1]) * std::sin(2 * pi * w[Dim - 1]));
			return m;
		};
	}


	double eggholder_function(const point_nd<2>& x) {
		const double sin1 = std::sin(std::sqrt(std::abs(x[1] + x[0]/2.0 + 47)));
		const double sin2 = std::sin(std::sqrt(std::abs(x[0] - x[0] - 47)));
		return -(x[1] + 47) * sin1 - x[0] +sin2;
	}
	
	template <domain_dim Dim>
	std::function<double(const point_nd<Dim>&)> make_func_griewank() {

		using namespace VectorMath;

		return [](const point_nd<Dim>& x) -> double {
			// x.norm_squared() / 4000.0 - 
			const auto int_range = point_nd<1>::int_range<1, Dim>();
			const double prod = cos(x.elwise_div(sqrt(int_range))).prod();
			
			return 1.0 + (x.norm_squared() / 4000.0) - prod;
		};
	}

	// Styblinski-Tang
	template <domain_dim Dim>
	std::function<double(const point_nd<Dim>&)> make_func_styblinski_tang() {

		return [](const point_nd<Dim>& x) -> double {
			return 0.5*(x ^ 4 - 16 * (x ^ 2) + 5* x).sum();
		};
	}
}

#endif