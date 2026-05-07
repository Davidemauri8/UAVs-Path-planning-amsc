#ifndef DISCRETE_TEST_FUNCTIONS
#define DISCRETE_TEST_FUNCTIONS

#include "../geometry/lattice_point.hpp"

typedef unsigned int domain_dim;

namespace TestFunctions {

	/*
	template <domain_dim Dim>
	std::function<int(const lattice_nd<Dim>&)> make_func_sum_of_different_powers_discrete() {
		// https://www.sfu.ca/~ssurjano/sumpow.html
		return [](const lattice_nd<Dim>& p) -> int {
			int s = 0;
			for (int i = 0; i < Dim; ++i) {
				int pow = 1;
				for (int k = 0; k < i + 2; ++k) pow *= p[i];
				if (pow >= 0) s += pow;
				else s -= pow;
			};
			return s;
		};
	}
	
	*/

	// The glob min are p = (-1,-1,..,-1) and p = (2, 2,...,2) with f(p) = 0
	int quintic_discrete(const lattice_nd<1>& p) {
		int s = 0;
		for (int i = 0; i < 1; ++i) {
			int pow_5 = p[i] * p[i] * p[i] * p[i] * p[i];
			int pow_4 = p[i] * p[i] * p[i] * p[i];
			int pow_3 = p[i] * p[i] * p[i];
			int pow_2 = p[i] * p[i];

			int sum = (pow_5 - 3 * pow_4 + 4 * pow_3 + 2 * pow_2 - 10 * p[i] - 4);
			if (sum >= 0) s += sum;
			else s -= sum;
		}
		return s;

	}

	template <domain_dim Dim>
	std::function<int(const lattice_nd<Dim>&)> make_func_quintic_discrete() {
	// https://en.wikipedia.org/wiki/Quintic_function
		return [](const lattice_nd<Dim>& p) -> int {
			int s = 0;
			for (int i = 0; i < Dim; ++i) {
				int pow_5 = p[i]*p[i]*p[i]*p[i]*p[i];
				int pow_4 = p[i]*p[i]*p[i]*p[i];
				int pow_3 = p[i]*p[i]*p[i];
				int pow_2 = p[i]*p[i];

				int sum = (pow_5 - 3 * pow_4 + 4 * pow_3 + 2 * pow_2 - 10 * p[i] - 4);
				if (sum >= 0) s += sum;
				else s -= sum;
			};
			return s;
		};
	}

}
#endif