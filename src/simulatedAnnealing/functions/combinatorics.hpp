#pragma once
#include "../geometry/combination.hpp"
#include <tuple>
#include <stdexcept>
#include <exception>

typedef unsigned int domain_dim;

namespace TestFunctions {

	typedef std::tuple<double, double> value_weight_pair;

	template <domain_dim Dim>
	std::function<double(const combination_nd<Dim>&)> make_knapsack(const std::vector<value_weight_pair>& v_w_pairs) {
		if (Dim != v_w_pairs.size())
			throw std::invalid_argument("Mismatch between knapsack problem dimension and size of given pairs");
		return [&v_w_pairs](const combination_nd<Dim> c) -> double {
			double v = 0.0;
			for (auto it = c.begin(); it != c.end(); ++it) {
				v += std::get<1>(v_w_pairs[*it]);
			}
			// Make this into a minimization problem by inverting the value function!
			return -v;
		};
	}

}