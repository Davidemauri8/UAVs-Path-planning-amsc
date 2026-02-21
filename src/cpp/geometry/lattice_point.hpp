#pragma once
#ifndef GEOMETRY_LATTICE_POINT
#define GEOMETRY_LATTICE_POINT

#include <type_traits>
#include <initializer_list>
#include <stdexcept>
#include <cstring>
#include <iostream>
#include <cmath>

typedef unsigned int domain_dim;


template <domain_dim Dim>
struct lattice_nd {

public:

    std::array<long, Dim> raw_data;

    lattice_nd() = delete;

    lattice_nd(
        const std::initializer_list<long> init_list
    ) {
        static_assert(Dim > 0, "Error: wrong dim size");
        if (init_list.size() != Dim && init_list.size() != 1)
            throw std::invalid_argument("Bad number of arguments");
        if (init_list.size() == 1)
            for (int i = 0; i < Dim; ++i) raw_data[i] = *init_list.begin();
        else {
            auto it = init_list.begin();
            for (int i = 0; i < Dim; ++i, ++it) raw_data[i] = *it;
        }
    }

    lattice_nd(
        const long value
    ) {
        static_assert(Dim > 0, "Error: wrong dim size");
        for (int i = 0; i < Dim; ++i)
            raw_data[i] = value;
    }

    lattice_nd& operator=(
        const long scalar //
        ) {
        for (int i = 0; i < Dim; ++i) raw_data[i] = scalar;

        return this;
    }

    template <typename Ind>
    int operator[](const Ind index) const {
        static_assert(std::is_integral<Ind>());
        // NOTE: No boundary checks!
        return raw_data[index];
    }

    template <typename Ind>
    long& operator[](const Ind index) {
        static_assert(std::is_integral<Ind>());
        // NOTE: No boundary checks!
        return raw_data[index];
    }

    constexpr long byte_size() const {
        return sizeof(long) * Dim;
    }

    lattice_nd<Dim>& operator/=(const double scalar) {
        for (int i = 0; i < Dim; ++i)
            raw_data[i] /= scalar;
        return *this;
    }

    lattice_nd<Dim>& operator*=(const double scalar) {
        for (int i = 0; i < Dim; ++i)
            raw_data[i] *= scalar;
        return *this;
    }


    lattice_nd<Dim>& operator+=(const lattice_nd<Dim>& other) {
        for (int i = 0; i < Dim; ++i)
            raw_data[i] += other.raw_data[i];
        return *this;
    }

    lattice_nd<Dim> operator+(const lattice_nd<Dim>& other) {
        lattice_nd<Dim> pr;
        for (unsigned int i = 0; i < Dim; ++i)
            pr.raw_data[i] = raw_data[i] + other.raw_data[i];
        return pr;
    }

    lattice_nd<Dim> operator-(const lattice_nd<Dim>& other) {
        lattice_nd<Dim> pr = 0.0;
        for (unsigned int i = 0; i < Dim; ++i)
            pr.raw_data[i] = raw_data[i] - other.raw_data[i];
        return pr;
    }

    double l1_norm_squared() const {
        double d = 0.0;
        for (unsigned int i = 0; i < Dim; ++i)
            d += std::abs(raw_data[i]);
        return d;
    }

    double l1_norm() {
        return std::sqrt(l1_norm_squared());
    }

    friend lattice_nd<Dim> operator-(const lattice_nd<Dim>& p1, const lattice_nd<Dim>& p2) {
        lattice_nd<Dim> pr = 0.0;
        for (int i = 0; i < Dim; ++i)
            pr.raw_data[i] = p1.raw_data[i] - p2.raw_data[i];
        return pr;
    }

};

template <domain_dim Dim>
std::ostream& operator<<(std::ostream& os, const lattice_nd<Dim>& p) {
    os << "(";
    if constexpr (Dim > 0)
        for (int i = 0; i < Dim - 1; ++i)
            os << p[i] << ", ";
    os << p[Dim - 1] << ")";
    return os;
}

#endif //! GEOMETRY_POINT