#pragma once
#ifndef GEOMETRY_POINT_HPP
#define GEOMETRY_POINT_HPP

#include <type_traits>
#include <initializer_list>
#include <functional>
#include <stdexcept>
#include <cstring>
#include <iostream>
#include <array>
#include <cmath>

#include "point_base.hpp"

template <domain_dim Dim>
struct point_nd: public PointBase<Dim> {

public:

    std::array<double, Dim> raw_data;

    // Disallow not initialized points.
    point_nd() = delete;

    point_nd(
        const std::initializer_list<double> init_list
    ) {
        static_assert(Dim > 0, "Error: wrong dim size");
        if (init_list.size() != Dim && init_list.size() != 1)
            throw std::invalid_argument("Bad number of arguments");
        auto it = init_list.begin();
        if (init_list.size() == 1)
            //// Needed to allow initialization of the type point_nd<Dim> =  { 1.0 };
            for (int i = 0; i < Dim; ++i) raw_data[i] = *it;
        else
            for (int i = 0; i < Dim; ++i, ++it) raw_data[i] = *it;
    }

    point_nd( // Initialization of the type point_nd<D> p = scalar
        const double value
    ) {
        static_assert(Dim > 0, "Error: wrong dim size");
        for (int i = 0; i < Dim; ++i) raw_data[i] = value;
    }

    point_nd<Dim> operator+(const point_nd<Dim>& other) const {
        point_nd<Dim> pr = 0.0;
        for (int i = 0; i < Dim; ++i)
            pr.raw_data[i] = raw_data[i] + other.raw_data[i];
        return pr;
    }

    point_nd<Dim>& operator+=(const point_nd<Dim>& other) {
        for (int i = 0; i < Dim; ++i)
            raw_data[i] += other.raw_data[i];
        return *this;
    }

    point_nd<Dim> operator*(const point_nd<Dim>& other) const {
        point_nd<1> pr = 0.0;
        for (int i = 0; i < Dim; ++i)
            pr.raw_data[i] = raw_data[i] * other.raw_data[i];
        return pr;
    }

    point_nd<Dim> operator-() {
        point_nd<Dim> pr = 0.0;
        for (int i = 0; i < Dim; ++i)
            raw_data[i] = -pr.raw_data[i];
        return pr;
    }

    point_nd<Dim>& operator*=(const double scalar) {
        for (int i = 0; i < Dim; ++i)
            raw_data[i] *= scalar;
        return *this;
    }

    point_nd<Dim>& operator/=(const double scalar) {
        for (int i = 0; i < Dim; ++i)
            raw_data[i] /= scalar;
        return *this;
    }

    template <typename Ind>
    double& operator[](const Ind index) {
        static_assert(std::is_integral<Ind>());
        // NOTE: No boundary checks!
        return raw_data[index];
    }

    template <typename Ind>
    double operator[](const Ind index) const {
        static_assert(std::is_integral<Ind>());
        // NOTE: No boundary checks!
        return raw_data[index];
    }

    constexpr long byte_size() const {
        return sizeof(double) * Dim;
    }

    constexpr long dimension() const {
        return Dim;
    }

    double norm_squared() const {
        double d = 0.0;
        for (unsigned int i = 0; i < Dim; ++i)
            d += raw_data[i] * raw_data[i];
        return d;
    }

    double sum() const {
        double d = 0.0;
        for (unsigned int i = 0; i < Dim; ++i)
            d += raw_data[i];
        return d;
    }

    double prod() const {
        double d = raw_data[0];
        for (unsigned int i = 1; i < Dim; ++i)
            d *= raw_data[i];
        return d;
    }

    point_nd elwise_prod(const point_nd<Dim>& p2) const {
        point_nd<Dim> pq = 0.0;
        for (unsigned int i = 0; i < Dim; ++i)
            pq[i] = raw_data[i] * p2.raw_data[i];
        return pq;
    }

    point_nd elwise_div(const point_nd<Dim>& p2) const {
        point_nd<Dim> pq = 0.0;
        for (unsigned int i = 0; i < Dim; ++i)
            pq[i] = raw_data[i] / p2.raw_data[i];
        return pq;
    }

    point_nd operator^(const long pow) const {
        point_nd<Dim> pq = 0.0;
        for (unsigned int i = 0; i < Dim; ++i)
            pq[i] = std::pow(raw_data[i], pow);
        return pq;
    }

    double norm() const {
        return std::sqrt(norm_squared());
    }

    friend point_nd<Dim> operator-(const point_nd<Dim>& p1, const point_nd<Dim>& p2) {
        point_nd<Dim> pr = 0.0;
        for (int i = 0; i < Dim; ++i)
            pr.raw_data[i] = p1.raw_data[i] - p2.raw_data[i];
        return pr;
    }

    friend point_nd<Dim> operator*(const double d, const point_nd<Dim>& p) {
        point_nd<Dim> pr = 0.0;
        for (int i = 0; i < Dim; ++i)
            pr.raw_data[i] = p.raw_data[i]*d;
        return pr;
    }

    operator double() const {
        static_assert(Dim == 1, "Conversion to double is only allowed for point_nd<1>");
        return raw_data[0];
    }
    
    template <domain_dim Start, domain_dim End>
    point_nd<End - Start> slice() const {
        static_assert(Start < End, "Start index must be less than end index.");
        point_nd<End - Start> sliced_point = 0.0;
        for (unsigned int i = Start; i < End; ++i)
            sliced_point[i-Start] = raw_data[i];
        return sliced_point;
    }

    template <domain_dim Start, domain_dim End>
    static point_nd<1 + End - Start> int_range() {
        static_assert(Start < End, "Start index must be less than end index.");
        point_nd<1 + End - Start> sliced_point = 0.0;
        for (unsigned int i = Start; i < End+1; ++i)
            sliced_point[i - Start] = i;
        return sliced_point;
    }

protected:
   

};

template <domain_dim Dim>
std::ostream& operator<<(std::ostream& os, const point_nd<Dim>& p) {
    os << "(";
    if constexpr (Dim > 0)
        for (int i = 0; i < Dim - 1; ++i)
            os << p[i] << ", ";
    os << p[Dim - 1] << ")";
    return os;
}

namespace VectorMath {

    template <domain_dim Dim>
    point_nd<Dim> sin(const point_nd<Dim>& p) {
        point_nd<Dim> result = 0.0;
        for (unsigned int i = 0; i < Dim; ++i) {
            result[i] = std::sin(p[i]);  // Apply sin to each element
        }
        return result;
    }

    template <domain_dim Dim>
    point_nd<Dim> cos(const point_nd<Dim>& p) {
        point_nd<Dim> result = 0.0;
        for (unsigned int i = 0; i < Dim; ++i) {
            result[i] = std::cos(p[i]);  // Apply sin to each element
        }
        return result;
    }

    template <domain_dim Dim>
    point_nd<Dim> sqrt(const point_nd<Dim>& p) {
        point_nd<Dim> result = 0.0;
        for (unsigned int i = 0; i < Dim; ++i) {
            result[i] = std::sqrt(p[i]);  // Apply sin to each element
        }
        return result;
    }

    template <domain_dim Dim>
    point_nd<Dim> abs(const point_nd<Dim>& p) {
        point_nd<Dim> result = 0.0;
        for (unsigned int i = 0; i < Dim; ++i) {
            result[i] = std::abs(p[i]);  // Apply sin to each element
        }
        return result;
    }

    template <domain_dim Dim>
    std::function<point_nd<Dim>(const point_nd<Dim>& p)> vectorize(const std::function<double(double)> &f) {
        return [&f](const point_nd<Dim>& p) {
            point_nd<Dim> result = 0.0;
            for (unsigned int i = 0; i < Dim; ++i) {
                result[i] = f(p[i]);  // Apply the function to each element
            }
            return result;
        };
    }

}

#endif //! GEOMETRY_POINT