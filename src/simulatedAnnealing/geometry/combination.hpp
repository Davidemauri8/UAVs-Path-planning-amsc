#pragma once
#ifndef GEOMETRY_COMBINATION_HPP
#define GEOMETRY_COMBINATION_HPP

#include <cmath>
#include <cstring>
#include <bitset>

#include "point_base.hpp"

#define select_bit(b, string)	string[b / 8]
#define mask_for(b)				( (unsigned long long) 1 << (unsigned long long) b )

template <domain_dim Dim>
class combination_nd: public PointBase<Dim> {

	// Store this char by char to avoid issues with endianness
	std::array<unsigned char, ( Dim + 7 ) / 8> binary_string;

public:

	combination_nd() {
		reset();
	}

	// For compatibility
	combination_nd(const double d) {
		reset();
	}

	void reset() {
		memset(binary_string.data(), 0.0, sizeof(binary_string));
	}

	constexpr unsigned long byte_size() const {
		return sizeof(binary_string);
	}

	unsigned char get_byte(long i) const {
		if (i >= binary_string.size())
			return 0;
		else if (i == binary_string.size() - 1) {
			unsigned char b = binary_string[i];
			// Notice this bit hack: we get the mask with 1s only in the allowed values
			// by subtracting from the next-possible bit..
			b &= ((unsigned long) 1 << (Dim % 8)) - 1;
			return b;
		}
		return binary_string[i];
	}

	void flip(long bit) {
		select_bit(bit, binary_string) ^= mask_for((bit % 8));
		return;
	}

	void set(long bit) {
		select_bit(bit, binary_string) |= mask_for((bit % 8));
		return;
	}

	void unset(long bit) {
		select_bit(bit, binary_string) &= ~mask_for((bit % 8));
		return;
	}


	long get(long bit) const {
		return select_bit(bit, binary_string) & mask_for((bit % 8));
	}

	unsigned char operator[](long index) const {
		// NOTE: no boundary checks!
		return binary_string[index];
	}

	class NonZeroBitIterator {

		const combination_nd<Dim>& container;
		long index;

	public:
		NonZeroBitIterator(const combination_nd<Dim>& c, long start)
			: container(c), index(start) {
			while (index < Dim && !container.get(index)) {
				++index;
			}
		}

		long operator*() const {
			return index;
		}

		NonZeroBitIterator& operator++() {
			do {
				++index;
			} while (index < Dim && !container.get(index));
			return *this;
		}

		bool operator!=(const NonZeroBitIterator& other) const {
			return index != other.index;
		}
	};

	NonZeroBitIterator begin() const {
		return NonZeroBitIterator(*this, 0);
	}

	NonZeroBitIterator end() const {
		return NonZeroBitIterator(*this, Dim);
	}

};


template <domain_dim Dim>
std::ostream& operator<<(std::ostream& os, const combination_nd<Dim>& p) {
	os << "(";
	if constexpr (Dim > 0)
		for (int i = p.byte_size() - 1; i >= 1; --i)
			os << std::bitset<8>(p.get_byte(i)) << "|";
	os << std::bitset<8>(p[0]) << ")";
	return os;
}

#undef select_bit
#undef mask_for
#endif