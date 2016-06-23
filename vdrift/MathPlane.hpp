#pragma once

#include "MathVector.hpp"

template <class T>
class MathPlane {
public:
	MathPlane() {}
	MathPlane(const T a, const T b, const T c, const T d) : v(a, b, c, d) {}
	MathPlane(const MathPlane<T>& other) { std::memcpy(&v, &other, sizeof(MathPlaneABCD)); }
	template <typename T2>
	MathPlane(const MathPlane<T2>& other) { *this = other; }

	inline void set(const T a, const T b, const T c, const T d) { v = MathPlaneABCD(a, b, c, d); }
	inline void set(const T* arr) { std::memcpy(&v, arr, sizeof(MathPlaneABCD)); }

	inline const T& operator[](const int i) const {
		assert(i < 4);
		return v[i];
	}
	inline T& operator[](const int i) {
		assert(i < 4);
		return v[i];
	}

	template <typename T2>
	const MathPlane<T>& operator=(const MathPlane<T2>& other) {
		v.a = other[0];
		v.b = other[1];
		v.c = other[2];
		v.d = other[3];

		return *this;
	}

	template <typename T2>
	inline bool operator==(const MathPlane<T2>& other) const {
		return (v.a == other[0] && v.b == other[1] && v.c == other[2] && v.d == other[3]);
	}

	template <typename T2>
	inline bool operator!=(const MathPlane<T2>& other) const {
		return (v.a != other[0] || v.b != other[1] || v.c != other[2] || v.d != other[3]);
	}

	T distanceToPoint(const MathVector<T, 3>& point) const {
		T abcSq = sqrt(v.a * v.a + v.b * v.b + v.c * v.c);
		assert(abcSq != 0);
		return (v.a * point[0] + v.b * point[1] + v.c * point[2] + v.d) / abcSq;
	}

private:
	struct MathPlaneABCD {
		T a, b, c, d;
		inline const T& operator[](const int i) const { return ((T*)this)[i]; }
		inline T& operator[](const int i) { return ((T*)this)[i]; }

		MathPlaneABCD() : a(0), b(1), c(0), d(0) {}
		MathPlaneABCD(const T na, const T nb, const T nc, const T nd) : a(na), b(nb), c(nc), d(nd) {}
	} v;
};

template <typename T>
std::ostream& operator<<(std::ostream& os, const MathPlane<T>& v) {
	for (size_t i = 0; i < 3; i++) {
		os << v[i] << ", ";
	}
	os << v[3];
	return os;
}
