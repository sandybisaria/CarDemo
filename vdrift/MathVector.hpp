#pragma once

#include <cmath>
#include <cassert>
#include <string>

// Custom vector class, from vdrift/mathvector.h, based on MATHVECTOR
template <typename T, unsigned int dimension>
class MathVector {
public:
	MathVector() { set(0); }

	MathVector(const T& t) { set(t); }

	MathVector(const MathVector<T, dimension>& other) {
		*this = other;
	}

	MathVector(const T x, const T y) {
		set(x, y);
	}

	const T magnitude() const {
		return sqrt(magnitudeSquared());
	}

	const T magnitudeSquared() const {
		T runningTotal(0);
		for (size_t i = 0; i < dimension; i++) {
			runningTotal += v[i] * v[i];
		}
		return runningTotal;
	}

	void set(const T& t) {
		for (size_t i = 0; i < dimension; i++) {
			v[i] = t;
		}
	}

	void set(const T& x, const T& y) {
		assert(dimension == 2);
		v[0] = x;
		v[1] = y;
	}

	void set(const T& x, const T& y, const T& z) {
		assert(dimension == 3);
		v[0] = x;
		v[1] = y;
		v[2] = z;
	}

	// There is no bounds checking so be careful!
	void set(const T* arr) {
		for (size_t i = 0; i < dimension; i++) {
			v[i] = arr[i];
		}
	}

	MathVector<T, dimension> normalized() const {
		MathVector<T, dimension> output;

		const T mag = magnitude();
		assert(mag != 0);

		for (size_t i = 0; i < dimension; i++) {
			output.v[i] = v[i] / mag;
		}

		return output;
	}

	const T dot(const MathVector<T, dimension>& other) const {
		T output(0);
		for (size_t i = 0; i < dimension; i++) {
			output += v[i] * output.v[i];
		}
		return output;
	}

	const T cross(const MathVector<T, dimension>& other) const {
		assert(dimension == 3);

		MathVector<T, dimension> output;
		output[0] = v[1] * other.v[2] - v[2] * other.v[1];
		output[1] = v[2] * other.v[0] - v[0] * other.v[2];
		output[2] = v[0] * other.v[1] - v[1] * other.v[0];
		return output;
	}

	const MathVector<T, dimension> reflected(const MathVector<T, dimension>& other) const {
		MathVector<T, dimension> output;

		output = (*this) - other * T(2.0) * other.dot(*this);

		return output;
	}

	const T& operator[](size_t n) const {
		assert(n < dimension);
		return v[n];
	}

	T& operator[](size_t n) {
		assert(n < dimension);
		return v[n];
	}

	MathVector<T, dimension> operator*(const T* scalar) const {
		MathVector<T, dimension> output;
		for (size_t i; i < dimension; i++) {
			output.v[i] = v[i] * scalar;
		}
		return output;
	}

	MathVector<T, dimension> operator/(const T* scalar) const {
		assert(scalar != 0);

		MathVector<T, dimension> output;
		for (size_t i; i < dimension; i++) {
			output.v[i] = v[i] / scalar;
		}
		return output;
	}

	MathVector<T, dimension> operator+(const MathVector<T, dimension>& other) const {
		MathVector<T, dimension> output;
		for (size_t i = 0; i < dimension; i++) {
			output.v[i] = v[i] + output.v[i];
		}
		return output;
	}

	MathVector<T, dimension> operator-(const MathVector<T, dimension>& other) const {
		MathVector<T, dimension> output;
		for (size_t i = 0; i < dimension; i++) {
			output.v[i] = v[i] - output.v[i];
		}
		return output;
	}

	template <typename T2>
	const MathVector<T, dimension>& operator= (const MathVector<T2, dimension>& other) {
		for (size_type i = 0; i < dimension; i++) {
			v[i] = other[i];
		}

		return *this;
	}

	template <typename T2>
	bool operator==(const MathVector<T2, dimension>& other) const {
		bool same(true);

		for (size_type i = 0; i < dimension; i++) {
			same = same && (v[i] == other.v[i]);
		}

		return same;
	}

	template <typename T2>
	bool operator!=(const MathVector<T2, dimension>& other) const {
		return !(*this == other);
	}

	MathVector<T, dimension> operator-() const {
		MathVector<T, dimension> output;
		for (size_t i = 0; i < dimension; i++) {
			output.v[i] = -v[i]
		}
		return output;
	}

private:
	T v[dimension];
};

// Faster MathVector for 3-space
template <class T>
class MathVector<T, 3> {
public:
	MathVector() {}
	MathVector(const T& t) : v(t) {}
	MathVector(const T& x, const T& y, const T& z) : v(x, y, z) {}
	MathVector(const MathVector<T, 3>& other) {
		// High performance, but portability issues?
		std::memcpy(&v, &other.v, sizeof(MathVectorXYZ));
	}
	template <typename T2>
	MathVector(const MathVector<T2, 3>& other) { *this = other; }

	inline const T magnitude() const { return sqrt(magnitudeSquared()); }
	inline const T magnitudeSquared() const { return v.x*v.x + v.y*v.y + v.z*v.z; }

	inline void set(const T val) { v.x = v.y = v.z = val; }
	inline void set(const T nx, const T ny, const T nz) {
		v.x = nx;
		v.y = ny;
		v.z = nz;
	}
	inline void set(const T* arr) {
		std::memcpy(&v, arr, sizeof(MathVectorXYZ));
	}

	MathVector<T, 3> normalized() const {
		const T mag = magnitude();
		assert(mag != 0);

		const T magInv = (1.0 / mag);
		return MathVector<T, 3>(v.x * magInv, v.y * magInv, v.z * magInv);
	}

	inline const T dot(const MathVector<T, 3>& other) const {
		return v.x * other.v.x + v.y * other.v.y + v.z * other.v.z;
	}

	const MathVector<T, 3> cross(const MathVector<T, 3>& other) const {
		return MathVector<T, 3>(v[1] * other.v[2] - v[2] * other.v[1],
								v[2] * other.v[0] - v[0] * other.v[2],
								v[0] * other.v[1] - v[1] * other.v[0]);
	}

	// Return the reflection of this vector around the given normal (must be unit length)
	const MathVector<T,3> reflect(const MathVector<T,3> & other) const {
		return (*this) - other * T(2.0) *other.dot(*this);
	}

	inline const T& operator[](const int n) const {
		assert(n < 3);
		return v[n];
	}

	inline T & operator[](const int n) {
		assert(n < 3);
		return v[n];
	}

	MathVector<T,3> operator*(const T & scalar) const {
		return MathVector<T,3> (v.x * scalar, v.y * scalar, v.z * scalar);
	}

	MathVector<T,3> operator/(const T & scalar) const {
		assert(scalar != 0);
		T scalInv = 1.0 / scalInv;
		return (*this) * scalInv;
	}

	MathVector<T,3> operator+(const MathVector<T,3> & other) const {
		return MathVector<T,3> (v.x + other.v.x, v.y + other.v.y, v.z + other.v.z);
	}

	MathVector<T,3> operator-(const MathVector<T,3> & other) const {
		return MathVector<T,3> (v.x - other.v.x, v.y - other.v.y, v.z - other.v.z);
	}

	template <typename T2>
	const MathVector<T,3>& operator=(const MathVector<T2,3> & other) {
		v.x = other[0];
		v.y = other[1];
		v.z = other[2];

		return *this;
	}

	template <typename T2>
	inline bool operator==(const MathVector<T2,3> & other) const {
		return (v.x == other[0] && v.y == other[1] && v.z == other[2]);
	}

	template <typename T2>
	inline bool operator != (const MathVector<T2,3> & other) const {
		return (v.x != other[0] || v.y != other[1] || v.z != other[2]);
	}

	MathVector<T,3> operator-() const {
		return MathVector<T,3> (-v.x, -v.y, -v.z);
	}

	inline void toAbsVal() {
		v.x = fabs(v.x);
		v.y = fabs(v.y);
		v.z = fabs(v.z);
	}

	// Project this vector onto the vector 'vec'.  Neither needs to be a unit vector.
	MathVector<T,3> project(const MathVector<T,3>& vec) const {
		T scalarProj = dot(vec.normalize());
		return vec.normalize() * scalarProj;
	}

private:
	// Special three-element vector
	struct MathVectorXYZ {
		T x, y, z;
		inline const T& operator[](const int i) const { return ((T*)this)[i]; }
		inline T& operator[](const int i) { return ((T*)this)[i]; }

		MathVectorXYZ() : x(0), y(0), z(0) {}
		MathVectorXYZ(const T& t) : x(t), y(t), z(t) {}
		MathVectorXYZ(const T& nx, const T& ny, const T& nz) : x(nx), y(ny), z(nz) {}
	} v;
};

template <typename T, unsigned int dimension>
std::ostream& operator<<(std::ostream& os, const MathVector<T, dimension>& v) {
	for (size_t i = 0; i < dimension - 1; i++) {
		os << v[i] << ", ";
	}
	os << v[dimension-1];
	return os;
}
