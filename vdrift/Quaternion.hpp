#pragma once

#include "MathVector.hpp"

// From vdrift/quaternion.h
template <typename T>
class Quaternion {
public:
	Quaternion() { loadIdentity(); }
	Quaternion(const T& nx, const T& ny, const T& nz, const T& nw) {
		v[0] = nx;
		v[1] = ny;
		v[2] = nz;
		v[3] = nw;
	}
	Quaternion(const Quaternion<T>& other) { *this = other; }

	// Load the [1,(0,0,0)] quaternion
	void loadIdentity() {
		v[3] = 1;
		v[0] = v[1] = v[2] = 0;
	}

	const T& operator[](size_t n) const {
		assert(n < 4);
		return v[n];
	}

	T & operator[](size_t n) {
		assert(n < 4);
		return v[n];
	}

	const T& x() const { return v[0]; }
	const T& y() const { return v[1]; }
	const T& z() const { return v[2]; }
	const T& w() const { return v[3]; }

	T& x() { return v[0]; }
	T& y() { return v[1]; }
	T& z() { return v[2]; }
	T& w() { return v[3]; }

	template <typename T2>
	const Quaternion<T>& operator=(const Quaternion<T2>& other) {
		for (size_t i = 0; i < 4; i++)
			v[i] = other[i];

		return *this;
	}

	const T magnitude() const {
		return sqrt(v[3]*v[3] + v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
	}

	void normalize() {
		T len = magnitude();
		for (size_t i = 0; i < 4; i++)
			v[i] /= len;
	}

	/// Set the given matrix to a matrix representation of this quaternion.
	/// No array bound checking is done.
	/// The matrix type can be any type that is accessible with [].
	template <typename T2>
	void representAsMatrix4(T2& destMat) const {
		T xx = v[0]*v[0];
		T xy = v[0]*v[1];
		T xz = v[0]*v[2];
		T xw = v[0]*v[3];

		T yy = v[1]*v[1];
		T yz = v[1]*v[2];
		T yw = v[1]*v[3];

		T zz = v[2]*v[2];
		T zw = v[2]*v[3];

		destMat[0] = 1.0 - 2.0*(yy+zz);
		destMat[1] = 2.0*(xy+zw);
		destMat[2] = 2.0*(xz-yw);
		destMat[3] = 0;

		destMat[4] = 2.0*(xy-zw);
		destMat[5] = 1.0-2.0*(xx+zz);
		destMat[6] = 2.0*(yz+xw);
		destMat[7] = 0;

		destMat[8] = 2.0*(xz+yw);
		destMat[9] = 2.0*(yz-xw);
		destMat[10] = 1.0-2.0*(xx+yy);
		destMat[11] = 0;

		destMat[12] = 0;
		destMat[13] = 0;
		destMat[14] = 0;
		destMat[15] = 1;
	}

	// Set the given matrix to a matrix representation of this quaternion.
	// No array bound checking is done.
	// The matrix type can be any type that is accessible with [].
	template <typename T2>
	void representAsMatrix3(T2& destMat) const {
		T xx = v[0]*v[0];
		T xy = v[0]*v[1];
		T xz = v[0]*v[2];
		T xw = v[0]*v[3];

		T yy = v[1]*v[1];
		T yz = v[1]*v[2];
		T yw = v[1]*v[3];

		T zz = v[2]*v[2];
		T zw = v[2]*v[3];

		destMat[0] = 1.0 - 2.0*(yy+zz);
		destMat[1] = 2.0*(xy+zw);
		destMat[2] = 2.0*(xz-yw);

		destMat[3] = 2.0*(xy-zw);
		destMat[4] = 1.0-2.0*(xx+zz);
		destMat[5] = 2.0*(yz+xw);

		destMat[6] = 2.0*(xz+yw);
		destMat[7] = 2.0*(yz-xw);
		destMat[8] = 1.0-2.0*(xx+yy);
	}

	// May return a non-normalized quaternion
	Quaternion<T> operator*(const Quaternion<T>& other ) const {
		T A, B, C, D, E, F, G, H;

		A = (v[3] + v[0]) * (other.v[3] + other.v[0]);
		B = (v[2] - v[1]) * (other.v[1] - other.v[2]);
		C = (v[3] - v[0]) * (other.v[1] + other.v[2]);
		D = (v[1] + v[2]) * (other.v[3] - other.v[0]);
		E = (v[0] + v[2]) * (other.v[0] + other.v[1]);
		F = (v[0] - v[2]) * (other.v[0] - other.v[1]);
		G = (v[3] + v[1]) * (other.v[3] - other.v[2]);
		H = (v[3] - v[1]) * (other.v[3] + other.v[2]);


		Quaternion output(A - ( E + F + G + H) * 0.5,
						  C + ( E - F + G - H) * 0.5,
						  D + ( E - F - G + H) * 0.5,
						  B + (-E - F + G + H) * 0.5);
		return output;
	}

	// May return a non-normalized quaternion
	Quaternion<T> operator*(const T& scalar) const {
		Quaternion output(v[0]*scalar, v[1]*scalar, v[2]*scalar, v[3]*scalar);

		return output;
	}

	// May return a non-normalized quaternion
	Quaternion<T> operator+(const Quaternion<T>& other) const {
		Quaternion output(v[0]+other.v[0], v[1]+other.v[1], v[2]+other.v[2], v[3]+other.v[3]);

		return output;
	}

	template <typename T2>
	bool operator==(const Quaternion<T2>& other) const {
		bool same(true);

		for (size_t i = 0; i < 4; ++i) {
			same = same && (v[i] == other.v[i]);
		}

		return same;
	}

	template <typename T2>
	bool operator!=(const Quaternion<T2>& other) const {
		return !(*this == other);
	}

	// Returns the conjugate
	Quaternion<T> operator-() const {
		Quaternion output;
		output.v[3] = v[3];
		for (size_t i = 0; i < 3; ++i)
		{
			output.v[i] = -v[i];
		}
		return output;
	}

	// Rotate the quaternion around the given axis by the given amount.
	// a is in radians.  The axis is assumed to be a unit vector
	void rotate(const T& a, const T& ax, const T& ay, const T& az) {
		Quaternion output;
		output.setAxisAngle(a, ax, ay, az);
		(*this) = output * (*this);

		normalize();
	}

	// Set the quaternion to rotation a around the given axis.
	// a is in radians.  the axis is assumed to be a unit vector.
	void setAxisAngle(const T& a, const T& ax, const T& ay, const T& az) {
		T sina2 = sin(a/2);

		v[3] = cos(a/2);
		v[0] = ax * sina2;
		v[1] = ay * sina2;
		v[2] = az * sina2;
	}

	// Rotate a vector (accessible with []) by this quaternion.
	// Note that the output is saved back to the input vec.
	template <typename T2>
	void rotateVector(T2& vec) const {
		Quaternion dirconj = -(*this);

		Quaternion qtemp;
		qtemp.v[3] = 0;
		for (size_t i = 0; i < 3; ++i)
			qtemp.v[i] = vec[i];

		Quaternion qout = (*this) * qtemp * dirconj;

		for (size_t i = 0; i < 3; ++i)
			vec[i] = qout.v[i];
	}

	// Get the scalar angle (in radians) between two quaternions
	const T getAngleBetween(const Quaternion<T>& quat2) const {
		// Establish a forward vector
		T forward[3];
		forward[0] = 0;
		forward[1] = 0;
		forward[2] = 1;

		// Create vectors for quaternions
		T vec1[3];
		T vec2[3];
		for (size_t i = 0; i < 3; ++i)
			vec1[i] = vec2[i] = forward[i];

		rotateVector(vec1);
		quat2.rotateVector(vec2);

		// return the angle between the vectors
		T dotprod(0);
		for (size_t i = 0; i < 3; ++i)
			dotprod += vec1[i]*vec2[i];

		return acos(dotprod);
	}

	// Interpolate between this quaternion and another by scalar amount t [0,1] and return the result
	Quaternion<T> quatSlerp (const Quaternion<T>& quat2, const T& t) const {
		T to1[4];
		T omega, cosom, sinom, scale0, scale1;

		// Calc cosine
		cosom = v[0] * quat2.v[0] + v[1] * quat2.v[1] + v[2] * quat2.v[2] + v[3] * quat2.v[3];

		// Adjust signs (if necessary)
		if (cosom < 0.0) {
			cosom = -cosom;
			to1[0] = -quat2.v[0];
			to1[1] = -quat2.v[1];
			to1[2] = -quat2.v[2];
			to1[3] = -quat2.v[3];
		} else {
			to1[0] = quat2.v[0];
			to1[1] = quat2.v[1];
			to1[2] = quat2.v[2];
			to1[3] = quat2.v[3];
		}

		const T DELTA(0.00001);

		//calculate coefficients
		if (1.0 - cosom > DELTA) {
			// Standard case (slerp)
			omega = acos(cosom);
			sinom = sin(omega);
			scale0 = sin((1.0 - t) * omega) / sinom;
			scale1 = sin(t * omega) / sinom;
		} else {
			// "from" and "to" quaternions are very close so we can do a linear interpolation
			scale0 = 1.0 - t;
			scale1 = t;
		}

		//calculate final values
		Quaternion<T> qout;
		qout.v[0] = scale0 * v[0] + scale1 * to1[0];
		qout.v[1] = scale0 * v[1] + scale1 * to1[1];
		qout.v[2] = scale0 * v[2] + scale1 * to1[2];
		qout.v[3] = scale0 * v[3] + scale1 * to1[3];
		qout.normalize();
		return qout;
	}

private:
	T v[4]; // X, Y, Z, W
};

template <typename T>
std::ostream & operator<<(std::ostream &os, const Quaternion<T>& v) {
	os << "x=" << v[0] << ", y=" << v[1] << ", z=" << v[2] << ", w=" << v[3];
	return os;
}
