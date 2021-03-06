#pragma once

#include "../vdrift/MathVector.hpp"
#include "../vdrift/Matrix3.hpp"
#include "../vdrift/Quaternion.hpp"

#include "LinearMath/btMatrix3x3.h"
#include "LinearMath/btQuaternion.h"
#include "LinearMath/btVector3.h"

// vdrift to Bullet utility methods
static btVector3 toBulletVector(const MathVector<float, 3>& v) {
	return btVector3(v[0], v[1], v[2]);
}

static btQuaternion toBulletQuaternion(const Quaternion<float>& q) {
	return btQuaternion(q.x(), q.y(), q.z(), q.w());
}

static btQuaternion toBulletQuaternion(const Quaternion<double>& q) {
	return btQuaternion(q.x(), q.y(), q.z(), q.w());
}

static btMatrix3x3 toBulletMatrix(const Matrix3<float>& m) {
	return btMatrix3x3(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);
}

template <typename T> static MathVector<T, 3> toMathVector(const btVector3& v) {
	return MathVector<T, 3>(v.x(), v.y(), v.z());
}

template <typename T> static Quaternion<T> toMathQuaternion(const btQuaternion& q) {
	return Quaternion<T>(q.x(), q.y(), q.z(), q.w());
}
