#pragma once

#include "Bezier.hpp"
#include "MathVector.hpp"
#include "TerrainSurface.hpp"

#include <btBulletCollisionCommon.h>

#include <cassert>

class CollisionContact {
public:
	CollisionContact()
		: depth(0), surface(TerrainSurface::none()), patch(0), colObj(0) { }

	const MathVector<float, 3>& getPosition() const { return position; }
	const MathVector<float, 3>& getNormal() const { return normal; }
	float getDepth() const { return depth; }
	const TerrainSurface* getSurface() const { return surface; }
	const Bezier* getPatch() const { return patch; }
	const btCollisionObject* getCollisionObject() const { return colObj; }

	// Set contact data (used by ray cast)
	void set(const MathVector<float, 3>& pos, const MathVector<float, 3>& norm, float dist,
			 const TerrainSurface* ts, const Bezier* b, const btCollisionObject* c) {
#ifndef COMPILE_UNIT_TESTS
		assert(ts != NULL); // Assertion only needed during actual simulation
#endif
		position = pos; normal = norm; depth = dist; surface = ts; patch = b; colObj = c;
	}

	// Update/interpolate contact
	bool castRay(const MathVector<float, 3>& origin, const MathVector<float, 3>& dir, float len) {
		// Plane-based approximation
		float nd = normal.dot(dir);
		if (nd < 0) {
			// Angle between vectors greater than 90 deg, intersection
			depth = normal.dot(position - origin) / nd;
			position = origin + dir * depth;
			return true;
		}
		position = origin + dir * len;
		depth = len;
		return false;
	}

private:
	MathVector<float, 3> position, normal;
	float depth;
	const TerrainSurface* surface;
	const Bezier* patch; //TODO Seemingly unused
	const btCollisionObject* colObj;
};
