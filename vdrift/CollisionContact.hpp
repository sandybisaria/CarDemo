#pragma once

#include "TerrainSurface.hpp"
#include "MathVector.hpp"
#include "Bezier.hpp"

#include <btBulletCollisionCommon.h>
#include <cassert>

class CollisionContact {
public:
	CollisionContact()
		: depth(0), surface(TerrainSurface::none()), patch(NULL), colObj(NULL) { }

	const MathVector<float, 3>& getPosition() const { return position; }
	const MathVector<float, 3>& getNormal() const { return normal; }
	float getDepth() const { return depth; }
	const TerrainSurface* getSurfacePtr() const { return surface; }
	const TerrainSurface& getSurface() const { return *surface; }
	const Bezier* getPatch() const { return patch; }
	const btCollisionObject* getCollisionObject() const { return colObj; }

	// Set contact data (used by ray cast)
	void set(const MathVector<float, 3>& p, const MathVector<float, 3>& n, float d,
			 const TerrainSurface* s, const Bezier* b, const btCollisionObject* c) {
		assert(s != NULL);
		position = p; normal = n; depth = d; surface = s; patch = b; colObj = c;
	}

	// Update/interpolate contact
	bool castRay(const MathVector<float, 3>& origin, const MathVector<float, 3>& dir, float len) {
		// Plane-based approximation
		float nd = normal.dot(dir);
		if (nd < 0) {
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
	const Bezier* patch;
	const btCollisionObject* colObj;
};
