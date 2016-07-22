#pragma once

#include "Bezier.hpp"
#include "MathVector.hpp"

class TextureGL;

class RoadPatch {
public:
	RoadPatch() : curvature(0) { }

	const Bezier& getPatch() const { return patch; }
		  Bezier& getPatch()	   { return patch; }

	// Returns true if the given ray intersects this patch.
	// Outputs the contact point and normal.
	bool collide(const MathVector<float, 3>& origin, const MathVector<float, 3>& direction, float segLen,
				 MathVector<float, 3>& outtri, MathVector<float, 3>& normal) {
		bool col = patch.collideSubDivQuadSimpleNorm(origin, direction, outtri, normal);
		return col && ((outtri - origin).magnitude() <= segLen);
	}

private:
	float curvature;
	Bezier patch;
};