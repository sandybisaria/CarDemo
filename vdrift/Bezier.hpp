#pragma once

#include "MathVector.hpp"
#include "AABB.hpp"

#include <iostream>
#include <cassert>
#include <cmath>
#define _USE_MATH_DEFINES

// Stuntrally's BEZIER class
//TODO Investigate if actually needed; left in for now due to use in Collision classes
class Bezier {
public:
	Bezier();
	Bezier(const Bezier& other) { copyFrom(other); }

	~Bezier() { }

	Bezier& operator=(const Bezier& other) { return copyFrom(other); }
	Bezier& copyFrom(const Bezier& other);

	friend std::ostream& operator<<(std::ostream& os, const Bezier& b);

	void resetNextPatch() { nextPatch = NULL; }

	// Initialize this Bezier to the quad defined by the given corner points
	void setFromCorners(const MathVector<float, 3>& fl, const MathVector<float, 3>& fr,
						const MathVector<float, 3>& bl, const MathVector<float, 3>& br);

	// Attach this Bezier and the other Bezier by moving them and adjusting control points as necessary.
	// Note that the other Bezier will be modified.
	void attach(Bezier& other, bool reverse);
	void attach(Bezier& other) { attach(other, false); }

	// Return true if the ray starting at the given origin going in the given direction intersects this Bezier.
	// Output the contact point and normal to the given outtri and normal variables.
	bool collideSubDivQuadSimple(const MathVector<float, 3>& origin, const MathVector<float, 3>& direction,
								 MathVector<float, 3>& outtri) const;
	bool collideSubDivQuadSimpleNorm(const MathVector<float, 3>& origin, const MathVector<float, 3>& direction,
									 MathVector<float, 3>&outtri, MathVector<float, 3>& normal) const;

	void readFrom(std::istream &file);

	// Flip points on both axes
	void reverse();

	// Diagnostic that checks for twisted Beziers; true if there is a problem
	bool checkForProblems() const;

	// Halve the Bezier defined by the given size 4 points4 array into the output size 4 arrays left4 and right4
	void deCasteljauHalveCurve(MathVector<float, 3>* points4, MathVector<float,3>* left4,
							   MathVector<float, 3>* right4) const;

	// Access corners of the patch (front left, front right, back left, back right)
	const MathVector<float, 3>& getFrontLeft() { return points[0][0]; }
	const MathVector<float, 3>& getFrontRight() { return points[0][3]; }
	const MathVector<float, 3>& getBackLeft() { return points[3][0]; }
	const MathVector<float, 3>& getBackRight() { return points[3][3]; }

	AABB<float> getAABB() const;

	// x = n % 4, y = n / 4
	const MathVector<float, 3>& operator[](int n) const {
		assert(n < 4 * 4);
		int x = n % 4, y = n / 4;
		return points[x][y];
	}
	const MathVector<float, 3>& getPoint(unsigned int x,
										 unsigned int y) const {
		assert(x < 4 && y < 4); return points[x][y];
	}

	Bezier* getNextPatch() const { return nextPatch; }
	float getRoadRadius() const { return roadRadius; }

private:
	// Return the Bernstein given the normalized coordinate u and an array of four points p
	MathVector<float, 3> bernstein(float u, MathVector<float, 3>* p) const;

	// Return the Bernstein tangent (normal) given the normalized coordinate u and an array of four points p
	MathVector<float, 3> bernsteinTangent(float u, MathVector<float, 3> *p) const;

	// Return the 3D point on the Bezier surface at the given normalized coordinates px and py
	MathVector<float, 3> surfCoord(float px, float py) const;

	// Return the normal of the Bezier surface at the given normalized coordinates px and py
	MathVector<float, 3> surfNorm(float px, float py) const;

	// Return true if the ray at orig with direction dir intersects the given quadrilateral.
	// Also put the collision depth in t and the collision coordinates in (u, v)
	bool intersectsQuadrilateral(const MathVector<float, 3>& orig,
								 const MathVector<float, 3>& dir,
								 const MathVector<float, 3>& v_00,
								 const MathVector<float, 3>& v_10,
								 const MathVector<float, 3>& v_11,
								 const MathVector<float, 3>& v_01,
								 float &t, float &u, float &v) const;

	MathVector<float, 3> points[4][4];
	MathVector<float, 3> center;
	float radius;
	float length;
	// Ignore distFromStart

	Bezier* nextPatch;
	float roadRadius;
	int turn; // -1 = left turn, +1 = right turn, 0 = straight
	float roadCurvature;
	// Ignored "racing line" variables
};
