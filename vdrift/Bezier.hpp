#pragma once

#include "MathVector.hpp"
#include "AABB.hpp"

#include <ostream>
#include <cassert>
#include <cmath>
#define _USE_MATH_DEFINES

class Bezier {
public:
	Bezier();
	Bezier(const Bezier& other) { copyFrom(other); }

	~Bezier() { }

	Bezier& operator=(const Bezier& other) { return copyFrom(other); }
	Bezier& copyFrom(const Bezier& other);

	friend std::ostream& operator<<(std::ostream& os, const Bezier& b);

	float getDistFromStart() const { return distFromStart; }
	void resetDistFromStart() { distFromStart = 0.f; }

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

	// Flip points on both axes
	void reverse();

	// Diagnostic that checks for twisted Beziers; true if there is a problem
	bool checkForProblems() const;

	// Halve the Bezier defined by the given size 4 points4 array into the output size 4 arrays left4 and right4
	void deCasteljauHalveCurve(MathVector<float, 3>* points4, MathVector<float,3>* left4,
							   MathVector<float, 3>* right4) const;

	// Access corners of the patch (front left, front right, back left, back right)
	const MathVector<float, 3>& getFrontLeft() { return points[0][0]; }
	const MathVector<float, 3>& getFrontRight() { return points[0][ARR_SIZE-1]; }
	const MathVector<float, 3>& getBackLeft() { return points[ARR_SIZE-1][0]; }
	const MathVector<float, 3>& getBackRight() { return points[ARR_SIZE-1][ARR_SIZE-1]; }

	AABB<float> getAABB() const;

	// x = n % 4, y = n / 4
	const MathVector<float, 3>& operator[](const int n) const {
		assert(n < ARR_SIZE * ARR_SIZE);
		int x = n % 4, y = n / 4;
		return points[x][y];
	}
	const MathVector<float, 3>& getPoint(const unsigned int x, const unsigned int y) const {
		assert(x < ARR_SIZE && y < ARR_SIZE); return points[x][y];
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
	bool intersectsQuadrilateral(const MathVector<float, 3>& orig, const MathVector<float, 3>& dir,
								 const MathVector<float, 3>& v_00, const MathVector<float, 3>& v_10,
								 const MathVector<float, 3>& v_11, const MathVector<float, 3>& v_01,
								 float &t, float &u, float &v) const;

	const static int ARR_SIZE = 4;
	MathVector<float, 3> points[ARR_SIZE][ARR_SIZE];
	MathVector<float, 3> center;
	float radius;
	float length;
	float distFromStart; //TODO Is this needed or even relevant?

	Bezier* nextPatch;
	float roadRadius;
	int turn; //-1 = left turn, +1 = right turn, 0 = straight
	float roadCurvature;
	//TODO Ignore "racing line" variables
};
