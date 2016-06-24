#include "Bezier.hpp"

Bezier::Bezier()
	: nextPatch(NULL), turn(0), distFromStart(0.f), length(0.f),
	  radius(1), roadRadius(100), roadCurvature(1) { }

Bezier& Bezier::copyFrom(const Bezier& other) {
	for (int x = 0; x < ARR_SIZE; x++)
		for (int y = 0; y < ARR_SIZE; y++)
			points[x][y] = other.points[x][y];

	center = other.center; radius = other.radius; length = other.length;
	distFromStart = other.distFromStart; nextPatch = other.nextPatch;
	roadRadius = other.roadRadius; turn = other.turn; roadCurvature = other.roadCurvature;

	return *this;
}

void Bezier::setFromCorners(const MathVector<float, 3>& fl, const MathVector<float, 3>& fr,
							const MathVector<float, 3>& bl, const MathVector<float, 3>& br) {
	MathVector<float, 3> temp;

	center = fl + fr + bl + br; center = center * 0.25;

	radius = 0;
	if ((fl - center).magnitude() > radius) radius = (fl - center).magnitude();
	if ((fr - center).magnitude() > radius) radius = (fr - center).magnitude();
	if ((bl - center).magnitude() > radius) radius = (bl - center).magnitude();
	if ((br - center).magnitude() > radius) radius = (br - center).magnitude();

	// Assign corners
	points[0][0] = fl;
	points[0][ARR_SIZE] = fr;
	points[ARR_SIZE][ARR_SIZE] = br;
	points[ARR_SIZE][0] = bl;

	// Calculate intermediate front and back points TODO Dependant on ARR_SIZE = 4
	temp = fr - fl;
	if (temp.magnitude() < 0.0001) {
		points[0][1] = fl;
		points[0][2] = fl;
	} else {
		points[0][1] = fl + temp.normalize() * (temp.magnitude() / 3.0);
		points[0][2] = fl + temp.normalize() * (2.0 * temp.Magnitude() / 3.0);
	}

	temp = br - bl;
	if (temp.magnitude() < 0.0001) {
		points[3][1] = bl;
		points[3][2] = bl;
	} else {
		points[3][1] = bl + temp.normalize() * (temp.magnitude() / 3.0);
		points[3][2] = bl + temp.normalize() * (2.0 * temp.magnitude() / 3.0);
	}

	// Calculate intermediate left and right points
	int i;
	for (i = 0; i < ARR_SIZE; ++i) {
		temp = points[3][i] - points[0][i];
		if (temp.Magnitude() > 0.0001) {
			points[1][i] = points[0][i] + temp.normalize() * (temp.magnitude() / 3.0);
			points[2][i] = points[0][i] + temp.normalize() * (2.0 * temp.magnitude() / 3.0);
		} else {
			points[1][i] = points[0][i];
			points[2][i] = points[0][i];
		}
	}
}

void Bezier::attach(Bezier& other, bool reverse) {
	nextPatch = &other;

	// Calculate road radius as the connection of this and the next patch
	MathVector<float, 3> a = surfCoord(0.5, 0.0), b = surfCoord(0.5, 1.0), c = other.surfCoord(0.5, 1.0);
	if (reverse) { a = surfCoord(0.5, 1.0); b = surfCoord(0.5, 0.0); c = other.surfCoord(0.5, 0.0); }

	MathVector<float, 3> d1 = a - b, d2 = c - b;
	float diff = d2.magnitude() - d1.magnitude();
	double dd = ((d1.magnitude() < 0.0001) || (d2.magnitude() < 0.0001)) ? 0.0 : d1.normalize().dot(d2.normalize());
	float angle = acos((dd >= 1.0L) ? 1.0L :(dd <= -1.0L) ? -1.0L : dd);
	float d1d2mag = d1.magnitude() + d2.magnitude();
	float alpha = (d1d2mag < 0.0001) ? 0.0f : (M_PI * diff + 2.0 * d1.magnitude() * angle) / d1d2mag / 2.0;

	if (fabs(alpha - M_PI/2.0) < 0.001) roadRadius = 10000.0;
	else roadRadius = d1.magnitude() / 2.0 / cos(alpha);

	if (d1.magnitude() < 0.0001) roadCurvature = 0.0;
	else roadCurvature = 2.0 * cos(alpha) / d1.magnitude();

	// Determine it's a left or right turn at the connection
	MathVector<float, 3> d = d1.cross(d2);
	if (fabs(d[0]) < 0.1 && fabs(d[1]) < 0.1 && fabs(d[2]) < 0.1) turn = 0; // Straight road ahead
	else if (d[1] > 0.0) turn = -1; // Left turn ahead
	else turn = 1; // Right turn ahead

	// Calculate distance from start of the road
	if (other.next_patch == NULL || reverse) other.dist_from_start = dist_from_start + d1.Magnitude();
	length = d1.Magnitude();
}

bool Bezier::collideSubDivQuadSimple(const MathVector<float, 3>& origin, const MathVector<float, 3>& direction,
								 	 MathVector<float, 3>& outtri) const {
	MathVector<float, 3> normal;
	return collideSubDivQuadSimpleNorm(origin, direction, outtri, normal);
}

bool Bezier:: collideSubDivQuadSimpleNorm(const MathVector<float, 3>& origin, const MathVector<float, 3>& direction,
										  MathVector<float, 3>&outtri, MathVector<float, 3>& normal) const {
	bool col = false;
	const int COLLISION_QUAD_DIVS = 6;
	const bool QUAD_DIV_FAST_DISCARD = true;

	float t, u, v;

	float su = 0;
	float sv = 0;

	float umin = 0;
	float umax = 1;
	float vmin = 0;
	float vmax = 1;

	MathVector<float, 3> ul = points[ARR_SIZE-1][ARR_SIZE-1];
	MathVector<float, 3> ur = points[ARR_SIZE-1][0];
	MathVector<float, 3> br = points[0][0];
	MathVector<float, 3> bl = points[0][ARR_SIZE-1];

	bool loop = true;

	float areacut = 0.5;

	for (int i = 0; i < COLLISION_QUAD_DIVS && loop; ++i) {
		float tu[2];
		float tv[2];

		tu[0] = umin;
		if (tu[0] < 0) tu[0] = 0;
		tu[1] = umax;
		if (tu[1] > 1) tu[1] = 1;

		tv[0] = vmin;
		if (tv[0] < 0) tv[0] = 0;
		tv[1] = vmax;
		if (tv[1] > 1) tv[1] = 1;

		ul = surfCoord(tu[0], tv[0]);
		ur = surfCoord(tu[1], tv[0]);
		br = surfCoord(tu[1], tv[1]);
		bl = surfCoord(tu[0], tv[1]);

		col = intersectsQuadrilateral(origin, direction, ul, ur, br, bl, t, u, v);

		if (col) {
			// Expand quad UV to surface UV
			su = u * (tu[1] - tu[0]) + tu[0];
			sv = v * (tv[1] - tv[0]) + tv[0];

			// Place max and min according to area hit
			vmax = sv + (0.5 * areacut) * (vmax - vmin);
			vmin = sv - (0.5 * areacut) * (vmax - vmin);
			umax = su + (0.5 * areacut) * (umax - umin);
			umin = su - (0.5 * areacut) * (umax - umin);
		} else {
			if ((i == 0) && QUAD_DIV_FAST_DISCARD) {
				outtri = origin;
				return false;
			} else {
				loop = false;
			}
		}
	}

	if (col) {
		outtri = surfCoord(su, sv);
		normal = surfNorm(su, sv);
		return true;
	} else {
		outtri = origin;
		return false;
	}
}

void Bezier::reverse() {
	MathVector<float, 3> oldpoints[ARR_SIZE][ARR_SIZE];

	for (int n = 0; n < ARR_SIZE; ++n)
		for (int i = 0; i < ARR_SIZE; ++i)
			oldpoints[n][i] = points[n][i];

	for (int n = 0; n < ARR_SIZE; ++n)
		for (int i = 0; i < ARR_SIZE; ++i)
			points[n][i] = oldpoints[ARR_SIZE-1-n][ARR_SIZE-1-i];
}

bool Bezier::checkForProblems() const {
	MathVector<float,3> corners[ARR_SIZE];
	corners[0] = points[0][0];
	corners[1] = points[0][3];
	corners[2] = points[3][3];
	corners[3] = points[3][0];

	bool problem = false;

	for (int i = 0; i < ARR_SIZE; ++i) {
		MathVector<float, 3> leg1(corners[(i+1)%4] - corners[i]);
		MathVector<float, 3> leg2(corners[(i+2)%4] - corners[i]);
		MathVector<float, 3> leg3(corners[(i+3)%4] - corners[i]);

		MathVector<float, 3> dir1 = leg1.cross(leg2);
		MathVector<float, 3> dir2 = leg1.cross(leg3);
		MathVector<float, 3> dir3 = leg2.cross(leg3);

		if (dir1.dot(dir2) < -0.0001) problem = true;
		if (dir1.dot(dir3) < -0.0001) problem = true;
		if (dir3.dot(dir2) < -0.0001) problem = true;
	}

	return problem;
}

void Bezier::deCasteljauHalveCurve(MathVector<float, 3>* points4, MathVector<float,3>* left4,
								   MathVector<float, 3>* right4) const {
	left4[0] = points4[0];
	left4[1] = (points4[0] + points4[1]) * 0.5;
	MathVector<float, 3> point23 = (points4[1] + points4[2]) * 0.5;
	left4[2] = (left4[1] + point23) * 0.5;

	right4[3] = points4[3];
	right4[2] = (points4[3] + points4[2]) * 0.5;
	right4[1] = (right4[2] + point23) * 0.5;

	left4[3] = right4[0] = (right4[1] + left4[2]) * 0.5;
}

AABB<float> Bezier::getAABB() const {
	float maxv[3];
	float minv[3];
	bool havevals[6];
	for (int n = 0; n < 6; ++n)
		havevals[n] = false;

	for (int x = 0; x < ARR_SIZE; ++x) {
		for (int y = 0; y < ARR_SIZE; ++y) {
			MATHVECTOR<float,3> temp(points[x][y]);

			// Cache for bounding box stuff
			for (int n = 0; n < 3; ++n) {
				if (!havevals[n]) {
					maxv[n] = temp[n];
					havevals[n] = true;
				} else if (temp[n] > maxv[n])
					maxv[n] = temp[n];

				if (!havevals[n+3]) {
					minv[n] = temp[n];
					havevals[n+3] = true;
				} else if (temp[n] < minv[n])
					minv[n] = temp[n];
			}
		}
	}

	MathVector<float, 3>bboxmin(minv[0], minv[1], minv[2]);
	MathVector<float, 3>bboxmax(maxv[0], maxv[1], maxv[2]);

	AABB <float> box;
	box.setFromCorners(bboxmin, bboxmax);
	return box;
}
