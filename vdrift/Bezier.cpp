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
		points[0][1] = fl + temp.normalized() * (temp.magnitude() / 3.0);
		points[0][2] = fl + temp.normalized() * (2.0 * temp.magnitude() / 3.0);
	}

	temp = br - bl;
	if (temp.magnitude() < 0.0001) {
		points[3][1] = bl;
		points[3][2] = bl;
	} else {
		points[3][1] = bl + temp.normalized() * (temp.magnitude() / 3.0);
		points[3][2] = bl + temp.normalized() * (2.0 * temp.magnitude() / 3.0);
	}

	// Calculate intermediate left and right points
	int i;
	for (i = 0; i < ARR_SIZE; ++i) {
		temp = points[3][i] - points[0][i];
		if (temp.magnitude() > 0.0001) {
			points[1][i] = points[0][i] + temp.normalized() * (temp.magnitude() / 3.0);
			points[2][i] = points[0][i] + temp.normalized() * (2.0 * temp.magnitude() / 3.0);
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
	double dd = ((d1.magnitude() < 0.0001) || (d2.magnitude() < 0.0001)) ? 0.0 : d1.normalized().dot(d2.normalized());
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
	if (other.nextPatch == NULL || reverse) other.distFromStart = distFromStart + d1.magnitude();
	length = d1.magnitude();
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
			MathVector<float,3> temp(points[x][y]);

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

MathVector<float, 3> Bezier::bernstein(float u, MathVector<float, 3>* p) const {
	float oneminusu(1.0f - u);

	MathVector<float, 3> a = p[0] * (u * u *u);
	MathVector<float, 3> b = p[1] * (3 * u * u * oneminusu);
	MathVector<float, 3> c = p[2] * (3 * u * oneminusu * oneminusu);
	MathVector<float, 3> d = p[3] * (oneminusu * oneminusu * oneminusu);

	return MathVector<float, 3>(a[0] + b[0] + c[0] + d[0],
								a[1] + b[1] + c[1] + d[1],
								a[2] + b[2] + c[2] + d[2]);
}

MathVector<float, 3> Bezier::bernsteinTangent(float u, MathVector<float, 3> *p) const {
	MathVector<float, 3> a = (p[1] - p[0]) * (3 * pow(u, 2));
	MathVector<float, 3> b = (p[2] - p[1]) * (3 * 2 * u * (1 -u ));
	MathVector<float, 3> c = (p[3] - p[2]) * (3 * pow((1 - u), 2));

	return a + b + c;
}

MathVector<float, 3> Bezier::surfCoord(float px, float py) const {
	MathVector<float, 3> temp[ARR_SIZE];
	MathVector<float, 3> temp2[ARR_SIZE];
	int i, j;

	// Get splines along x axis
	for (j = 0; j < ARR_SIZE; j++) {
		for (i = 0; i < ARR_SIZE; ++i) {
			temp2[i] = points[j][i];
		}
		temp[j] = bernstein(px, temp2);
	}

	return bernstein(py, temp);
}

MathVector<float, 3> Bezier::surfNorm(float px, float py) const {
	MathVector<float, 3> temp[ARR_SIZE];
	MathVector<float, 3> temp2[ARR_SIZE];
	MathVector<float, 3> tempx[ARR_SIZE];

	// Get splines along x axis
	for (int j = 0; j < ARR_SIZE; j++) {
		for (int i = 0; i < ARR_SIZE; ++i) {
			temp2[i] = points[j][i];
		}
		temp[j] = bernstein(px, temp2);
	}

	// Get splines along y axis
	for (int j = 0; j < ARR_SIZE; j++)
	{
		for (int i = 0; i < ARR_SIZE; ++i) {
			temp2[i] = points[i][j];
		}
		tempx[j] = bernstein(py, temp2);
	}

	return -(bernsteinTangent(px, tempx).cross(bernsteinTangent(py, temp)).normalized());
}

bool Bezier::intersectsQuadrilateral(const MathVector<float, 3>& orig, const MathVector<float, 3>& dir,
									 const MathVector<float, 3>& v_00, const MathVector<float, 3>& v_10,
									 const MathVector<float, 3>& v_11, const MathVector<float, 3>& v_01,
									 float &t, float &u, float &v) const {
	const float EPSILON = 0.000001;

	// Reject rays that are parallel to Q, and rays that intersect the plane
	// of Q either on the left of the line V00V01 or below the line V00V10.
	MathVector<float, 3> E_01 = v_10 - v_00;
	MathVector<float, 3> E_03 = v_01 - v_00;
	MathVector<float, 3> P = dir.cross(E_03);
	float det = E_01.dot(P);

	if (std::abs(det) < EPSILON) return false;

	MathVector<float, 3> T = orig - v_00;
	float alpha = T.dot(P) / det;

	if (alpha < 0.0) return false;

	MathVector<float, 3> Q = T.cross(E_01);
	float beta = dir.dot(Q) / det;

	if (beta < 0.0) return false;

	if (alpha + beta > 1.0) {
		// Reject rays that that intersect the plane of Q either on
		// the right of the line V11V10 or above the line V11V00.
		MathVector<float, 3> E_23 = v_01 - v_11;
		MathVector<float, 3> E_21 = v_10 - v_11;
		MathVector<float, 3> P_prime = dir.cross(E_21);
		float det_prime = E_23.dot(P_prime);

		if (std::abs(det_prime) < EPSILON) return false;

		MathVector<float, 3> T_prime = orig - v_11;
		float alpha_prime = T_prime.dot(P_prime) / det_prime;

		if (alpha_prime < 0.0) return false;

		MathVector<float, 3> Q_prime = T_prime.cross(E_23);
		float beta_prime = dir.dot(Q_prime) / det_prime;

		if (beta_prime < 0.0) return false;
	}

	// Compute the ray parameter of the intersection point, and
	// reject the ray if it does not hit Q.
	t = E_03.dot(Q) / det;

	if (t < 0.0) return false;

	// Compute the barycentric coordinates of the fourth vertex.
	// These do not depend on the ray, and can be precomputed
	// and stored with the quadrilateral.
	float alpha_11, beta_11;
	MathVector<float, 3> E_02 = v_11 - v_00;
	MathVector<float, 3> n = E_01.cross(E_03);

	if ((std::abs(n[0]) >= std::abs(n[1])) && (std::abs(n[0]) >= std::abs(n[2]))) {
		alpha_11 = ((E_02[1] * E_03[2]) - (E_02[2] * E_03[1])) / n[0];
		beta_11 = ((E_01[1] * E_02[2]) - (E_01[2]  * E_02[1])) / n[0];
	} else if ((std::abs(n[1]) >= std::abs(n[0])) && (std::abs(n[1]) >= std::abs(n[2]))) {
		alpha_11 = ((E_02[2] * E_03[0]) - (E_02[0] * E_03[2])) / n[1];
		beta_11 = ((E_01[2] * E_02[0]) - (E_01[0]  * E_02[2])) / n[1];
	} else {
		alpha_11 = ((E_02[0] * E_03[1]) - (E_02[1] * E_03[0])) / n[2];
		beta_11 = ((E_01[0] * E_02[1]) - (E_01[1]  * E_02[0])) / n[2];
	}

	// Compute the bilinear coordinates of the intersection point.
	if (std::abs(alpha_11 - (1.0)) < EPSILON) {
		// Q is a trapezium.
		u = alpha;
		if (std::abs(beta_11 - (1.0)) < EPSILON) v = beta; // Q is a parallelogram.
		else v = beta / ((u * (beta_11 - (1.0))) + (1.0)); // Q is a trapezium.
	} else if (std::abs(beta_11 - (1.0)) < EPSILON) {
		// Q is a trapezium.
		v = beta;
		if ( ((v * (alpha_11 - (1.0))) + (1.0)) == 0 ) return false;
		u = alpha / ((v * (alpha_11 - (1.0))) + (1.0));
	} else {
		float A = (1.0) - beta_11;
		float B = (alpha * (beta_11 - (1.0)))
				- (beta * (alpha_11 - (1.0))) - (1.0);
		float C = alpha;
		float D = (B * B) - ((4.0) * A * C);
		if (D < 0) return false;
		float Q = (-0.5) * (B + ((B < (0.0) ? (-1.0) : (1.0))
				* std::sqrt(D)));
		u = Q / A;
		if ((u < (0.0)) || (u > (1.0))) u = C / Q;
		v = beta / ((u * (beta_11 - (1.0))) + (1.0));
	}

	return true;
}

std::ostream& operator<<(std::ostream& os, const Bezier& b) {
	os << "====" << std::endl;
	for (int x = 0; x < Bezier::ARR_SIZE; x++) {
		for (int y = 0; y < Bezier::ARR_SIZE; y++) {
			os << b[y*Bezier::ARR_SIZE+x] << std::endl;
		}
		os << "----" << std::endl;
	}
	os << "====" << std::endl;
	return os;
}

#ifdef COMPILE_UNIT_TESTS
#include <gtest/gtest.h>

TEST(Bezier, BezierFunctions) {
	MathVector<float, 3> p[4], l[4], r[4];
	p[0].set(-1,0,0); p[1].set(-1,1,0); p[2].set(1,1,0); p[3].set(1,0,0);

	Bezier b; b.deCasteljauHalveCurve(p, l, r);

	EXPECT_EQ(l[0],(MathVector<float, 3>(-1, 0, 0)));
	EXPECT_EQ(l[1],(MathVector<float, 3>(-1, 0.5, 0)));
	EXPECT_EQ(l[2],(MathVector<float, 3>(-0.5, 0.75, 0)));
	EXPECT_EQ(l[3],(MathVector<float, 3>(0, 0.75, 0)));

	EXPECT_EQ(r[3],(MathVector<float, 3>(1, 0, 0)));
	EXPECT_EQ(r[2],(MathVector<float, 3>(1, 0.5, 0)));
	EXPECT_EQ(r[1],(MathVector<float, 3>(0.5, 0.75, 0)));
	EXPECT_EQ(r[0],(MathVector<float, 3>(0, 0.75, 0)));

	//TODO Commented due to "stack smashing" error
//	b.setFromCorners(MathVector<float, 3>(1, 0, 1),
//					 MathVector<float, 3>(-1, 0, 1),
//					 MathVector<float, 3>(1, 0, -1),
//					 MathVector<float, 3>(-1,0,-1));
//	EXPECT_FALSE(b.checkForProblems());
}

#endif
