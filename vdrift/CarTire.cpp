#include "CarTire.hpp"

#include <iostream>

void CarTire::findSigmaHatAlphaHat(double load, double& outputSigmaHat, double& outputAlphaHat, int iterations) {
	double x, y, yMax = 0, junk, x4 = 4.0 / iterations, x40 = 40.0 / iterations;
	for (x = -2; x < 2; x += x4) {
		y = PacejkaFx(x, load, 1.0, junk);
		if (y > yMax) {
			outputSigmaHat = x;
			yMax = y;
		}
	}
	yMax = 0;
	for (x = -20; x < 20; x += x40) {
		y = PacejkaFy(x, load, 0, 1.0, junk);
		if (y > yMax) {
			outputAlphaHat = x;
			yMax = y;
		}
	}
}

void CarTire::lookUpSigmaHatAlphaHat(double normForce, double& sh, double& ah) const {
	assert(!sigmaHat.empty());
	assert(!alphaHat.empty());
	assert(sigmaHat.size() == alphaHat.size());

	const int HAT_ITERATIONS = sigmaHat.size();
	const double HAT_LOAD = 0.5;
	double nf = normForce * 0.001;
	if (nf < HAT_LOAD) {
		sh = sigmaHat[0];
		ah = alphaHat[0];
	} else if (nf >= HAT_LOAD * HAT_ITERATIONS) {
		sh = sigmaHat[HAT_ITERATIONS - 1];
		ah = alphaHat[HAT_ITERATIONS - 1];
	} else {
		int lBound;
		double blend;
		lBound = (int) (nf / HAT_LOAD);
		lBound--;
		lBound = std::min(std::max(0, lBound), (int)sigmaHat.size() - 2); //FIXME Band-aid fix nf being nan

		blend = (nf - HAT_LOAD * (lBound + 1)) / HAT_LOAD;
		sh = sigmaHat.at(lBound) * (1.0 - blend) + sigmaHat.at(lBound+1) * blend;
		sh = alphaHat.at(lBound) * (1.0 - blend) + alphaHat.at(lBound+1) * blend;
	}
}

MathVector<double, 3> CarTire::getForce(double normForce, double fricCoeff,
							   const MathVector<double, 3>& hubVel, double patchSpeed,
							   double currCamber, CarWheel::SlideSlip* slips) const {
	assert(fricCoeff > 0);

	double sigHat(0), alHat(0);
	lookUpSigmaHatAlphaHat(normForce, sigHat, alHat);

	double fz = normForce * 0.001;
	fz = std::min(fz, 30.); // Cap to "prevent explosions"

	const double EPSILON = 1e-6;
	if (fz < EPSILON) {
		MathVector<double, 3> zero(0);
		return zero;
	}

	double sigma(0.0), tanAlpha(0.0), alpha(0.0);
	double v = hubVel[0];
	double denom = std::max(std::abs(v), 0.01);

	sigma = (patchSpeed - v) / denom;
	tanAlpha = hubVel[1] / denom;
	alpha = -atan2(hubVel[1], denom) * 180.0 / M_PI;

	// Avoid crashes?
	if (isnan(alpha) || isnan(1.0f / sigHat)) {
		MathVector<double, 3> zero(0);
		return zero;
	}
	assert(!isnan(alpha));

	double gamma = currCamber * 180.0 / M_PI;

	// Beckman method for pre-combining longitudinal and lateral forces
	double s = sigma / sigHat;
	assert(!isnan(s));
	double a = alpha / alHat;
	assert(!isnan(a));

	double rho = std::max(sqrt(s*s + a*a), 0.0001); // Avoid division by zero
	assert(!isnan(rho));

	double maxFx(0), maxFy(0), maxMz(0);
	double fx = (s / rho) * PacejkaFx(rho * sigHat, fz, fricCoeff, maxFx);
	assert(!isnan(fx));
	double fy = (a / rho) * PacejkaFy(rho * alHat, fz, gamma, fricCoeff, maxFy);
	assert(!isnan(fy));
	double mz = PacejkaMz(sigma, alpha, fz, gamma, fricCoeff, maxMz);

	if (slips) {
		slips->preFx = fx;
		slips->preFy = fy;
	}

	// Combining Method 1: Traction Circle
	float longFactor = 1.0;
	float combForce = std::abs(fx) + std::abs(fy);
	if (combForce > 1) // Avoid division by zero
		longFactor = std::abs(fx) / combForce; // 1.0 when fy = 0; 0.0 when fx = 0
	float maxForce = std::abs(maxFx) * longFactor + (1.0 - longFactor) * std::abs(maxFy); // Linear interp
	if (combForce > maxForce) { // Cap forces
		// Scale down forces to fit into the max
		double sc = maxForce / combForce;

		fx *= sc;
		assert(!isnan(fx));
		maxFx *= sc;

		fy *= sc;
		assert(!isnan(fy));
		maxFy *= sc;
	}

	if (slips) {
		slips->slide = sigma; slips->slideRatio = s;
		slips->slip = alpha; slips->slipRatio = a;
		slips->fxSr = s / rho; slips->fxRsr = rho * sigHat;
		slips->fyAr = a / rho; slips->fyRar = rho * alHat;
		slips->fx = fx; slips->fxm = maxFx;
		slips->fy = fy; slips->fym = maxFy;
		slips->fz = fz;
	}

	MathVector<double, 3> outVec(fx, fy, mz);
	return outVec;
}


void CarTire::calculateSigmaHatAlphaHat(int tableSize) {
	const int HAT_LOAD = 0.5;
	sigmaHat.resize(tableSize, 0);
	alphaHat.resize(tableSize, 0);
	for (int i = 0; i < tableSize; i++) {
		findSigmaHatAlphaHat((double)(i + 1) * HAT_LOAD, sigmaHat[i], alphaHat[i]);
	}
}

// Load is the normal force in newtons
double CarTire::getMaximumFx(double load) const {
	const std::vector<double>& b = longitudinal;
	double fz = load * 0.001;

	return (b[1] * fz + b[2]) * fz;
}
double CarTire::getMaximumFy(double load, double currCamber) const {
	const std::vector<double>& a = lateral;
	double fz = load * 0.001;
	double gamma = currCamber * 180.0 / M_PI;

	double d = (a[1] * fz + a[2]) * fz;
	double sv = ((a[11] * fz + a[12]) * gamma + a[13]) * fz + a[14];

	return d + sv;
}
double CarTire::getMaximumMz(double load, double currCamber) const {
	const std::vector<double>& c = aligning;
	double fz = load * 0.001;
	double gamma = currCamber * 180.0 / M_PI;

	double d = (c[1] * fz + c[2]) * fz;
	double sv = (c[14] * fz * fz + c[15] * fz) * gamma + c[16] * fz + c[17];

	return -d - sv;
}


double CarTire::PacejkaFx(double sigma, double fz, double fricCoeff, double& maxForceOutput) const {
	const std::vector<double>& b = longitudinal;

	double d = (b[1]*fz + b[2]) *fz *fricCoeff;
	assert(b[0]* (b[1]*fz + b[2]) != 0);
	double B = ( b[3]*fz+b[4] ) *exp ( -b[5]*fz ) / ( b[0]* ( b[1]*fz+b[2] ) );
	double e = ( b[6]*fz*fz+b[7]*fz+b[8] );
	double s = ( 100*sigma + b[9]*fz+b[10] );
	double fx = d*sin ( b[0] * atan ( s*B+e* ( atan ( s*B )-s*B ) ) );

	maxForceOutput = d;

	assert(!isnan(fx));
	return fx;
}
double CarTire::PacejkaFy(double alpha, double fz, double gamma, double fricCoeff, double& maxForceOutput) const {
	const std::vector<double>& a = lateral;

	double D = ( a[1]*fz+a[2] ) *fz*fricCoeff;
	double B = a[3]*sin ( 2.0*atan ( fz/a[4] ) ) * ( 1.0-a[5]*std::abs ( gamma ) ) / ( a[0]* ( a[1]*fz+a[2] ) *fz );
	assert(!isnan(B));
	double E = a[6]*fz+a[7];
	double S = alpha + a[8]*gamma+a[9]*fz+a[10];
	double Sv = ( ( a[11]*fz+a[12] ) *gamma + a[13] ) *fz+a[14];
	double Fy = D*sin ( a[0]*atan ( S*B+E* ( atan ( S*B )-S*B ) ) ) +Sv;

	maxForceOutput = D+Sv;

	assert(!isnan(Fy));
	return Fy;
}
double CarTire::PacejkaMz(double sigma, double alpha, double fz, double gamma, double fricCoeff,
						  double& maxForceOutput) const {
	const std::vector<double>& c = aligning;

	double D = ( c[1]*fz+c[2] ) *fz*fricCoeff;
	double B = ( c[3]*fz*fz+c[4]*fz ) * ( 1.0-c[6]*std::abs ( gamma ) ) *exp ( -c[5]*fz ) / ( c[0]*D );
	double E = ( c[7]*fz*fz+c[8]*fz+c[9] ) * ( 1.0-c[10]*std::abs ( gamma ) );
	double S = alpha + c[11]*gamma+c[12]*fz+c[13];
	double Sv = ( c[14]*fz*fz+c[15]*fz ) *gamma+c[16]*fz + c[17];
	double Mz = D*sin ( c[0]*atan ( S*B+E* ( atan ( S*B )-S*B ) ) ) +Sv;

	maxForceOutput = D+Sv;

	assert(!isnan(Mz));
	return Mz;
}
double CarTire::getOptimumSteeringAngle(double load) const {
	double sigHat(0), alHat(0);
	lookUpSigmaHatAlphaHat(load, sigHat, alHat);
	return alHat;
}

CarTire* CarTire::none() {
	static CarTire s;
	static bool init = true;
	if (init) {
		init = false;
		int i=0;
		s.lateral[i++] = 1.61;
		s.lateral[i++] = -0;
		s.lateral[i++] = 2775;
		s.lateral[i++] = 2220;
		s.lateral[i++] = 19.6;
		s.lateral[i++] = 0.013;
		s.lateral[i++] = -0.14;
		s.lateral[i++] = 0.14;
		s.lateral[i++] = 0.019;
		s.lateral[i++] = -0.019;
		s.lateral[i++] = -0.18;
		s.lateral[i++] = 0;
		s.lateral[i++] = 0;
		s.lateral[i++] = 0;
		s.lateral[i++] = 0;
		i = 0;
		s.longitudinal[i++] = 1.73;
		s.longitudinal[i++] = -0.49;
		s.longitudinal[i++] = 3439;
		s.longitudinal[i++] = 279;
		s.longitudinal[i++] = 470;
		s.longitudinal[i++] = 0;
		s.longitudinal[i++] = 0.0008;
		s.longitudinal[i++] = 0.005;
		s.longitudinal[i++] = -0.024;
		s.longitudinal[i++] = 0;
		s.longitudinal[i++] = 0;
		i = 0;
		s.aligning[i++] = 2.10;
		s.aligning[i++] = -3.9;
		s.aligning[i++] = -3.9;
		s.aligning[i++] = -1.26;
		s.aligning[i++] = -8.20;
		s.aligning[i++] = 0.025;
		s.aligning[i++] = 0;
		s.aligning[i++] = 0.044;
		s.aligning[i++] = -0.58;
		s.aligning[i++] = 0.18;
		s.aligning[i++] = 0.043;
		s.aligning[i++] = 0.048;
		s.aligning[i++] = -0.0035;
		s.aligning[i++] = -0.18;
		s.aligning[i++] = 0.14;
		s.aligning[i++] = -1.029;
		s.aligning[i++] = 0.27;
		s.aligning[i++] = -1.1;
		s.name = "None";
		s.user = 0;

		s.calculateSigmaHatAlphaHat();
	}
	return &s;

}
