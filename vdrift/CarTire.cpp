#include "CarTire.hpp"

#include <iostream>

void CarTire::findSigmaHatAlphaHat(double load, double& outputSigmaHat, double& outputAlphaHat, int iterations) {
	double x, y, ymax, junk, x4 = 4.0 / iterations, x40 = 40.0 / iterations;
	ymax = 0;
	for (x = -2; x < 2; x += x4) {
		y = PacejkaFx(x, load, 1.0, junk);
		if (y > ymax) {
			outputSigmaHat = x;
			ymax = y;
		}
	}
	ymax = 0;
	for (x = -20; x < 20; x += x40) {
		y = PacejkaFy(x, load, 0, 1.0, junk);
		if (y > ymax) {
			outputAlphaHat = x;
			ymax = y;
		}
	}
}

void CarTire::lookUpSigmaHatAlphaHat(double normalForce, double& sh, double& ah) const {
	assert(!sigmaHat.empty());
	assert(sigmaHat.size() == alphaHat.size());

	int HAT_ITERATIONS = sigmaHat.size();

	double HAT_LOAD = 0.5;
	double nf = normalForce * 0.001;
	if (nf < HAT_LOAD) {
		sh = sigmaHat[0];
		ah = sigmaHat[0];
	} else if (nf >= HAT_LOAD*HAT_ITERATIONS) {
		sh = sigmaHat[HAT_ITERATIONS-1];
		ah = sigmaHat[HAT_ITERATIONS-1];
	} else {
		int lbound;
		double blend;
		lbound = (int)(nf/HAT_LOAD);
		lbound--;
		if (lbound < 0) lbound = 0;
		blend = (nf-HAT_LOAD*(lbound+1))/HAT_LOAD;
		sh = sigmaHat[lbound]*(1.0-blend)+sigmaHat[lbound+1]*blend;
		ah = alphaHat[lbound]*(1.0-blend)+alphaHat[lbound+1]*blend;
	}
}

MathVector<double, 3> CarTire::getForce(double normalForce, double fricCoeff,
							   const MathVector<double, 3>& hubVelocity, double patchSpeed,
							   double currentCamber, CarWheel::SlideSlip* slips) const {
	assert(fricCoeff > 0);

	double sigmaHatValue(0);
	double alphaHatValue(0);

	lookUpSigmaHatAlphaHat(normalForce, sigmaHatValue, alphaHatValue);

	double Fz = normalForce * 0.001;

	// Cap Fz at a magic number to prevent explosions
	if (Fz > 30) Fz = 30;

	const double EPSILON = 1e-6;
	if (Fz < EPSILON) {
		MathVector<double,3> zero(0);
		return zero;
	}

	double sigma = 0.0;
	double tanAlpha = 0.0;
	double alpha = 0.0;

	double V = hubVelocity[0];
	double denom = std::max(std::abs(V), 0.01);

	sigma = (patchSpeed - V) / denom;
	tanAlpha = hubVelocity[1] / denom;
	alpha = -atan2(hubVelocity[1], denom) * 180.0/M_PI;

	// Avoid crashes
	if (isnan(alpha) || isnan(1.f/sigmaHatValue)) {
		MathVector<double,3> zero(0);
		return zero;
	}
	assert(!isnan(alpha));

	double gamma = currentCamber * 180.0/M_PI;

	// Beckman method for pre-combining longitudinal and lateral forces
	double s = sigma / sigmaHatValue;  assert(!isnan(s));
	double a = alpha / alphaHatValue;  assert(!isnan(a));

	double rho = std::max( sqrt( s*s+a*a ), 0.0001);  //avoid divide by zero
	assert(!isnan(rho));

	double maxFx(0), maxFy(0), maxMz(0);
	double Fx = (s / rho) * PacejkaFx( rho*sigmaHatValue, Fz,        fricCoeff, maxFx );  assert(!isnan(Fx));
	double Fy = (a / rho) * PacejkaFy( rho*alphaHatValue, Fz, gamma, fricCoeff, maxFy );  assert(!isnan(Fy));
	double Mz = PacejkaMz( sigma, alpha, Fz, gamma, fricCoeff, maxMz );

	if (slips) {
		slips->preFx = Fx;
		slips->preFy = Fy;
	}

	// Combining method 1: traction circle
	// Determine to what extent the tires are long (x) gripping vs lat (y) gripping
	float longfactor = 1.0;
	float combforce = std::abs(Fx)+std::abs(Fy);
	if (combforce > 1)  // Dvoid divide by zero (assume longfactor = 1 for this case)
		longfactor = std::abs(Fx)/combforce;  // 1.0 when Fy is zero, 0.0 when Fx is zero
	// Determine the maximum force for this amount of long vs lat grip
	float maxforce = std::abs(maxFx)*longfactor + (1.0-longfactor)*std::abs(maxFy); // inear interpolation
	if (combforce > maxforce) { // Cap forces
		// Scale down forces to fit into the maximum
		double sc = maxforce / combforce;
		Fx *= sc;  assert(!isnan(Fx));  maxFx *= sc;
		Fy *= sc;  assert(!isnan(Fy));	maxFy *= sc;
	}

	assert(!isnan(Fx));
	assert(!isnan(Fy));


	if (slips) {
		slips->slide = sigma;  slips->slideRatio = s;
		slips->slip  = alpha;  slips->slipRatio  = a;
		slips->fxSr = s / rho;  slips->fxRsr = rho*sigmaHatValue;
		slips->fyAr = a / rho;  slips->fyRar = rho*alphaHatValue;
		slips->frict = fricCoeff;
		slips->fx = Fx;  slips->fxm = maxFx;
		slips->fy = Fy;  slips->fym = maxFy;
		slips->fz = Fz;
	}

	MathVector<double, 3> outvec(Fx, Fy, Mz);
	return outvec;
}


void CarTire::calculateSigmaHatAlphaHat(int tableSize) {
	double HAT_LOAD = 0.5;
	sigmaHat.resize(tableSize, 0);
	alphaHat.resize(tableSize, 0);
	for (int i = 0; i < tableSize; i++) {
		findSigmaHatAlphaHat((double)(i + 1) * HAT_LOAD, sigmaHat[i], alphaHat[i]);
	}
}

// Load is the normal force in newtons
double CarTire::getMaximumFx(double load) const {
	const std::vector<double>& b = longitudinal;
	double Fz = load * 0.001;
	return (b[1]*Fz + b[2]) * Fz;
}

double CarTire::getMaximumFy(double load, double currentCamber) const {
	const std::vector<double>& a = lateral;
	double Fz = load * 0.001;
	double gamma = currentCamber * 180.0/M_PI;

	double D = (a[1]*Fz + a[2]) *Fz;
	double Sv = ((a[11]*Fz + a[12]) * gamma + a[13]) *Fz + a[14];

	return D+Sv;
}

double CarTire::getMaximumMz(double load, double currentCamber) const {
	const std::vector<double>& c = aligning;
	double Fz = load * 0.001;
	double gamma = currentCamber * 180.0/M_PI;

	double D = (c[1]*Fz + c[2]) *Fz;
	double Sv = (c[14]*Fz*Fz + c[15]*Fz) * gamma + c[16]*Fz + c[17];

	return -(D+Sv);
}

// Pacejka's magic formulas
// Longitudinal
double CarTire::PacejkaFx(double sigma, double Fz, double fricCoeff, double& maxForceOutput) const {
	const std::vector<double>& b = longitudinal;

	double D = (b[1]*Fz + b[2]) *Fz *fricCoeff;
	assert(b[0]* (b[1]*Fz + b[2]) != 0);
	double B = ( b[3]*Fz+b[4] ) *exp ( -b[5]*Fz ) / ( b[0]* ( b[1]*Fz+b[2] ) );
	double E = ( b[6]*Fz*Fz+b[7]*Fz+b[8] );
	double S = ( 100*sigma + b[9]*Fz+b[10] );
	double Fx = D*sin ( b[0] * atan ( S*B+E* ( atan ( S*B )-S*B ) ) );

	maxForceOutput = D;

	assert(!isnan(Fx));
	return Fx;
}

// Lateral
double CarTire::PacejkaFy(double alpha, double Fz, double gamma, double fricCoeff, double& maxForceOutput) const {
	const std::vector<double>& a = lateral;

	double D = ( a[1]*Fz+a[2] ) *Fz*fricCoeff;
	double B = a[3]*sin ( 2.0*atan ( Fz/a[4] ) ) * ( 1.0-a[5]*std::abs ( gamma ) ) / ( a[0]* ( a[1]*Fz+a[2] ) *Fz );
	assert(!isnan(B));
	double E = a[6]*Fz+a[7];
	double S = alpha + a[8]*gamma+a[9]*Fz+a[10];
	double Sv = ( ( a[11]*Fz+a[12] ) *gamma + a[13] ) *Fz+a[14];
	double Fy = D*sin ( a[0]*atan ( S*B+E* ( atan ( S*B )-S*B ) ) ) +Sv;

	maxForceOutput = D+Sv;

	assert(!isnan(Fy));
	return Fy;
}

// Aligning
double CarTire::PacejkaMz(double sigma, double alpha, double Fz, double gamma, double fricCoeff,
						  double& maxForceOutput) const {
	const std::vector<double>& c = aligning;

	double D = ( c[1]*Fz+c[2] ) *Fz*fricCoeff;
	double B = ( c[3]*Fz*Fz+c[4]*Fz ) * ( 1.0-c[6]*std::abs ( gamma ) ) *exp ( -c[5]*Fz ) / ( c[0]*D );
	double E = ( c[7]*Fz*Fz+c[8]*Fz+c[9] ) * ( 1.0-c[10]*std::abs ( gamma ) );
	double S = alpha + c[11]*gamma+c[12]*Fz+c[13];
	double Sv = ( c[14]*Fz*Fz+c[15]*Fz ) *gamma+c[16]*Fz + c[17];
	double Mz = D*sin ( c[0]*atan ( S*B+E* ( atan ( S*B )-S*B ) ) ) +Sv;

	maxForceOutput = D+Sv;

	assert(!isnan(Mz));
	return Mz;
}

// Optimum steering angle in degrees, given a load in newtons
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
