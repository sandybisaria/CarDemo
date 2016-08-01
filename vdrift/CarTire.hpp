#pragma once

#include "CarWheel.hpp"
#include "MathVector.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <string>
#include <vector>

#define _USE_MATH_DEFINES

// Stuntrally's CARTIRE class
class CarTire {
public:
	CarTire() {
		longitudinal.resize(11, 0.0);
		lateral.resize(15, 0.0);
		aligning.resize(18, 0.0);
	}

	void findSigmaHatAlphaHat(double load, double& outputSigmaHat, double& outputAlphaHat, int iterations = 400) const;
	void lookUpSigmaHatAlphaHat(double normForce, double& sh, double& ah) const;
	void calculateSigmaHatAlphaHat(size_t tableSize = 20);

	MathVector<double, 3> getForce(double normForce, double fricCoeff,
								   const MathVector<double, 3>& hubVel, double patchSpeed,
								   double currCamber, CarWheel::SlideSlip* slips) const;

	double getMaximumFx(double load) const;
	double getMaximumFy(double load, double currCamber) const;
	double getMaximumMz(double load, double currCamber) const;

	double PacejkaFx(double sigma, double fz, double fricCoeff, double& maxForceOutput) const;
	double PacejkaFy(double alpha, double fz, double gamma, double fricCoeff,
					 double& maxForceOutput) const;
	double PacejkaMz(double alpha, double fz, double gamma, double fricCoeff,
					 double& maxForceOutput) const;
	double getOptimumSteeringAngle(double load) const;

	static CarTire* none(); // Default CarTire that "won't crash"

//---- Constants
	std::vector<double> longitudinal; // Params of longitudinal Pacejka equation; series b
	std::vector<double> lateral; // Params of lateral Pacejka equation; series a
	std::vector<double> aligning; // Params of aligning moment Pacejka equation; series c

	std::vector<double> sigmaHat; // Max grip in longitudinal direction
	std::vector<double> alphaHat; // Max grip in lateral direction

	std::string name; // Name of .tire file (where params are loaded from)
};
