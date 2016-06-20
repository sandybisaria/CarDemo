#pragma once

#include "MathVector.hpp"

#include <vector>

// Loaded from a .tire file
class CarTire {
public:
	CarTire() {
		longitudinal.resize(11, 0.0);
		lateral.resize(15, 0.0);
		aligning.resize(18, 0.0);
		user = 0;
	}

	void lookUpSigmaHatAlphaHat(double normForce, double& sh, double& ah) const;
	void calculateSigmaHatAlphaHat(int tableSize = 20);

	MathVector<double, 3> getForce(double normForce, double fricCoeff,
								   const MathVector<double, 3>& hubVel, double patchSpeed,
								   double currCamber) const;

	double getMaximumFx(double load) const;
	double getMaximumFy(double load, double currCamber) const;
	double getMaximumMz(double load, double currCamber) const;

	double PacejkaFx(double sigma, double fz, double fricCoeff, double& maxForceOutput) const;
	double PacejkaFy(double alpha, double fz, double gamma, double fricCoeff, double& maxForceOutput) const;
	double PacejkaMz(double sigma, double alpha, double fz, double gamma, double fricCoeff, double& maxForceOutput) const;
	double getOptimumSteeringAngle(double load) const;

	static CarTire* none(); // Default CarTire that "won't crash"

private:
	void findSigmaHatAlphaHat(double load, double& outputSigmaHat, double& outputAlphaHat, int iterations = 400);

	// Constants
	std::vector<double> longitudinal; // Params of longitudinal Pacejka equation; series b
	std::vector<double> lateral; // Params of lateral Pacejka equation; series a
	std::vector<double> aligning; // Params of aligning moment Pacejka equation; series c

	std::vector<double> sigmaHat; // Max grip in longitudinal direction
	std::vector<double> alphaHat; // Max grip in lateral direction

	std::string name; // Name of .tire file
	int user; // 1 = user, 0 = orig file?
};
