#pragma once

#include "MathVector.hpp"

class CarAero {
public:
	// Default constructor = aerodynamically transparent device (i.e. no drag or lift)
	CarAero()
		: airDens(1.2), dragFrontArea(0), dragCoeff(0),
		  liftSurfArea(0), liftCoeff(0), liftEff(0) {}

	void set(const MathVector<double, 3>& np, double ndfa, double ndc,
			 double nlsa, double nlc, double nle) {
		pos = np;
		dragFrontArea = ndf;
		dragCoeff = ndc;
		liftSurfArea = nlsa;
		liftCoeff = nlc;
		liftEff = nle;
	}

	const MathVector<double, 3>& getPosition() const { return pos; }

	// If updStats, then liftVec and dragVec will be updated during calculations
	MathVector<double, 3> getForce(const MathVector<double, 3>& bodyspaceWindVec, bool updStats = true) const {
		MathVector<double, 3> dragV = bodyspaceWindVec * bodyspaceWindVec.magnitude() * 0.5 *
									  airDens * dragCoeff * dragFrontArea;

		double windSpeed = -bodyspaceWindVec[0]; // Positive if wind is heading at us
		if (windSpeed < 0)
			// Assume surface generates little lift in reverse
			windSpeed = -windSpeed * 0.2;
		const double k = 0.5 * airDens * windSpeed * windSpeed;
		const double lift = k * liftCoeff * liftSurfArea;
		const double drag = -liftCoeff * lift * (1.0 - liftEff);
		MathVector<double, 3> liftV(drag, 0, lift);

		MathVector<double, 3> force = dragV + liftV;
		if (updStats) {
			liftVec = liftV;
			dragVec = dragV;
		}
		return force;
	}

	double getAerodynamicDownforceCoefficient() const {
		return 0.5 * airDens * liftCoeff * liftSurfArea;
	}

	double getAerodynamicDragCoefficient() const {
		return 0.5 * airDens * (dragCoeff * dragFrontArea + liftCoeff * liftCoeff * liftSurfArea * (1.0 - liftEff));
	}

	MathVector<double, 3> getLiftVector() const { return liftVec; }
	MathVector<double, 3> getDragVector() const { return dragVec; }

private:
	double airDens; // Air density in kg/m^3

	double dragFrontArea; // Projected frontal area in m^2
	double dragCoeff; // Drag coefficient

	double liftSurfArea; // Wing surface area in m^2
	double liftCoeff; // Lift coefficient
	double liftEff; // Wing efficiency

	MathVector<double, 3> pos; // Position that drag/lift forces are applied on body

	// For info only
	mutable MathVector<double, 3> liftVec;
	mutable MathVector<double, 3> dragVec;
};
