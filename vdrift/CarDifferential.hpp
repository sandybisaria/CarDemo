#pragma once

#include "CarConstants.hpp"

// Stuntrally's CARDIFFERENTIAL class
class CarDifferential {
public:
	CarDifferential()
		: finalDrive(4.1), antiSlip(600.0), antiSlipTorque(0), antiSlipTorqueDecelFactor(0),
		  torqueSplit(0.5), side1Speed(0), side2Speed(0), side1Torque(0), side2Torque(0) {}

	void computeWheelTorques(double driveshaftTorque) {
		// Determine torque from anti-slip mechanism
		double cas = antiSlip;

		// If torque-sensitive
		if (antiSlipTorque > 0)	cas = antiSlipTorque * driveshaftTorque;
		//TODO Dev wanted to add minimum anti-slip

		// Determine deceleration behavior
		if (cas < 0) cas *= -antiSlipTorqueDecelFactor;

		cas = std::max(0.0, cas);
		double drag = clamp(cas * (side1Speed - side2Speed), -antiSlip, antiSlip);

		double torque = driveshaftTorque * finalDrive;
		side1Torque = torque * (1.0 - torqueSplit) - drag;
		side2Torque = torque * torqueSplit + drag;
	}

	void setAntiSlip(double as, double ast, double astdf) {
		antiSlip = as;
		antiSlipTorque = ast;
		antiSlipTorqueDecelFactor = astdf;
	}

	double calculateDriveshaftSpeed(double newSide1Speed, double newSide2Speed) {
		side1Speed = newSide1Speed;
		side2Speed = newSide2Speed;

		return getDriveshaftSpeed();
	}

	double getDriveshaftSpeed() const {
		return finalDrive * (side1Speed + side2Speed) * 0.5;
	}

	double getFinalDrive() const { return finalDrive; }
	void setFinalDrive(double fd) { finalDrive = fd; }

	const double& getSide1Speed()  const { return side1Speed; }
	const double& getSide2Speed()  const { return side2Speed; }
	const double& getSide1Torque() const { return side1Torque; }
	const double& getSide2Torque() const { return side2Torque; }

private:
//---- Constants
	// Gear ratio
	double finalDrive;

	// For modeling of speed-sensitive limited-slip differentials.
	// The maximum anti_slip torque that will be applied.
	// For speed-sensitive limited-slip differentials,
	// the anti-slip multiplier that's always applied.
	double antiSlip;

	// For modeling of speed-sensitive limited-slip differentials.
	// Anti-slip dependence on torque.
	double antiSlipTorque;

	// For modeling of speed-sensitive limited-slip differentials
	// that are 1.5 or 2-way. Set to 0.0 for 1-way LSD.
	double antiSlipTorqueDecelFactor;

	// For modeling of epicyclic differentials.
	// Ranges from 0.0 to 1.0, where 0.0 applies all torque to side1
	double torqueSplit;

//---- Variables
	// side1 is left/front, side2 is right/back
	double side1Speed, side2Speed;
	double side1Torque, side2Torque;
};
