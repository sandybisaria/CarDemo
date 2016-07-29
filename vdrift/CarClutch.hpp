#pragma once

// Stuntrally's CARCLUTCH class
class CarClutch {
public:
	// Makes an S2000-like car
	CarClutch()
		: clutchMaxTorque(336.53), threshold(0.001), clutchPosition(0.0), locked(false),
		  lastTorque(0.0f), engineSpeed(0.0f), driveSpeed(0.0f) {}

	void setMaxTorque(const double& cmt) { clutchMaxTorque = cmt; }

	// 1.0 is fully engaged
	void setClutch(double cp) { clutchPosition = cp; }
	double getClutch() const { return clutchPosition; }

	// Clutch is modeled as a limited highly-viscous coupling
	double getTorque(double newEngineSpeed, double newDriveSpeed) {
		engineSpeed = newEngineSpeed;
		driveSpeed = newDriveSpeed;

		double newSpeedDiff = engineSpeed - driveSpeed;
		locked = true;

		double torqueCapacity = clutchMaxTorque; // Constant
		double maxTorque = clutchPosition * torqueCapacity;
		double frictionTorque = maxTorque * newSpeedDiff; // Viscous coupling (locked clutch)

		if (frictionTorque > maxTorque) {
			frictionTorque = maxTorque;
			locked = false;
		} else if (frictionTorque < -maxTorque) {
			frictionTorque = -maxTorque;
			locked = false;
		}

		double torque = frictionTorque;
		lastTorque = torque;
		return torque;
	}

	bool isLocked() const { return locked; }
	double getLastTorque() const { return lastTorque; }

private:
//---- Constants
	// The torque capacity (max transmitted torque) of the clutch is:
	// TC = sliding * radius * area * max-pressure.
	// Should be one-two times the max engine torque, typically 1.25
	double clutchMaxTorque;

	// Clutch pretends to be fully engaged when (engine speed - transmission speed)
	// is less than (threshold * normal-force)
	double threshold;

//---- Variables
	double clutchPosition;
	bool locked;

//---- For info only
	double lastTorque;
	double engineSpeed;
	double driveSpeed;
};
