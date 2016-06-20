#pragma once

class CarBrake {
public:
	// Makes an S2000-like car
	CarBrake()
		: fric(0.73), maxPress(4e6), radius(0.14), area(0.015),
		  bias(0.5), threshold(2e-4), handbrake(0), brakeFactor(0), handbrakeFactor(0),
		  locked(false), lastTorque(0) {}

	// 0.0 (no brakes applied) to 1.0 (brakes applied fully)
	void setBrakeFactor(double bf) { brakeFactor = bf; }
	void setHandbrakeFactor(double hf) { handbrakeFactor = hf; }

	// Magnitude of brake torque
	double getTorque() {
		double brake = brakeFactor > handbrake * handbrakeFactor ? brakeFactor : handbrake * handbrakeFactor;
		double pressure = brake * bias * maxPress;
		double normal = pressure * area;
		double torque = fric * normal * radius;

		lastTorque = torque;
		return torque;
	}

	// Used by autoclutch system
	bool willLock() const { return locked; }
	void setWillLock(double lock) { locked = lock; }

	void setFriction(const double& f) { fric = f; }
	double getFriction() { return fric; }

	void setMaxPressure(const double& mp) { maxPress = mp; }
	void setRadius(const double& r) { radius = r; }
	void setArea(const double& a) { area = a; }
	void setBias(const double& b) { bias = b; }

	bool getLocked() const { return locked; }
	double getBrakeFactor() const { return brakeFactor; }
	double getHandbrakeFactor() const { return handbrakeFactor; }
	void setHandbrake(const double& h) { handbrake = h; }

private:
	// Constants
	double fric; // Sliding coefficient of friction for brake pads on rotor
	double maxPress; // Max allowed pressure
	double radius; // Effective radius of rotor
	double area; // Area of brake pads
	double bias; // Fraction of pressure to be applied to brake
	double threshold; // Brake will lock when (linear brake velocity / normal force) is under this
	double handbrake; // Friction factor applied when handbrake is pulled

	// Variables
	double brakeFactor;
	double handbrakeFactor;
	bool locked;

	// For info only
	double lastTorque;
};
