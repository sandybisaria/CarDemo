#pragma once

#include "MathVector.hpp"
#include "Spline.hpp"
#include "RotationalFrame.hpp"
#include "Matrix3.hpp"
#include "LinearInterp.hpp"

#include <map>

class CarEngine {
public:
	// Default makes an S2000-like car
	CarEngine();

	double realPowTorqueMul; // .car params

	double getTorqueCurve(const double curThrottle, const double curRPM) const;
	double getFrictionTorque(double curAngVel, double fricFactor, double throttlePos);

	void setInertia(const double& i) {
		Matrix3<double> inertia;
		inertia.scale(i);
		crankshaft.setInertia(inertia);
	}
	double getInertia() const { return crankshaft.getInertia()[0]; }

	void setFrictionB(double val) { fricCoeffB = val; }
	double getFrictionB() const { return fricCoeffB; }

	void setMaxRPM(double val) { maxRPM = val; }
	double getMaxRPM() const { return maxRPM; }

	double getIdle() const { return idle; }

	void setStartRPM(double val) { startRPM = val; }
	double getStartRPM() const { return startRPM; }

	void setStallRPM(double val) { stallRPM = val; }
	double getStallRPM() const { return stallRPM; }

	void setFuelConsumption(double val) { fuelConsumption = val; }
	double getFuelConsumption() const { return fuelConsumption; }

	void integrateStep1(const double dt) { crankshaft.integrateStep1(dt); }
	void integrateStep2(const double dt) { crankshaft.integrateStep2(dt); }

	const double getRPM() const { return crankshaft.getAngularVelocity()[0] * 30 / M_PI; }

	// 0.0 = no throttle; 1.0 = full throttle;
	void setThrottle(double val) { throttlePosition = val; }
	double getThrottle() const { return throttlePosition; }

	void setInitialConditions() {
		MathVector<double, 3> v;
		crankshaft.setInitialTorque(v);
		startEngine();
	}

	void startEngine() {
		MathVector<double, 3> v;
		v[0] = startRPM * M_PI / 30.0;
		crankshaft.setAngularVelocity(v);
	}

	// Used to set engine drag from clutch being partially engaged
	void setClutchTorque(double val) { clutchTorque = val; }

	double getAngularVelocity() const { return crankshaft.getAngularVelocity()[0]; }
	void setAngularVelocity(double val) {
		MathVector<double, 3> v(val, 0, 0);
		crankshaft.setAngularVelocity(v);
	}

	// Sum of all torque (except clutch forces) acting on engine)
	double getTorque() const { return combustionTorque + frictionTorque; }

	void computeForces();
	void applyForces();

	// Set torque curve using vector of (RPM, torque) pairs.
	// Also recalculate engine friction.
	// maxPowerRPM should be set to engine red line
	void setTorqueCurve(double maxPowerRPM, std::vector<std::pair<double, double> >& curve);

	void setMass(double val) { mass = val; }
	double getMass() const { return mass; }

	void setPosition(MathVector<double, 3>& value) { position = value; }
	MathVector<double, 3> getPosition() const { return position; }

	double fuelRate() const { return fuelConsumption * getAngularVelocity() * throttlePosition; }

	void setOutOfFuel(bool value) { outOfFuel = value; }

	// True if engine is combusting fuel
	bool isCombusting() const { return !stalled; }

private:
	// Constants
	double maxRPM; // The "red line" in RPM
	double idle; // Idle throttle percentage; calculated algorithmically
	double startRPM; // Initial RPM
	double stallRPM; // RPM at which engine dies
	double fuelConsumption; // fuelConsumption * RPM * throttle = liters of fuel consumer per second
	double friction; // Friction coefficient from engine; calculated algorithmically
	double fricCoeffB;

	std::map<double, double> torqueMap; // Set of RPMs that map to torque values
	double mass;
	MathVector<double, 3> position;
	Spline<double> torqueCurve;

	// Variables
	double throttlePosition;
	double clutchTorque;
	bool outOfFuel;
	bool revLimitExceeded;
	RotationalFrame crankshaft;

	// For info only
	double frictionTorque;
	double combustionTorque;
	bool stalled;
};
