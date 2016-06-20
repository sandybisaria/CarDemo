#pragma once

#include "RotationalFrame.hpp"

#include <cmath>
#define _USE_MATH_DEFINES

class CarWheel {
public:
	// Default makes an S2000-like car
	CarWheel()
		: rollHeight(0.9), mass(18.1), linRollRes(1.3e-2), quadRollRes(6.5e-6),
		  inertiaCache(10.0), steerAngle(0.0), radius(0.3),
		  feedback(0.0), angVel(0.0), camberDeg(0.0), fluidRes(0.0) {}

	double getRollingResistance(const double vel, const double rollResFactor) const {
		// Surface influence on rolling resistance
		double rollRes = linRollRes * rollResFactor;

		// Heat due to tire deformation increases rolling resistance
		rollRes += vel * vel * quadRollRes;

		double res = rollRes;

		// Determine direction
		if (vel < 0)
			res = -res;

		return res;
	}
	void setRollingResistance(double lin, double quad) {
		linRollRes = lin;
		quadRollRes = quad;
	}

	MathVector<double, 3> getExtendedPosition() const { return extendedPosition; }
	void setExtendedPosition(const MathVector<double, 3>& ep) { extendedPosition = ep; }

	double getRPM() const { return rotFr.getAngularVelocity()[0] * 30.0 / M_PI; }

	// Used for telemetry only
	const double& getAngVelInfo() { return angVel; }

	double getAngularVelocity() const { return rotFr.getAngularVelocity()[1]; }
	void setAngularVelocity(double angVel) {
		MathVector<double, 3> v(0, angVel, 0);
		rotFr.setAngularVelocity(v);
	}

	double getSteerAngle() const { return steerAngle; }
	void setSteerAngle(const double& sa) { steerAngle = sa; }

	void SetRadius (const double& value) { radius = value; }
	double GetRadius() const { return radius; }

	void SetRollHeight (const double& value) { rollHeight = value; }
	double GetRollHeight() const { return rollHeight; }

	void SetMass (const double& value) { mass = value; }
	double GetMass() const { return mass; }

	void SetInertia(double i) {
		inertiaCache = i;

		Matrix3<double> in;
		in.scale(i);
		rotFr.setInertia(in);
	}
	double GetInertia() const {	return inertiaCache; }

	void SetFeedback(double fb) { feedback = fb; }
	double GetFeedback() const { return feedback; }

	void setInitialConditions() {
		MathVector<double, 3> v;
		rotFr.setInitialTorque(v);
	}
	void zeroForces() {
		MathVector<double, 3> v;
		rotFr.setTorque(v);
	}

	void integrateStep1(const double dt) { rotFr.integrateStep1(dt); }
	void integrateStep2(const double dt) { rotFr.integrateStep2(dt); }

	void setTorque(const double t) {
		MathVector<double, 3> tv(0, t, 0);
		rotFr.setTorque(tv);

		angVel = getAngularVelocity();
	}
	double getTorque() { return rotFr.getTorque()[1]; }

	double getLockUpTorque(const double dt) const { return rotFr.getLockUpTorque(dt)[1]; }

	const Quaternion<double>& getOrientation() const { return rotFr.getOrientation(); }

	void setCamberDeg(const double& cd) { camberDeg = cd; }

	double fluidRes;
private:
	// Constants
	MathVector<double, 3> extendedPosition // Position when suspension is fully extended (zero g)
	double rollHeight; // How far off the road lateral forces are applied to the chassis
	double mass;
	RotationalFrame rotFr; // Simulation of wheel rotation

	double linRollRes; // Linear rolling resistance on hard surface
	double quadRollRes; // Quadratic rolling resistance on hard surface

	// Variables
	double inertiaCache;
	double steerAngle; // Negative values -> steering left
	double radius; // Total radius of wheel
	double feedback // Effect value of force feedback

	// For info only
	double angVel;
	double camberDeg;

};
