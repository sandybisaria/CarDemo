#pragma once

#include "RotationalFrame.hpp"

#include <cmath>

#define _USE_MATH_DEFINES

// Stuntrally's CARWHEEL class
class CarWheel {
public:
	// Default makes an S2000-like car
	CarWheel()
		: rollHeight(0.9), mass(18.1), linRollRes(1.3e-2), quadRollRes(6.5e-6),
		  inertiaCache(10.0), steerAngle(0.0), radius(0.3),
		  feedback(0.0), angVel(0.0), camberDeg(0.0), fluidRes(0.0) { setInertia(10.0); }

	double getRollingResistance(double vel, double rollResFactor) const {
		// Surface influence on rolling resistance
		double rollRes = linRollRes * rollResFactor;

		// Heat due to tire deformation increases rolling resistance
		rollRes += vel * vel * quadRollRes;

		double res = rollRes;
		// Determine direction
		if (vel < 0) { res = -res; }

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
	void setSteerAngle(double sa) { steerAngle = sa; }

	void setRadius (double value) { radius = value; }
	double getRadius() const { return radius; }

	void setRollHeight (double value) { rollHeight = value; }
	double getRollHeight() const { return rollHeight; }

	void setMass (double value) { mass = value; }
	double getMass() const { return mass; }

	void setInertia(double i) {
		inertiaCache = i;

		Matrix3<double> in;
		in.scale(i);
		rotFr.setInertia(in);
	}
	double getInertia() const {	return inertiaCache; }

	void setFeedback(double fb) { feedback = fb; }
	double getFeedback() const { return feedback; }

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
	double getCamberDeg() const { return camberDeg; }

	struct SlideSlip {
		double slide; // Ratio of tire contact patch speed to road speed, minus one
		double slip; // Angle in degrees between wheel heading and actual wheel velocity
		double slideRatio; // Ratio of slide to tire's optimum slide
		double slipRatio; // Ratio of slip to tire's optimum slip
		double fxSr, fxRsr, fyAr, fyRar, frict, gamma, fx, fxm, preFx, fy, fym, preFy, fz;

		SlideSlip()
			: slide(0), slip(0), slideRatio(0), slipRatio(0),
			  fxSr(0), fxRsr(0), fyAr(0), fyRar(0),
			  frict(0), gamma(0), fx(0), fxm(0), preFx(0), fy(0), fym(0), preFy(0), fz(0) { }

	} slips;

	double fluidRes;
private:
//---- Constants
	MathVector<double, 3> extendedPosition; // Position when suspension is fully extended (zero g)
	double rollHeight; // How far off the road lateral forces are applied to the chassis
	double mass;
	RotationalFrame rotFr; // Simulation of wheel rotation

	double linRollRes; // Linear rolling resistance on hard surface
	double quadRollRes; // Quadratic rolling resistance on hard surface

//---- Variables
	double inertiaCache;
	double steerAngle; // Negative values = steering left!
	double radius; // Total radius of wheel
	double feedback; // Effect value of force feedback

//---- For info only
	double angVel;
	double camberDeg;

};
