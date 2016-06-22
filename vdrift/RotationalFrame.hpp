#pragma once

#include "Quaternion.hpp"
#include "MathVector.hpp"
#include "Matrix3.hpp"

#define SUVAT // Using Suvat method of integration

// From vdrift/rotationalframe.h
class RotationalFrame {
public:
	RotationalFrame() : haveOldTorque(false), integrationStep(0) {}

	void setInertia(const Matrix3<double>& inertia) {
		MathVector<double, 3> avOld = getAngVelFromMom(angMom);

		inertiaTensor = inertia;
		invInertiaTensor = inertiaTensor.inverse();

		worldInvInertiaTensor = orientationMat.transpose().multiply(invInertiaTensor).multiply(orientationMat);
		worldInertiaTensor = orientationMat.transpose().multiply(inertiaTensor).multiply(orientationMat);

		angMom = worldInertiaTensor.multiply(avOld);
		angVel = getAngVelFromMom(angMom);
	}

	const Matrix3<double>& getInertia() const { return worldInertiaTensor; }
	const Matrix3<double>& getInertiaLocal() const { return inertiaTensor; }

	void setOrientation(const Quaternion<double>& no) { orientation = no; }
	const Quaternion<double>& getOrientation() const { return orientation; }

	void setAngularVelocity(const MathVector<double, 3>& nav) {
		angMom = worldInertiaTensor.multiply(nav);
		angVel = nav;
	}
	const MathVector<double, 3> getAngularVelocity() const { return angVel; }

	// Modified velocity Verlet integration two-step method
	// Both steps must be called per frame
	// Forces can only be set between steps 1 and 2
	void integrateStep1(const double& dt) {
		assert(integrationStep == 0);
		assert(haveOldTorque); // Must call setInitialTorque()

#ifdef MODIFIEDVERLET
		angMom = angMom + oldTorque * dt * 0.5;
		orientation = orientation + getSpinFromMom(angMom) * dt;
		orientation.normalize();
		recalculateSecondary();
#endif

		integrationStep++;
	}

	// Modified velocity Verlet integration two-step method
	// Both steps must be called per frame
	// Forces can only be set between steps 1 and 2
	void integrateStep2(const double& dt) {
		assert(integrationStep == 1);

#ifdef MODIFIEDVERLET
		angMom = ngMom + torque * dt * 0.5;
#endif

#ifdef NSV
		angMom = angMom + torque * dt;
		orientation = orientation + getSpinFromMom(angMom) * dt;
		orientation.normalize()
#endif

#ifdef EULER
		orientation = orientation + getSpinFromMom(angMom) * dt;
		orientation.normalize();
		angMom = angMom + torque * dt;
#endif

#ifdef SUVAT
		orientation = orientation + getSpinFromMom(angMom + torque * dt * 0.5) * dt;
		orientation.normalize();
		angMom = angMom + torque * dt;
#endif

#ifdef SECOND_ORDER
		MathVector<double, 3> angAcc = worldInvInertiaTensor.multiply(torque - angVel.cross(angMom));
		MathVector<double, 3> avgRot = angVel + angAcc * dt / 2.0 + angAcc.cross(angVel) * dt * dt / 12.0;
		Quaternion<double> dq = Quaternion<double>(avgRot[0], avgRot[1], avgRot[2], 0) * orientation * 0.5 * dt;
		orientation = orientation + dq;
		orientation.normalize();
#endif

		recalculateSecondary();
		integrationStep = 0;
		torque.set((double)0);
	}

	// Must only be called between integration steps 1 and 2
	const MathVector<double, 3> getLockUpTorque(const double dt) const {
		assert(integrationStep == 1);
#ifdef MODIFIEDVERLET
	return -angMom * 2 / dt;
#else
	return -angMom / dt;
#endif
	}

	// Must only be called between integration steps 1 and 2
	void applyTorque(const MathVector<double, 3>& t) {
		assert(integrationStep == 1);
		torque = torque + t;
	}

	// Must only be called between integration steps 1 and 2
	void setTorque(const MathVector<double, 3>& t) {
		assert(integrationStep == 1);
		torque = t;
	}

	const MathVector<double, 3>& getTorque() { return oldTorque; }

	// Must be called once when simulation starts to set initial torque
	void setInitialTorque(const MathVector<double, 3>& t) {
		assert(integrationStep == 0);

		oldTorque = t;
		haveOldTorque = true;
	}

private:
	void recalculateSecondary() {
		oldTorque = torque;
		haveOldTorque = true;

		orientation.representAsMatrix3(orientationMat);

		worldInvInertiaTensor = orientationMat.transpose().multiply(invInertiaTensor).multiply(orientationMat);
		worldInertiaTensor = orientationMat.transpose().multiply(inertiaTensor).multiply(orientationMat);

		angVel = getAngVelFromMom(angMom);
	}

	// orientationMat and worldInvIntertiaTensor must have been calculated
	MathVector<double, 3> getAngVelFromMom(const MathVector<double, 3>& mom) const {
		return worldInvInertiaTensor.multiply(mom);
	}

	Quaternion<double> getSpinFromMom(const MathVector<double, 3>& am) const {
		const MathVector<double, 3> av = getAngVelFromMom(am);
		Quaternion<double> qav = Quaternion<double>(av[0], av[1], av[2], 0);
		return qav * orientation * 0.5;
	}

	// Primary values
	Quaternion<double> orientation;
	MathVector<double, 3> angMom; // Angular momentum
	MathVector<double, 3> torque;

	// Secondary values
	MathVector<double, 3> oldTorque;
	Matrix3<double> orientationMat;
	Matrix3<double> worldInvInertiaTensor;
	Matrix3<double> worldInertiaTensor;
	MathVector<double, 3> angVel; // Angular velocity

	// Constants
	Matrix3<double> invInertiaTensor;
	Matrix3<double> inertiaTensor;

	bool haveOldTorque;
	int integrationStep;
};
