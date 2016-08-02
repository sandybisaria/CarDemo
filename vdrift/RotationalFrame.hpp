#pragma once

#include "Quaternion.hpp"
#include "MathVector.hpp"
#include "Matrix3.hpp"

// Stuntrally's ROTATIONALFRAME class, using the Suvat method of integration.
// Alternative integration methods removed
class RotationalFrame {
public:
	RotationalFrame() : haveOldTorque(false), integrationStep(0) {}

	void setInertia(const Matrix3<double>& inertia) {
		MathVector<double, 3> avOld = getAngularVelocityFromMomentum(angMom);

		inertiaTensor = inertia;
		invInertiaTensor = inertiaTensor.inverse();

		worldInvInertiaTensor = orientationMat.transpose().multiply(invInertiaTensor).multiply(orientationMat);
		worldInertiaTensor = orientationMat.transpose().multiply(inertiaTensor).multiply(orientationMat);

		angMom = worldInertiaTensor.multiply(avOld);
		angVel = getAngularVelocityFromMomentum(angMom);
	}

	const Matrix3<double>& getInertia() const { return worldInertiaTensor; }
	const Matrix3<double>& getInertiaLocal() const { return inertiaTensor; }

	void setOrientation(const Quaternion<double>& no) { orientation = no; }
	const Quaternion<double>& getOrientation() const { return orientation; }

	void setAngularVelocity(const MathVector<double, 3>& nav) {
		angMom = worldInertiaTensor.multiply(nav);
		angVel = nav;
	}
	const MathVector<double, 3>& getAngularVelocity() const { return angVel; }

//---- Modified velocity Verlet integration two-step method
//---- Both steps must be called per frame
//---- Forces can only be set between steps 1 and 2
	void integrateStep1(double dt) {
		assert(integrationStep == 0);
		assert(haveOldTorque); // Must call setInitialTorque()
		integrationStep++;
	}
	void integrateStep2(double dt) {
		assert(integrationStep == 1);

		orientation = orientation + getSpinFromMomentum(angMom + torque * dt * 0.5) * dt;
		orientation.normalize();
		angMom = angMom + torque * dt;

		recalculateSecondary();
		integrationStep = 0;
		torque.set((double)0);
	}

//---- Must only be called between integration steps 1 and 2
	const MathVector<double, 3> getLockUpTorque(double dt) const {
		assert(integrationStep == 1);
		return -angMom / dt;
	}
	void applyTorque(const MathVector<double, 3>& t) {
		assert(integrationStep == 1);
		torque = torque + t;
	}
	void setTorque(const MathVector<double, 3>& t) {
		assert(integrationStep == 1);
		torque = t;
	}

	const MathVector<double, 3>& getTorque() const { return oldTorque; }

//---- Must be called once when simulation starts to set initial torque
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

		worldInvInertiaTensor = orientationMat.transpose().multiply(invInertiaTensor).
								multiply(orientationMat);
		worldInertiaTensor = orientationMat.transpose().multiply(inertiaTensor).
							 multiply(orientationMat);

		angVel = getAngularVelocityFromMomentum(angMom);
	}

//---- orientationMat and worldInvIntertiaTensor must have been calculated
	MathVector<double, 3> getAngularVelocityFromMomentum(const MathVector<double, 3>& mom) const {
		return worldInvInertiaTensor.multiply(mom);
	}

	Quaternion<double> getSpinFromMomentum(const MathVector<double, 3>& am) const {
		const MathVector<double, 3> av = getAngularVelocityFromMomentum(am);
		Quaternion<double> qav = Quaternion<double>(av[0], av[1], av[2], 0);
		return (qav * orientation) * 0.5;
	}

//----- Primary
	Quaternion<double> orientation;
	MathVector<double, 3> angMom;
	MathVector<double, 3> torque;

//---- Secondary
	MathVector<double, 3> oldTorque;
	Matrix3<double> orientationMat;
	Matrix3<double> worldInvInertiaTensor;
	Matrix3<double> worldInertiaTensor;
	MathVector<double, 3> angVel;

//---- Constants
	Matrix3<double> invInertiaTensor;
	Matrix3<double> inertiaTensor;

//---- Housekeeping
	bool haveOldTorque;
	int integrationStep;
};
