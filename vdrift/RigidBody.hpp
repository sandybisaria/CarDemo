#pragma once

#include "LinearFrame.hpp"
#include "MathVector.hpp"
#include "Quaternion.hpp"
#include "RotationalFrame.hpp"

// Stuntrally's RIGIDBODY class
class RigidBody {
public:
//---- Accessor methods to LinearFrame
	void setInitialForce(const MathVector<double, 3>& f) { linFr.setInitialForce(f); }
	void setForce(const MathVector<double, 3>& f) { linFr.setForce(f); }
	const MathVector<double, 3>& getForce() const { return linFr.getForce(); }

	void setMass(double mass) { linFr.setMass(mass); }
	const double getMass() const { return linFr.getMass(); }

	void setPosition(const MathVector<double, 3>& pos) { linFr.setPosition(pos); }
	const MathVector<double, 3>& getPosition() const { return linFr.getPosition(); }

	void setVelocity(const MathVector<double, 3>& vel) { linFr.setVelocity(vel); }
	const MathVector<double, 3> getVelocity() const { return linFr.getVelocity(); }
	const MathVector<double, 3> getVelocity(const MathVector<double, 3>& offset) {
		return linFr.getVelocity() + rotFr.getAngularVelocity().cross(offset);
	}

//---- Accessor methods to RotationalFrame
	void setInitialTorque(const MathVector<double, 3>& t) { rotFr.setInitialTorque(t); }
	void setTorque(const MathVector<double, 3>& t) { rotFr.setTorque(t); }
	const MathVector<double, 3>& getTorque() const { return rotFr.getTorque(); }

	void setInertia(const Matrix3<double>& i) { rotFr.setInertia(i); }
	const Matrix3<double>& getInertia() const { return rotFr.getInertia(); }
	const Matrix3<double>& getInertiaLocal() const { return rotFr.getInertiaLocal(); }

	void setOrientation(const Quaternion<double>& o) { rotFr.setOrientation(o); }
	const Quaternion<double>& getOrientation() const { return rotFr.getOrientation(); }

	void setAngularVelocity(const MathVector<double, 3>& nav) { rotFr.setAngularVelocity(nav); }
	const MathVector<double, 3>& getAngularVelocity() const { return rotFr.getAngularVelocity(); }


//---- Make sure to set the required forces/torques in between integration steps
	void integrateStep1(double dt) {
		linFr.integrateStep1(dt);
		rotFr.integrateStep1(dt);
	}
	void integrateStep2(double dt) {
		linFr.integrateStep2(dt);
		rotFr.integrateStep2(dt);
	}

	const MathVector<double, 3> transformLocalToWorld(const MathVector<double, 3>& localPt) const {
		MathVector<double, 3> output(localPt);

		getOrientation().rotateVector(output);
		output = output + getPosition();

		return output;
	}
	const MathVector<double, 3> transformWorldToLocal(const MathVector<double, 3>& worldPt) const {
		MathVector<double, 3> output(worldPt);

		output = output - getPosition();
		(-getOrientation()).rotateVector(output);

		return output;
	}

	// Apply force in world space
	void applyForce(const MathVector<double, 3>& force) { linFr.applyForce(force); }

	// Apply force at offset from center of mass in world space
	void applyForce(const MathVector<double, 3>& force, const MathVector<double, 3>& offset) {
		linFr.applyForce(force);
		rotFr.applyTorque(offset.cross(force));
	}

	// Apply torque in world space
	void applyTorque(const MathVector<double, 3>& torque) { rotFr.applyTorque(torque); }

private:
	LinearFrame linFr;
	RotationalFrame rotFr;
};
