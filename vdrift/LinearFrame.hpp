#pragma once

#include "MathVector.hpp"

// Stuntrally's LINEARFRAME class, using the Suvat method of integration.
// Alternative integration methods removed
class LinearFrame {
public:
	LinearFrame() : invMass(1.0), haveOldForce(false), integrationStep(0) { }

	void setMass(double mass) { invMass = 1.0 / mass; }
	const double getMass() const { return 1.0 / invMass; }

	void setPosition(const MathVector<double, 3>& newPos) { pos = newPos; }
	const MathVector<double, 3>& getPosition() const { return pos; }

	void setVelocity(const MathVector<double, 3>& newVel) { mom = newVel / invMass; }
	const MathVector<double, 3> getVelocity() const { return getVelocityFromMomentum(mom); }

//---- Modified velocity Verlet integration two-step method
//---- Both steps must be run per frame. Forces can only be set between steps 1 and 2
	void integrateStep1(const double& dt) {
		assert(integrationStep == 0);
		assert(haveOldForce); // Call setInitialForce() on first-time integration

		integrationStep++;
	}
	void integrateStep2(const double& dt) {
		assert(integrationStep == 1);

		pos = pos + mom * invMass * dt + force * invMass * dt * dt * 0.5;
		mom = mom + force * dt;

		recalculateSecondary();

		integrationStep = 0;
		force.set(0.0);
	}

//---- Must only be called between integration steps 1 and 2
	void applyForce(const MathVector<double, 3>& f) {
		assert(integrationStep == 1);
		force = force + f;
	}
	void setForce(const MathVector<double, 3>& f) {
		assert(integrationStep == 1);
		force = f;
	}

//---- Must be called once at simulation start to set initial force
	void setInitialForce(const MathVector<double, 3>& f) {
		assert(integrationStep == 0);
		oldForce = f;
		haveOldForce = true;
	}

	const MathVector<double, 3>& getForce() const { return oldForce; }

private:
	void recalculateSecondary() {
		oldForce = force;
		haveOldForce = true;
	}

	MathVector<double, 3> getVelocityFromMomentum(const MathVector<double, 3>& mom) const {
		return mom * invMass;
	}

//---- Primary
	MathVector<double, 3> pos;
	MathVector<double, 3> mom; // Momentum
	MathVector<double, 3> force;

//---- Secondary
	MathVector<double, 3> oldForce;

//----- Constants
	double invMass;

//---- Housekeeping
	bool haveOldForce;
	int integrationStep;
};
