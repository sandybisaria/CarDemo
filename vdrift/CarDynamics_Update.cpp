#include "CarDynamics.hpp"

#include <iostream>

// Last function executed (after integration) in Bullet's stepSimulation
void CarDynamics::updateAction(btCollisionWorld* collisionWorld, btScalar dt) {
	bool success = synchronizeBody(); // Obtain velocity/position orientation from Bullet after dt
	if (!success) return; // If values invalid

	updateWheelContacts(); // Given new velocity/position
	tick(dt); // Run internal simulation of transmission, body, driveline
	synchronizeChassis(); // Update velocity on Bullet
}

void CarDynamics::update() {
	if (!chassis) return;

	btTransform tr;
	chassis->getMotionState()->getWorldTransform(tr);

	chassisRotation = toMathQuaternion<double>(tr.getRotation());
	MathVector<double, 3> chassisCenterOfMass = toMathVector<double>(tr.getOrigin());
	chassisRotation.rotateVector(centerOfMass);
	chassisPosition = chassisCenterOfMass - centerOfMass;

	//TODO updateBuoyancy()
}

bool CarDynamics::synchronizeBody() {
	MathVector<double, 3> v = toMathVector<double>(chassis->getLinearVelocity());
	MathVector<double, 3> w = toMathVector<double>(chassis->getAngularVelocity());
	MathVector<double, 3> p = toMathVector<double>(chassis->getCenterOfMassPosition());
	Quaternion<double> q = toMathQuaternion<double>(chassis->getOrientation());

	body.setPosition(p);
	body.setOrientation(q);
	body.setVelocity(v);
	body.setAngularVelocity(w);
	//
	//	std::cout << "VELOCITY GET: " << v << std::endl;

	return true;
}

void CarDynamics::updateWheelContacts() {
	MathVector<float, 3> rayDir = getDownVector();
	for (int i = 0; i < numWheels; i++) {
		CollisionContact& wheelCon = wheelContact[i];
		MathVector<float, 3> rayStart = localToWorld(wheels[i].getExtendedPosition());
		rayStart = rayStart - rayDir * wheels[i].getRadius();
		float rayLen = 1.5;

		world->castRay(rayStart, rayDir, rayLen, chassis, wheelCon, this, i, false); // False because we have car collisions
	}
}

// One simulation step
void CarDynamics::tick(double dt) {
	// Must happen before updateDriveline
	updateTransmission(dt);

	const int numReps = 30; //TODO Would be from SETTINGS
	const float internalDt = dt / numReps;
	for (int i = 0; i < numReps; i++) {
		double driveTorque[MAX_WHEEL_COUNT];
		updateDriveline(internalDt, driveTorque);
		updateBody(internalDt, driveTorque);
		//TODO Ignoring feedback var
	}

	fuelTank.consume(engine.fuelRate() * dt);
	//TODO Ignoring fHitTime;
}


void CarDynamics::synchronizeChassis() {
	chassis->setLinearVelocity(toBulletVector(body.getVelocity()));
	chassis->setAngularVelocity(toBulletVector(body.getAngularVelocity()));
//
//	std::cout << "VELOCITY SYNC: " << body.getVelocity() << std::endl;
}

void CarDynamics::updateBody(double dt, double driveTorque[]) {
	body.integrateStep1(dt);
	//TODO Skipping camera body, camera bounce

	updateWheelVelocity();
	applyEngineTorqueToBody();
	applyAerodynamicsToBody();

	//TODO Care about scene damage, wind, car flips, boosts?

	int i;
	double normalForce;
	for (i = 0; i < numWheels; i++) {
		MathVector<double, 3> suspForce = updateSuspension(i, dt);
		normalForce = suspForce.dot(wheelContact[i].getNormal());
		if (normalForce < 0) normalForce = 0;

		MathVector<double, 3> tireFriction = applyTireForce(i, normalForce, wheelRots[i]);
		applyWheelTorque(dt, driveTorque[i], i, tireFriction, wheelRots[i]);
	}

	body.integrateStep2(dt);

	for (i = 0; i < numWheels; i++) {
		wheelPos[i] = getWheelPositionAtDisplacement(WheelPosition(i), suspension[i].getDisplacementPercent());
		wheelRots[i] = getBodyOrientation() * getWheelSteeringAndSuspensionOrientation(WheelPosition(i));
	}

	interpolateWheelContacts(dt);

	//TODO Add ABS and TCS
}
