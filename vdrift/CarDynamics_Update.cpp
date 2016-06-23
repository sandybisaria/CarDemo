#include "CarDynamics.hpp"

// Last function executed (after integration) in Bullet's stepSimulation
void CarDynamics::updateAction(btCollisionWorld* collisionWorld, btScalar dt) {

}

void CarDynamics::update() {
	if (!chassis) return;

	btTransform tr;
	chassis->getMotionState()->getWorldTransform(tr);
	chassisRotation = toMathQuaternion<double>(tr.getRotation());

	MathVector<double, 3> chassisCenterOfMass = toMathVector<double>(tr.getOrigin());
	MathVector<double, 3> com = centerOfMass;
	chassisRotation.rotateVector(com);
	chassisPosition = chassisCenterOfMass - com;

	//TODO updateBuoyancy()
}
