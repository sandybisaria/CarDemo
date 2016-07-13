#include "States.hpp"

#include "BasicController.hpp"

ConstantState::ConstantState(ControllerInterface* interface, double speed, double angle)
	: mInterface(interface), mSpeed(speed), mAngle(angle) {
	mInterface->setTargetSpeed(mSpeed);
	mInterface->setTargetAngle(mAngle);
}

BaseState* ConstantState::update(float dt) {
	mInterface->setTargetSpeed(mSpeed);
	mInterface->setTargetAngle(mAngle, false);

	return NULL;
}