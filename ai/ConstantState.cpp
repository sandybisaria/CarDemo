#include "States.hpp"

ConstantState::ConstantState(ControllerInterface* interface, double speed,
							 double angle)
	: BaseState(interface), mSpeed(speed), mAngle(angle) {
	mInterface->setTargetSpeed(mSpeed);
	mInterface->setTargetAngle(mAngle);
}

BaseState* ConstantState::update(float dt) {
	mInterface->setTargetSpeed(mSpeed);
	mInterface->setTargetAngle(mAngle, false);

	return NULL;
}
