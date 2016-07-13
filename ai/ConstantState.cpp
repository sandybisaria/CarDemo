#include "States.hpp"

#include "BasicController.hpp"

ConstantState::ConstantState(BasicController *controller, double speed, double angle)
	: mController(controller), mSpeed(speed), mAngle(angle) {
	mController->setTargetSpeed(mSpeed);
	mController->setTargetAngle(mAngle);
}

void ConstantState::update(float dt) {
	mController->setTargetSpeed(mSpeed);
	mController->setTargetAngle(mAngle, false);
}