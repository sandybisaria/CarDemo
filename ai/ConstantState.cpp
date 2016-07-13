#include "States.hpp"

#include "BasicController.hpp"

ConstantState::ConstantState(ControllerInterface* interface, double speed, double angle)
	: mInterface(interface), mSpeed(speed), mAngle(angle) {
	mInterface->setTargetSpeed(mSpeed);
	mInterface->setTargetAngle(mAngle);
}

ConstantState::~ConstantState() {
	delete mInterface;
}

void ConstantState::update(float dt) {
	mInterface->setTargetSpeed(mSpeed);
	mInterface->setTargetAngle(mAngle, false);
}