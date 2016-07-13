#include "States.hpp"

#include "BasicController.hpp"

IdleState::IdleState(BasicController *controller)
	: mController(controller) {
	mController->setTargetSpeed(10);
	mController->setTargetAngle(0.0);
}

void IdleState::update(float dt) {

}