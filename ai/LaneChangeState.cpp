#include "States.hpp"

LaneChangeState::LaneChangeState(ControllerInterface *interface, bool isLeft, double laneWidth)
	: BaseState(interface), mIsLeft(isLeft) {
	startSpeed = mInterface->getCarSpeed();

	theta = 15; // Hard-coded value for now
	radius = (laneWidth / 2) / (1 - cos(theta * (M_PI / 180)));

	std::cout << theta << " " << radius << std::endl;

	currState = new TurnState(mInterface, mIsLeft, radius, theta);
	halfwayDone = false;
}

BaseState* LaneChangeState::update(float dt) {
	BaseState* res = currState->update(dt);
	if (res != NULL) {
		if (!halfwayDone) {
			currState = new TurnState(mInterface, !mIsLeft, radius, theta);
			halfwayDone = true;
			return NULL;
		} else {
			return new ConstantState(mInterface, startSpeed, 0);
		}
	} else {
		return NULL;
	}
}