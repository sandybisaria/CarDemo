#include "States.hpp"

LaneChangeState::LaneChangeState(ControllerInterface *interface, bool isLeft, double laneWidth)
	: BaseState(interface), mIsLeft(isLeft) {
	startSpeed = mInterface->getCarSpeed();
	finalDir = mInterface->getCarDirection(); startPos = mInterface->getCarPosition();

	theta = 15; // Hard-coded value for now; seems to work alright for 10 m/s, 3.7m lanes
	radius = (laneWidth / 2) / (1 - cos(theta * (M_PI / 180)));

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
			double angleOff = mInterface->getAngle(finalDir, mInterface->getCarDirection());
			std::cout << "I've changed lanes, but my angle error is (was) " << angleOff << std::endl;

			MathVector<double, 2> startToEnd = mInterface->getCarPosition() - startPos;
			//TODO This multiplication bug nonsense is nonsense; why should I have to multiply like this -_-
			MathVector<double, 2> longitudinalVec; longitudinalVec[0] = finalDir[0] * startToEnd.dot(finalDir); longitudinalVec[1] = finalDir[1] * startToEnd.dot(finalDir);
			MathVector<double, 2> lateralVec = startToEnd - longitudinalVec;
			std::cout << "Actual lateral distance traveled was " << lateralVec.magnitude() << std::endl;

			return new ConstantState(mInterface, startSpeed, angleOff);
		}
	} else {
		return NULL;
	}
}