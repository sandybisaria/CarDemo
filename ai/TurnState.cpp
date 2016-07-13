#include "States.hpp"

TurnState::TurnState(ControllerInterface *interface, bool isLeftTurn, double turnRadius)
 	: mInterface(interface) {
	const double diagDist = turnRadius * sqrt(2);
	const double angle = (45.0 * M_PI / 180.0) * (isLeftTurn ? 1.0 : -1.0); // Angle signs are inverted
	MathVector<double, 2> forwardVec = mInterface->getCarDirection();

	MathVector<double, 2> vecToTurnPoint;
	vecToTurnPoint[0] = (forwardVec[0] * cos(angle) - forwardVec[1] * sin(angle)) * diagDist;
	vecToTurnPoint[1] = (forwardVec[0] * sin(angle) + forwardVec[1] * cos(angle)) * diagDist;

	MathVector<double, 2> turnPoint = mInterface->getCarPosition() + vecToTurnPoint;

	mWaypointState = new WaypointState(mInterface, turnPoint, 1);
}

BaseState* TurnState::update(float dt) {
	BaseState* nextState = mWaypointState->update(dt);
	if (nextState != NULL) {
		delete mWaypointState;
		std::cout << "Turn complete" << std::endl;
	}

	return nextState;
}

