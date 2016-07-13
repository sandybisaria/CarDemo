#include "States.hpp"

TurnState::TurnState(ControllerInterface *interface, bool isLeftTurn, double turnRadius, int subdivisions)
 	: mInterface(interface) {
	const double finalAngle = (45.0 * M_PI / 180.0) * (isLeftTurn ? 1.0 : -1.0); // Angle signs are inverted
	const double angleDiv = finalAngle / subdivisions;
	MathVector<double, 2> forwardVec = mInterface->getCarDirection();

	for (int i = 0; i < subdivisions; i++) {
		const double angle = angleDiv * (i + 1);
		const double dist = turnRadius * sqrt(2 * (1 - cos(angle)));

		MathVector<double, 2> vecToTurnPoint;
		vecToTurnPoint[0] = (forwardVec[0] * cos(angle) - forwardVec[1] * sin(angle)) * dist;
		vecToTurnPoint[1] = (forwardVec[0] * sin(angle) + forwardVec[1] * cos(angle)) * dist;

		MathVector<double, 2> turnPoint = mInterface->getCarPosition() + vecToTurnPoint;

		mWaypoints.push(turnPoint);
	}
	assert(mWaypoints.size() == subdivisions);

	// Setting "desired" final values
	{
		const double angle = (M_PI / 2) * (isLeftTurn ? 1.0 : -1.0); // Angle signs are inverted
		desiredFinalDir[0] = (forwardVec[0] * cos(angle) - forwardVec[1] * sin(angle));
		desiredFinalDir[1] = (forwardVec[0] * sin(angle) + forwardVec[1] * cos(angle));

		startSpeed = mInterface->getCarSpeed();
	}

	mWaypointState = new WaypointState(mInterface, mWaypoints.front(), 1);
	mWaypoints.pop();

	std::cout << "Turn started." << std::endl;
}

BaseState* TurnState::update(float dt) {
	BaseState* nextState = mWaypointState->update(dt);
	if (nextState != NULL) {
		delete mWaypointState;

		if (mWaypoints.empty()) {
			std::cout << "Turn complete. Angle error (deg): ";
			std::cout << 180.0 * mInterface->getAngle(desiredFinalDir, mInterface->getCarDirection()) / M_PI << std::endl;

			return new ConstantState(mInterface, startSpeed,
									 mInterface->getAngle(desiredFinalDir, mInterface->getCarDirection()));
		} else {
			mWaypointState = new WaypointState(mInterface, mWaypoints.front(), 1);
			mWaypoints.pop();
			return NULL;
		}
	} else {
		return NULL;
	}
}

