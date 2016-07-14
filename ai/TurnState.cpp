#include "States.hpp"

TurnState::TurnState(ControllerInterface *interface, bool isLeftTurn, double turnRadius, int subdivisions)
 	: mInterface(interface) {
	const double finalAngle = (45.0 * M_PI / 180.0) * (isLeftTurn ? 1.0 : -1.0); // Angle signs are inverted
	const double angleDiv = finalAngle / subdivisions;
	MathVector<double, 2> forwardVec = mInterface->getCarDirection();

	for (int i = 0; i < subdivisions; i++) {
//		if (i > subdivisions / 2 && i % 2 == 0) continue;

		const double angle = angleDiv * (i + 1);
		const double dist = turnRadius * sqrt(2 * (1 - cos(angle)));

		MathVector<double, 2> vecToTurnPoint;
		vecToTurnPoint[0] = (forwardVec[0] * cos(angle) - forwardVec[1] * sin(angle)) * dist;
		vecToTurnPoint[1] = (forwardVec[0] * sin(angle) + forwardVec[1] * cos(angle)) * dist;

		MathVector<double, 2> turnPoint = mInterface->getCarPosition() + vecToTurnPoint;

		mWaypoints.push(turnPoint);
	}

	// Setting "desired" final values
	{
		const double angle = (M_PI / 2) * (isLeftTurn ? 1.0 : -1.0); // Angle signs are inverted
		desiredFinalDir[0] = (forwardVec[0] * cos(angle) - forwardVec[1] * sin(angle));
		desiredFinalDir[1] = (forwardVec[0] * sin(angle) + forwardVec[1] * cos(angle));

		startSpeed = mInterface->getCarSpeed();
	}

	MathVector<double, 2> lastPoint(mWaypoints.back());
	float continueDist = 5;
	lastPoint[0] += desiredFinalDir[0] * continueDist;
	lastPoint[1] += desiredFinalDir[1] * continueDist;
	mWaypoints.push(lastPoint);

	mWaypointState = new WaypointState(mInterface, mWaypoints.front(), 1);
	mWaypoints.pop();
}

BaseState* TurnState::update(float dt) {
	BaseState* nextState = mWaypointState->update(dt);
	if (nextState != NULL) {
		delete mWaypointState;

		if (mWaypoints.empty()) {
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

