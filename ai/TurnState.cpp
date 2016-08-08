#include "States.hpp"

#include "predictSteering/predictSteering.h"

#include <cfloat>

//---- TurnState methods
TurnState::TurnState(ControllerInterface *interface, bool isLeftTurn, double turnRadius, double angle)
 	: BaseState(interface) {
	startSpeed = mInterface->getCarSpeed();

	double inputs[] = {startSpeed, turnRadius};

	// Using the predictSteering method (developed using neural networks!..)
	turn = predictSteering(inputs) * (isLeftTurn ? -1.0 : 1.0); // Left (CCW) is negative for turning
	turn = clamp(turn, -1.0, 1.0); // Maybe give an error or something when attempting an impossible turn

	angle = fabs(angle); // Ignore the angle sign; only listen to isLeftTurn
	const double turnAngle = (angle * M_PI / 180) * (isLeftTurn ? 1.0 : -1.0); // In geometry, left (CCW) is positive

	// Distance from start position to final point
	const double distFinalPoint = turnRadius * M_SQRT2 * sqrt(1 - cos(turnAngle));

	// vecFinalPoint is a vector from the car's starting position to the final point
	MathVector<double, 2> vecFinalPoint(distFinalPoint), carDir = mInterface->getCarDirection();
	vecFinalPoint[0] *= (carDir[0] * cos(turnAngle / 2) - carDir[1] * sin(turnAngle / 2));
	vecFinalPoint[1] *= (carDir[0] * sin(turnAngle / 2) + carDir[1] * cos(turnAngle / 2));

	finalPoint = vecFinalPoint + mInterface->getCarPosition();
	finalDir[0] = carDir[0] * cos(turnAngle) - carDir[1] * sin(turnAngle);
	finalDir[1] = carDir[0] * sin(turnAngle) + carDir[1] * cos(turnAngle);

	lastDist = DBL_MAX;
}

BaseState* TurnState::update(float dt) {
	double currDist = (mInterface->getCarPosition() - finalPoint).magnitude();
	if (currDist < 0.1) {
		std::cout << "Turn complete, angle error (rad) = "
				  << mInterface->getAngle(finalDir,mInterface->getCarDirection())
				  << std::endl;

		return new ConstantState(mInterface, startSpeed);
	}
	else if (currDist > lastDist) {
		// May want to more intelligently handle an imperfect turn
		std::cout << "Not quite perfect... off by " << currDist << " with angle error (rad) = "
				  << mInterface->getAngle(finalDir, mInterface->getCarDirection())
				  << std::endl;

		// If following a road, just go to the next waypoint...
		return new ConstantState(mInterface, startSpeed);
	}
	else {
		mInterface->setSteering(turn);
		mInterface->setTargetSpeed(startSpeed);

		lastDist = currDist;

		return NULL;
	}
}

//---- ConstantTurnState methods
ConstantTurnState::ConstantTurnState(ControllerInterface *interface, double turn,
									 double startSpeed)
	: BaseState(interface) {
	this->turn = turn;
	this->startSpeed = (startSpeed == 0 ? mInterface->getCarSpeed() : startSpeed);

	minX = maxX = mInterface->getCarPosition()[0];
	minY = maxY = mInterface->getCarPosition()[1];

	looped = canHaveLooped = false;
	startDir = mInterface->getCarDirection();

	radiusX = radiusY = 0;
}

BaseState* ConstantTurnState::update(float dt) {
	mInterface->setSteering(turn);
	mInterface->setTargetSpeed(startSpeed);

	bool updated = false;
	double x = mInterface->getCarPosition()[0], y = mInterface->getCarPosition()[1];
		 if (x < minX) { minX = x; updated = true; }
	else if (x > maxX) { maxX = x; updated = true; }
		 if (y < minY) { minY = y; updated = true; }
	else if (y > maxY) { maxY = y; updated = true; }
	if (updated) {
		radiusX = (maxX - minX) / 2;
		radiusY = (maxY - minY) / 2;
	}

	if (!canHaveLooped) {
		double dot = startDir.dot(mInterface->getCarDirection());
		if (dot < 0) { canHaveLooped = true; }
	}
	if (canHaveLooped && !looped) {
		double dot = startDir.dot(mInterface->getCarDirection());
		if (dot > 0.99) { looped = true; }
	}

	return NULL;
}