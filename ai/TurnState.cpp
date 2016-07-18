#include "States.hpp"

TurnState::TurnState(ControllerInterface *interface, bool isLeftTurn, double turnRadius)
 	: mInterface(interface) {
	startSpeed = mInterface->getCarSpeed();
	turn = getTurn(turnRadius, startSpeed) * (isLeftTurn ? -1.0 : 1.0); // Left (CCW) is negative for turning

	//TODO Should be passed as an argument
	const double turnAngle = M_PI_2 * (isLeftTurn ? 1.0 : -1.0); // In geometry, left (CCW) is positive

	const double distFinalPoint = turnRadius * M_SQRT2 * sqrt(1 - cos(turnAngle)); // Distance from start position to final point
	MathVector<double, 2> vecFinalPoint, carDir = mInterface->getCarDirection();
	// vecFinalPoint is a vector from the car's starting position to the final point
	vecFinalPoint[0] = (carDir[0] * cos(turnAngle / 2) - carDir[1] * sin(turnAngle / 2)) * distFinalPoint;
	vecFinalPoint[1] = (carDir[0] * sin(turnAngle / 2) + carDir[1] * cos(turnAngle / 2)) * distFinalPoint;

	finalPoint = vecFinalPoint + mInterface->getCarPosition();
	finalDir[0] = carDir[0] * cos(turnAngle) - carDir[1] * sin(turnAngle);
	finalDir[1] = carDir[0] * sin(turnAngle) + carDir[1] * cos(turnAngle);

	lastDist = DBL_MAX;
}

BaseState* TurnState::update(float dt) {
	double currDist = (mInterface->getCarPosition() - finalPoint).magnitude();
	if (currDist < 1) {
		std::cout << "Turn complete" << std::endl;
		return new ConstantState(mInterface, startSpeed, mInterface->getAngle(finalDir, mInterface->getCarDirection()));
	}
	else if (currDist > lastDist) {
		//TODO May want to more intelligently handle a miscalculated turn
		std::cout << "Not quite perfect... oh well" << std::endl;
		return new ConstantState(mInterface, startSpeed, mInterface->getAngle(finalDir, mInterface->getCarDirection()));
	}
	else {
		mInterface->setSteering(turn);
		mInterface->setTargetSpeed(startSpeed);

		lastDist = currDist;

		return NULL;
	}
}

double TurnState::getTurn(double turnRadius, double speed) {
	// Rough nonlinear regression based on collected data-points
	const double a = 0.0821,
				 b = 0.8608,
				 c = 0.01923,
				 d = 5.415,
				 e = 6.983;
	const double x = speed, y = turnRadius;

	return a * (b*y + exp(c*x + d)) / (y + e);
}