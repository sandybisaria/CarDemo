#include "BasicController.hpp"

#include "States.hpp"

BasicController::BasicController(Car* car)
	: mCar(car), testing(false),
	  myInterface(0), currentState(0) {
	reset();

	// PID constants
	kPSpeed = 7.65629; kISpeed = 0.00656; kDSpeed = 0.00020;
	kPAngle = 15.3125; kIAngle = 0.25; kDAngle = 0.03125;

	dirAlreadyUpdated = false;

	myInterface = new ControllerInterface(this);
	currentState = new CompositeState(myInterface, 0);
}

BasicController::~BasicController() {
	delete currentState;
	delete myInterface;
}

void BasicController::reset() {
	inputs.resize(CarInput::ALL, 0.0f);

	// Speed variables
	iSpeedAcc = 0;
	dLastESpeed = 0;
	lastSpeed = 0;
	reachedSpeed = false;

	// Angle variables
	iAngleAcc = 0;
	dLastEAngle = 0;
	lastAngle = 0;
	reachedAngle = false;
}

//---- Internal control methods
void BasicController::changeState(BaseState *newState) {
	if (currentState != NULL) { delete currentState; }
	currentState = newState;
}

void BasicController::setTargetSpeed(double newSpeed) {
	if (targetSpeed != newSpeed) { // Should we ever reset these?
		iSpeedAcc = 0;
		reachedSpeed = false;
	}

	targetSpeed = newSpeed;
}

void BasicController::setTargetAngle(double newAngle, bool resetDir) {
	if (targetAngle != newAngle) { // Should we ever reset these?
		iAngleAcc = 0;
		reachedAngle = false;
	}

	targetAngle = newAngle;

	if (resetDir) { initDir = mCar->getForwardVector(); }
}

//---- Control methods
void BasicController::goToPoint(MathVector<double, 2> waypoint, double radius) {
	changeState(new WaypointState(myInterface, waypoint, radius));
}

void BasicController::setSpeed(double speed) {
	changeState(new CompositeState(myInterface, speed, targetAngle));
}

void BasicController::setAngle(double angle) {
	changeState(new ConstantState(myInterface, targetSpeed, angle));
}

void BasicController::turn(bool isLeftTurn, double turnRadius, double angle) {
	changeState(new TurnState(myInterface, isLeftTurn, turnRadius, angle));
}

void BasicController::laneChange(bool isLeft, double laneWidth) {
	changeState(new LaneChangeState(myInterface, isLeft, laneWidth));
}

//---- Update methods
const std::vector<double>& BasicController::updateInputs(float dt) {
	BaseState* nextState = currentState->update(dt);
	if (nextState != NULL) { changeState(nextState); }

	updateSpeed(dt);
	if (!dirAlreadyUpdated) { updateDirection(dt); }
	dirAlreadyUpdated = false;

	if (testing) { updateDataCollection(dt); }

	return inputs;
}

void BasicController::updateSpeed(float dt) {
	const double speed = mCar->getSpeed();
	const double eSpeed = targetSpeed - speed;

	// Simple PID controller
	double thrBrkVal = 0;
	thrBrkVal += eSpeed * kPSpeed; // Proportional term

	// If error is too high, skip; don't want high integral accumulation
	if (fabs(eSpeed) < targetSpeed / 4) {
		thrBrkVal += kISpeed * iSpeedAcc; // Integral term
		iSpeedAcc += eSpeed * dt;
	}

	if (dt != 0) { thrBrkVal += kDSpeed * ((eSpeed - dLastESpeed) / dt); } // Derivative term
	dLastESpeed = eSpeed;

	thrBrkVal = clamp(thrBrkVal, -1.0, 1.0); // Clamp from -1 to 1
	if (thrBrkVal < 0) {
		inputs[CarInput::THROTTLE] = 0;
		inputs[CarInput::BRAKE] = -thrBrkVal;
	} else {
		inputs[CarInput::THROTTLE] = thrBrkVal;
		inputs[CarInput::BRAKE] = 0;
	}

	if (dt != 0) {
		double acc = (speed - lastSpeed) / dt;
		if (fabs(acc) < 0.01 && fabs(dLastESpeed) < 5 && !reachedSpeed &&
			fabs(eSpeed) < 0.1) { reachedSpeed = true; }
	}
	lastSpeed = speed;
}

void BasicController::updateDirection(float dt) {
	const double angle = getAngle(toFlatVector(mCar->getForwardVector()), toFlatVector(initDir));
	if (isnan(angle)) return; // Maybe should find out when a nan might occur...

	const double eAngle = targetAngle - angle;

	double steerVal = 0;
	steerVal += eAngle * kPAngle; // Proportional term

	if (fabs(eAngle) < fabs(targetAngle / 8)) {
		steerVal += kIAngle * iAngleAcc; // Integral term
		iAngleAcc += eAngle * dt;
	}

	if (dt != 0) {
		steerVal += kDAngle * ((eAngle - dLastEAngle) / dt); // Derivative term
		dLastEAngle = eAngle;
	}

	steerVal = clamp(steerVal, -1.0, 1.0);
	if (steerVal < 0) {
		inputs[CarInput::STEER_RIGHT] = 0;
		inputs[CarInput::STEER_LEFT] = -steerVal;
	} else {
		inputs[CarInput::STEER_RIGHT] = steerVal;
		inputs[CarInput::STEER_LEFT] = 0;
	}

	if (dt != 0) {
		double acc = (angle - lastAngle) / dt;
		if (fabs(acc) < 0.01 && !reachedAngle && fabs(eAngle) < 0.01) { reachedAngle = true; }
	}
	lastAngle = angle;
}

//---- Utility methods
double BasicController::getAngle(MathVector<double, 2> fromDir, MathVector<double, 2> toDir) {
	double angle = acos(fromDir.dot(toDir));

	MathVector<double, 3> cross = MathVector<double, 3>(toDir[0], toDir[1], 0).cross(
		MathVector<double, 3>(fromDir[0], fromDir[1], 0));

	if (cross.dot(MathVector<double, 3>(0, 0, 1)) > 0) { // > 0 because angle sign is inverted
		angle = -angle; // Determine direction of angle (i.e. to the left or to the right of initDir)
	}

	return angle;
}

MathVector<double, 2> BasicController::toFlatVector(MathVector<double, 3> vec, bool normalize) {
	MathVector<double, 2> res(vec[0], vec[1]);
	return normalize ? res.normalized() : res;
}

//---- Debug data collection methods
void BasicController::setupDataCollection() {
	double minSpeed = 1, maxSpeed = 33.75, speedStep = 0.50;
	double minTurn = 0.10, maxTurn = 1.00, turnStep = 0.01;

	for (double speed = maxSpeed; speed >= minSpeed; speed -= speedStep) {
		speeds.push_back(speed);
	}
	for (double turn = maxTurn; turn >= minTurn; turn -= turnStep) {
		turns.push_back(turn); turns.push_back(-turn);
	}

	assert(speeds.size() > 0 && turns.size() > 0);
	currentSpeed = currentTurn = 0;

	testing = true; testStage = WAIT_SPEED; timeElapsed = 0;
	changeState(new ConstantState(myInterface, speeds[currentSpeed], 0));
	std::cout << "SETUP COMPLETE! " << speeds[currentSpeed] << " " << turns[currentTurn] << std::endl;

	dataFile.open("data.txt", std::ios::app);
}

void BasicController::updateDataCollection(float dt) {
	timeElapsed += dt;
	if (testStage == WAIT_SPEED) {
		if (hasReachedTargetSpeed() || timeElapsed > 60) {
			std::cout << "HAS REACHED: " << targetSpeed << std::endl;
			changeState(new ConstantTurnState(myInterface, turns[currentTurn],
											  speeds[currentSpeed]));
			testStage = WAIT_STEER; timeElapsed = 0;
		}
	} else if (testStage == WAIT_STEER) {
		ConstantTurnState* cts = (ConstantTurnState*) currentState;
		bool tooMuchTime = timeElapsed > 60 * (50 - speeds[currentSpeed]);
		if (cts->hasLooped() || tooMuchTime) {
			if (tooMuchTime) {
				std::cout << "TIMEOUT..." << std::endl;
			} else {
				// Store the steering angle instead... (that way, rangeMul can change and our algorithm will work still)
				double steerAngle = turns[currentTurn] * mCar->getRangeMul() * mCar->getMaxAngle();
				dataFile << mCar->getSpeed() << " " << steerAngle << " " << cts->getAverageRadius()
						 << std::endl;
			}
			currentTurn++;
			if (currentTurn >= turns.size()) {
				currentTurn = 0;
				currentSpeed++;

				if (currentSpeed >= speeds.size()) {
					std::cout << "FINISHED!" << std::endl;
					exit(0);
				} else {
					std::cout << "COMP SPEED: " << speeds[currentSpeed-1] << std::endl;

					changeState(new ConstantState(myInterface, speeds[currentSpeed], 0));
					testStage = WAIT_SPEED; timeElapsed = 0;

					mCar->reset();
				}
			} else {
				std::cout << "COMP TURN: " << turns[currentTurn-1] << std::endl;
				changeState(new ConstantTurnState(myInterface, turns[currentTurn],
												  speeds[currentSpeed]));
				testStage = WAIT_STEER; timeElapsed = 0;
			}
		}
	}
}