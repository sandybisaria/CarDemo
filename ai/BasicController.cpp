#include "BasicController.hpp"

#include "States.hpp"

BasicController::BasicController(Car* car)
	: mCar(car), testing(false) {
	reset();

	// PID constants
	kPSpeed = 7.65629; kISpeed = 0.00656; kDSpeed = 0.00020;
	kPAngle = 15.3125; kIAngle = 0.25; kDAngle = 0.03125;

	dirAlreadyUpdated = false;

	myInterface = new ControllerInterface(this);
	currentState = new ConstantState(myInterface, 0, 0);
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

void BasicController::goToPoint(MathVector<double, 2> waypoint, double radius) {
	currentState = new WaypointState(myInterface, waypoint, radius);
}

void BasicController::setSpeed(double speed) {
	currentState = new ConstantState(myInterface, speed, targetAngle);
}

void BasicController::setAngle(double angle) {
	currentState = new ConstantState(myInterface, targetSpeed, angle);
}

void BasicController::turn(bool isLeftTurn, double turnRadius, double angle) {
	currentState = new TurnState(myInterface, isLeftTurn, turnRadius, angle);
}

void BasicController::laneChange(bool isLeft, double laneWidth) {
	currentState = new LaneChangeState(myInterface, isLeft, laneWidth);
}

const std::vector<double>& BasicController::updateInputs(float dt) {
	BaseState* nextState = currentState->update(dt);
	if (nextState != NULL) {
		delete currentState;
		currentState = nextState;
	}

	updateSpeed(dt);
	if (!dirAlreadyUpdated) { updateDirection(dt); }
	dirAlreadyUpdated = false;

	if (testing) { updateDataCollection(dt); }

	return inputs;
}

void BasicController::updateSpeed(float dt) {
	const double speed = mCar->getSpeedMPS();
	const double eSpeed = targetSpeed - speed;

	// Simple PID controller
	double thrBrkVal = 0;
	thrBrkVal += eSpeed * kPSpeed; // Proportional term

	if (fabs(eSpeed) < targetSpeed / 4) { // If error is too high, skip; don't want high integral accumulation
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
		if (fabs(acc) < 0.01 && !reachedSpeed && fabs(eSpeed) < 0.1) { reachedSpeed = true; }
	}
	lastSpeed = speed;
}

void BasicController::updateDirection(float dt) {
	const double angle = getAngle(toFlatVector(mCar->getForwardVector()), toFlatVector(initDir));
	if (isnan(angle)) return; // Abandon ship (maybe should find out where a nan might occur...)

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

void BasicController::setupDataCollection() {
	double minSpeed = 1, maxSpeed = 30, speedStep = 1;
	double minTurn = 0.05, maxTurn = 1.00, turnStep = 0.01;

	for (double speed = maxSpeed; speed >= minSpeed; speed -= speedStep) { speeds.push_back(speed); }
	for (double turn = maxTurn; turn >= minTurn; turn -= turnStep) { turns.push_back(turn); turns.push_back(-turn); }

	assert(speeds.size() > 0 && turns.size() > 0);
	currentSpeed = currentTurn = 0;

	testing = true; testStage = WAIT_SPEED; timeElapsed = 0;
	currentState = new ConstantState(myInterface, speeds[currentSpeed], 0);
	std::cout << "SETUP COMPLETE! " << speeds[currentSpeed] << " " << turns[currentTurn] << std::endl;

	dataFile.open("data.txt", std::ios::app);
}

void BasicController::updateDataCollection(float dt) {
	timeElapsed += dt;
	if (testStage == WAIT_SPEED) {
		if (hasReachedTargetSpeed() || timeElapsed > 60) {
			std::cout << "HAS REACHED: " << targetSpeed << std::endl;
			currentState = new ConstantTurnState(myInterface, turns[currentTurn], speeds[currentSpeed]);
			testStage = WAIT_STEER; timeElapsed = 0;
		}
	} else if (testStage == WAIT_STEER) {
		ConstantTurnState* cts = (ConstantTurnState*) currentState;
		if (cts->hasLooped() || timeElapsed > 60 * (50 - speeds[currentSpeed]) ) {
			dataFile << mCar->getSpeedMPS() << " " << turns[currentTurn] << " " << cts->getAverageRadius() << std::endl;

			currentTurn++;
			// Turning radii above 100m very rare... but want to do both left and right turns
			if (currentTurn >= turns.size() || (cts->getAverageRadius() > 100 && turns[currentTurn] < 0)) {
				currentTurn = 0;
				currentSpeed++;

				if (currentSpeed >= speeds.size()) {
					std::cout << "FINISHED!" << std::endl;
					exit(0);
				} else {
					std::cout << "COMP SPEED: " << speeds[currentSpeed-1] << std::endl;

					currentState = new ConstantState(myInterface, speeds[currentSpeed], 0);
					testStage = WAIT_SPEED; timeElapsed = 0;
				}
			} else {
				std::cout << "COMP TURN: " << turns[currentTurn-1] << std::endl;
				currentState = new ConstantTurnState(myInterface, turns[currentTurn], speeds[currentSpeed]);
				testStage = WAIT_STEER; timeElapsed = 0;
			}
		}
	}
}