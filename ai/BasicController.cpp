#include "BasicController.hpp"

#include "States.hpp"

BasicController::BasicController(Car* car)
	: mCar(car) {
	reset();

	// PID constants
	kPSpeed = 7.65629; kISpeed = 0.00656; kDSpeed = 0.00020;
	kPAngle = 443.75; kIAngle = 1; kDAngle = 1; // Could be refined further; may also be correlated with speed

	myInterface = new ControllerInterface(this);
	currentState = new ConstantState(myInterface, 10, 0);
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

	// Angle variables
	iAngleAcc = 0;
	dLastEAngle = 0;
}

void BasicController::setTargetSpeed(double newSpeed) {
	targetSpeed = newSpeed;
	iSpeedAcc = 0;
	dLastESpeed = 0;
}

void BasicController::setTargetAngle(double newAngle, bool resetDir) {
	targetAngle = newAngle;
	iAngleAcc = 0;
	dLastEAngle = 0;

	if (resetDir) initDir = mCar->getForwardVector();
}

const std::vector<double>& BasicController::updateInputs(float dt) {
	BaseState* nextState = currentState->update(dt);
	if (nextState != NULL) {
		delete currentState;
		currentState = nextState;
	}

	updateSpeed(dt);
	updateDirection(dt);

	return inputs;
}

void BasicController::goToPoint(MathVector<double, 2> waypoint, double radius) {
	currentState = new WaypointState(myInterface, waypoint, radius);
}

void BasicController::turn(bool isLeftTurn, double turnRadius) {
	int subdivisions = 50;
	currentState = new TurnState(myInterface, isLeftTurn, turnRadius, subdivisions);
}

void BasicController::updateSpeed(float dt) {
	const double eSpeed = targetSpeed - mCar->getSpeedMPS();

	// Simple PID controller
	double thrBrkVal = 0;
	thrBrkVal += eSpeed * kPSpeed; // Proportional term

	thrBrkVal += kISpeed * iSpeedAcc; // Integral term
	iSpeedAcc += eSpeed * dt;

	if (dt != 0) {
		thrBrkVal += kDSpeed * ((eSpeed - dLastESpeed) / dt); // Derivative term
		dLastESpeed = eSpeed;
	}

	thrBrkVal = clamp(thrBrkVal, -1.0, 1.0); // Clamp from -1 to 1
	if (thrBrkVal < 0) {
		inputs[CarInput::THROTTLE] = 0;
		inputs[CarInput::BRAKE] = -thrBrkVal;
	} else {
		inputs[CarInput::THROTTLE] = thrBrkVal;
		inputs[CarInput::BRAKE] = 0;
	}
}

void BasicController::updateDirection(float dt) {
	const double angle = getAngle(toFlatVector(mCar->getForwardVector()), toFlatVector(initDir));
	if (isnan(angle)) return; // Abandon ship (maybe should find out where a nan might occur...)

	const double eAngle = targetAngle - angle;

	double steerVal = 0;
	steerVal += eAngle * kPAngle; // Proportional term

	steerVal += kIAngle * iAngleAcc; // Integral term
	iAngleAcc += eAngle * dt;

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