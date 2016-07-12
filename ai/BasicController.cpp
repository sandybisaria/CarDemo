#include "BasicController.hpp"

BasicController::BasicController(Car* car)
	: mCar(car) {
	reset();

	setTargetSpeed(10);
	kPSpeed = 7.65629; kISpeed = 0.00656; kDSpeed = 0.00020;

	// Could be refined further; may also be correlated with speed
	kPAngle = 443.75; kIAngle = 1; kDAngle = 1;
	setTargetAngle(0);
}

BasicController::~BasicController() { }

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
}

void BasicController::setTargetAngle(double newAngle) {
	targetAngle = newAngle;
	initDir = mCar->getForwardVector();
}

const std::vector<double>& BasicController::updateInputs(float dt) {
	updateSpeed(dt);
	updateDirection(dt);

	return inputs;
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
	MathVector<double, 2> initDirFlat = toFlatVector(initDir);
	MathVector<double, 2> currentDirFlat = toFlatVector(mCar->getForwardVector());
	double angle = acos(initDirFlat.dot(currentDirFlat));

	MathVector<double, 3> cross = initDir.cross(mCar->getForwardVector());
	if (cross.dot(MathVector<double, 3>(0, 0, 1)) > 0) { // Use "> 0" because angle dir is reversed
		angle = -angle; // Determine direction of angle (i.e. to the left or to the right of initDir)
	}

	if (isnan(angle)) return; // Abandon ship (maybe should find out where a nan might occur...)

	double eAngle = targetAngle - angle;

	double steerVal = 0;
	steerVal += eAngle * kPAngle; // Proportional term

	steerVal += kIAngle * iAngleAcc; // Integral term
	iAngleAcc += eAngle * dt;

	if (dt != 0) {
		steerVal += kDAngle * ((eAngle - dLastEAngle) / dt); // Derivative term
		dLastEAngle = eAngle;
	}

	steerVal = clamp(steerVal, -1.0, 1.0); // Clamp from -1 to 1
//	std::cout << targetAngle << " " << angle << " " << eAngle << " " << steerVal;

	// Within some epsilon, give no steering input (prevent jittery steering)
	/*if (fabs(steerVal) < 0.0001) { // May n
		inputs[CarInput::STEER_RIGHT] = 0;
		inputs[CarInput::STEER_LEFT] = 0;
		std::cout << std::endl;
	} else*/
	if (steerVal < 0) {
		inputs[CarInput::STEER_RIGHT] = 0;
		inputs[CarInput::STEER_LEFT] = -steerVal;
//		std::cout << " LEFT" << std::endl;
	} else {
		inputs[CarInput::STEER_RIGHT] = steerVal;
		inputs[CarInput::STEER_LEFT] = 0;
//		std::cout << " RIGHT" << std::endl;
	}
}

// "Flat" in that height is ignored; return vector is normalized.
MathVector<double, 2> BasicController::toFlatVector(MathVector<double, 3> vec) {
	return MathVector<double, 2>(vec[0], vec[1]).normalized();
}