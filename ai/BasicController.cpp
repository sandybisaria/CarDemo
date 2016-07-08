#include "BasicController.hpp"

BasicController::BasicController(Car* car)
	: mCar(car),
	  targetSpeed(20), kPSpeed(7.65629), kDSpeed(0.00020), kISpeed(0.00656) {
	reset();
}

BasicController::~BasicController() {
}

void BasicController::reset() {
	inputs.resize(CarInput::ALL, 0.0f);

	// Speed variables
	dLastESpeed = 0;
	iSpeedAcc = 0;
}

void BasicController::setTargetSpeed(double newSpeed) {
	targetSpeed = newSpeed;
}

const std::vector<double>& BasicController::updateInputs(float dt) {
	updateSpeed(dt);

	return inputs;
}

void BasicController::updateSpeed(float dt) {
	const double eSpeed = targetSpeed - mCar->getSpeedMPS();

	double thrBrkVal = 0;
	thrBrkVal += eSpeed * kPSpeed; // Proportionality term

	if (dt != 0) {
		thrBrkVal += kDSpeed * ((eSpeed - dLastESpeed) / dt); // Derivative term
		dLastESpeed = eSpeed;
	}

	thrBrkVal += kISpeed * iSpeedAcc;
	iSpeedAcc += eSpeed * dt;

	thrBrkVal = clamp(thrBrkVal, -1.0, 1.0); // Clamp from -1 to 1
	if (thrBrkVal < 0) {
		inputs[CarInput::THROTTLE] = 0;
		inputs[CarInput::BRAKE] = -thrBrkVal;
	} else {
		inputs[CarInput::THROTTLE] = thrBrkVal;
		inputs[CarInput::BRAKE] = 0;
	}
}
