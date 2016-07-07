#include "BasicController.hpp"

BasicController::BasicController(Car* car)
	: mCar(car),
	  targetSpeed(10), kPSpeed(8), kDSpeed(0.001), kISpeed(0.001) {
	reset();
}

BasicController::~BasicController() {
}

void BasicController::reset() {
	inputs.resize(CarInput::ALL, 0.0f);

	dLastESpeed = 0;
	iSpeedAcc = 0;
}

void BasicController::setTargetSpeed(double newSpeed) {
	targetSpeed = newSpeed;
}

const std::vector<float>& BasicController::updateInputs(float dt) {
	// Set speed
	const double eSpeed = targetSpeed - mCar->getSpeedMPS();

	float thrBrkVal = 0;
	thrBrkVal += eSpeed * kPSpeed; // Proportionality term

	if (dt != 0) {
		thrBrkVal += kDSpeed * ((eSpeed - dLastESpeed) / dt); // Derivative term
		dLastESpeed = eSpeed;
	}

	thrBrkVal += kISpeed * iSpeedAcc;
	iSpeedAcc += eSpeed * dt;

	thrBrkVal = clamp(thrBrkVal, -1.f, 1.f); // Clamp from -1 to 1
	if (fabs(thrBrkVal) < 0.01) thrBrkVal = 0; // Output is so small as to be negligible
	if (thrBrkVal < 0) {
		inputs[CarInput::THROTTLE] = 0;
		inputs[CarInput::BRAKE] = -thrBrkVal;
	} else {
		inputs[CarInput::THROTTLE] = thrBrkVal;
		inputs[CarInput::BRAKE] = 0;
	}

	return inputs;
}
