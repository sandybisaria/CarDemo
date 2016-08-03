#include "BasicController.hpp"

void ControllerInterface::setSteering(double steering) {
	steering = clamp(steering, -1.0, 1.0);
	if (steering < 0) {
		mController->inputs[CarInput::STEER_RIGHT] = 0;
		mController->inputs[CarInput::STEER_LEFT] = -steering;
	} else {
		mController->inputs[CarInput::STEER_RIGHT] = steering;
		mController->inputs[CarInput::STEER_LEFT] = 0;
	}

	mController->dirAlreadyUpdated = true;
}

MathVector<double, 2> ControllerInterface::getCarPosition() {
	return toFlatVector(Axes::ogreToMath(mController->mCar->getPosition()), false);
}

// Direction is normalized
MathVector<double, 2> ControllerInterface::getCarDirection() {
	return toFlatVector(mController->mCar->getForwardVector());
}

double ControllerInterface::getCarSpeed() {
	return mController->mCar->getSpeed();
}