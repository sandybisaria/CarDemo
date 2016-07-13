#include "BasicController.hpp"

MathVector<double, 2> ControllerInterface::getCarPosition() {
	return toFlatVector(Axes::ogreToMath(mController->mCar->getPosition()), false);
}

// Direction is normalized
MathVector<double, 2> ControllerInterface::getCarDirection() {
	return toFlatVector(mController->mCar->getForwardVector());
}

double ControllerInterface::getCarSpeed() {
	return mController->mCar->getSpeedMPS();
}