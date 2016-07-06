#include "BasicController.hpp"

BasicController::BasicController(Car* car)
	: mCar(car),
	  targetSpeed(0) {
}

BasicController::~BasicController() {
}

void BasicController::reset() {
}
