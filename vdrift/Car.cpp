#include "Car.hpp"

#include "../util/ConfigFile.hpp"

Car::Car() {
	setNumWheels(4); // Default to four wheels
}

Car::~Car() {

}

void Car::setup(std::string carName) {
	mCarName = carName;

	// Top-level directory for all car data
	std::string carPath = "../data/cars/" + mCarName;
	std::string carSimPath = carPath + "/sim/" + mCarName + ".car";

	ConfigFile cf;
	if (!cf.load(carSimPath)) {
		return; //TODO Error if car not found
	}
}

void Car::setNumWheels(int n) {
	numWheels = n;

	/*TODO Keep track of the following in vectors:
	 * Wheel position/radius/width/trail/temperature
	 * SceneNodes* for "Wh" (Wheel), "WhE" (Wheel Emitter), "Brake"
	 * Suspension bump detection, "Last bezier patch that each wheel hit"
	 */
}
