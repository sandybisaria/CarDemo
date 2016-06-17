#include "CarDynamics.hpp"
#include "CarConstants.hpp"

CarDynamics::CarDynamics() {
	setNumWheels(DEF_WHEEL_COUNT);
}

bool CarDynamics::loadFromConfig(ConfigFile& cf) {
	//TODO Examine CAR::Load and CARDYNAMICS::Load
	return true;
}

void CarDynamics::setNumWheels(int nw) {
	numWheels = nw;
}
