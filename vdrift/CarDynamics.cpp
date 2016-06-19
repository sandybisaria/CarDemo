#include "CarDynamics.hpp"
#include "CarConstants.hpp"

CarDynamics::CarDynamics() {
	setNumWheels(DEF_WHEEL_COUNT);
}

bool CarDynamics::loadFromConfig(ConfigFile& cf) {
	//TODO Examine CAR::Load and CARDYNAMICS::Load

	int version = 2; // Assume version 2
	cf.getParam("version", version);

	// Load wheel positions
	for (int w = 0; w < numWheels; w++) {
		std::string wheelType = WHEEL_TYPE[w];

		float pos[ConfigVariable::V_SIZE];
		cf.getParam("wheel-" + wheelType + ".position", pos);

		if (version == 2)
			versionConvert(pos[0], pos[1], pos[2]);

		Ogre::Vector3 wp(pos[0], pos[1], pos[2]);
		wheelPos[w] = wp;

		std::cout << pos[0] << pos[1] << pos[2] << std::endl;
	}

	return true;
}

void CarDynamics::setNumWheels(int nw) {
	numWheels = nw;
	wheelPos.resize(numWheels);
}

void CarDynamics::versionConvert(float& x, float& y, float& z) {
	// From CarModel_Create.cpp ConvertV2to1()
	float tx = x, ty = y, tz = z;
	x = ty;  y = -tx;  z = tz;
}
