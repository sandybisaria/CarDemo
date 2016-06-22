#pragma once

#include "vdrift/Car.hpp"
#include "vdrift/cardefs.h"

#include <OgreVector3.h>

class CarPosInfo {
public:
	bool hasNew;

	// Car params
	Ogre::Vector3 pos;
	Ogre::Quaternion rot;

	// Wheel params
	Ogre::Vector3 wheelPos[MAX_WHEELS];
	Ogre::Quaternion wheelRot[MAX_WHEELS];

	CarPosInfo();

	void fromCar(Car* car); // Pull data from Car
};
