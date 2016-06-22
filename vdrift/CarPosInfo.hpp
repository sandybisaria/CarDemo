#pragma once

#include "CarConstants.hpp"

#include <OgreVector3.h>

class CarPosInfo {
public:
	CarPosInfo();

	Ogre::Vector3 getPos() { return pos; }
	Ogre::Quaternion getRot() { return rot; }

private:
	Ogre::Vector3 pos;
	Ogre::Quaternion rot;

	Ogre::Vector3 wheelPos[MAX_WHEEL_COUNT];
	Ogre::Quaternion wheelRot[MAX_WHEEL_COUNT];
};
