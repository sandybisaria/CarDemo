#pragma once

#include "../util/ConfigFile.hpp"
#include "CarPosInfo.hpp"

#include <OgreVector3.h>

class CarDynamics
	: public InfoSource {
public:
	CarDynamics();

	bool loadFromConfig(ConfigFile& cf);

	void setPos(Ogre::Vector3 newPos) { pos = newPos; }
	Ogre::Vector3 getPos() { return pos; }

	void setRot(Ogre::Quaternion newRot) { rot = newRot; }
	Ogre::Quaternion getRot() { return rot; }

	void setNumWheels(int nw);

private:
	int numWheels;

	Ogre::Vector3 pos;
	Ogre::Quaternion rot;
};
