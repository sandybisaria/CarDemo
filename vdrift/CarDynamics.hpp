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

	void setNumWheels(int nw);

private:
	int numWheels;

	Ogre::Vector3 pos;
};
