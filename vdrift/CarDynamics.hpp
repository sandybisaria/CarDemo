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

	void setWheelPos(std::vector<Ogre::Vector3> newWP) { wheelPos = newWP; }
	std::vector<Ogre::Vector3> getWheelPos() { return wheelPos; }

	void setNumWheels(int nw);

private:
	int numWheels;

	// Convert from v2 .car format to v1
	static void versionConvert(float& x, float& y, float& z);

	Ogre::Vector3 pos;
	Ogre::Quaternion rot;
	std::vector<Ogre::Vector3> wheelPos;
};
