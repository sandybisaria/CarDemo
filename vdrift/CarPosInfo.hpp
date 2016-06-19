#pragma once

#include "CarConstants.hpp"

#include <OgreVector3.h>

class InfoSource {
public:
	virtual ~InfoSource() {}

	virtual Ogre::Vector3 getPos() = 0;
	virtual Ogre::Quaternion getRot() = 0;
	virtual std::vector<Ogre::Vector3> getWheelPos() = 0;
};

class CarPosInfo {
public:
	CarPosInfo();
	void setSource(InfoSource* src);
	void setNumWheels(int nw);

	void update();

	Ogre::Vector3 getPos() { return pos; }
	Ogre::Quaternion getRot() { return rot; }
	std::vector<Ogre::Vector3> getWheelPos() { return wheelPos; }

private:
	InfoSource* src;

	Ogre::Vector3 pos;

	int numWheels;
	std::vector<Ogre::Vector3> wheelPos;
	Ogre::Quaternion rot;
	std::vector<Ogre::Quaternion> wheelRot;
};
