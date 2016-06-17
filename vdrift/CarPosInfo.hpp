#pragma once

#include "CarConstants.hpp"

#include <OgreVector3.h>

class InfoSource {
public:
	virtual Ogre::Vector3 getPos() = 0;
};

class CarPosInfo {
public:
	CarPosInfo();
	void setSource(InfoSource* src);

	void update();

	void setPos(Ogre::Vector3 newPos) { pos = newPos; }
	Ogre::Vector3 getPos() { return pos; }

private:
	InfoSource* src;

	Ogre::Vector3 pos, carY;

	//TODO Should arrays be dynamic?
	Ogre::Vector3 wheelPos[MAX_WHEEL_COUNT];
	Ogre::Quaternion rot, wheelRot[MAX_WHEEL_COUNT];
};
