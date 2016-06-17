#pragma once

#include "CarConstants.hpp"

#include <OgreVector3.h>

class InfoSource {
public:
	virtual Ogre::Vector3 getPos() = 0;
	virtual Ogre::Quaternion getRot() = 0;
};

class CarPosInfo {
public:
	CarPosInfo();
	void setSource(InfoSource* src);

	void update();

	Ogre::Vector3 getPos() { return pos; }

	Ogre::Quaternion getRot() { return rot; }

private:
	void setPos(Ogre::Vector3 newPos) { pos = newPos; }
	void setRot(Ogre::Quaternion newRot) { rot = newRot; }

	InfoSource* src;

	Ogre::Vector3 pos;

	// Static b/c CarPosInfo does not care how many wheels there are
	Ogre::Vector3 wheelPos[MAX_WHEEL_COUNT];
	Ogre::Quaternion rot, wheelRot[MAX_WHEEL_COUNT];
};
