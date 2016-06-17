#pragma once

#include "CarConstants.hpp"

#include <OgreVector3.h>

class CarPosInfo {
public:
	CarPosInfo();

	// For code readability
	inline bool hasNew() { return mNew; }
	inline void markAsRead() { mNew = false; }
	inline void onUpdate() { mNew = true; };

private:
	bool mNew; // True if updated with new info

	Ogre::Vector3 pos, carY;

	//TODO Should arrays be dynamic?
	Ogre::Vector3 wheelPos[MAX_WHEEL_COUNT];
	Ogre::Quaternion rot, wheelRot[MAX_WHEEL_COUNT];
};
