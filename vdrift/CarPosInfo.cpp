#include "CarPosInfo.hpp"

CarPosInfo::CarPosInfo()
	: mNew(false),
	  pos(0, 80, 0), carY(Ogre::Vector3::NEGATIVE_UNIT_Y),
	  rot(Ogre::Quaternion::IDENTITY) {
	for (int i = 0; i < MAX_WHEEL_COUNT; i++) {
		wheelPos[i] = Ogre::Vector3::ZERO;
		wheelRot[i] = Ogre::Quaternion::IDENTITY;
	}
}
