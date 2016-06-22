#include "CarPosInfo.hpp"

#include "Axes.hpp"

CarPosInfo::CarPosInfo()
	: hasNew(false), pos(Ogre::Vector3::ZERO), rot(Ogre::Quaternion::IDENTITY) {
	for (int w = 0; w < MAX_WHEELS; w++) {
		wheelPos[w] = Ogre::Vector3::ZERO;
		wheelRot[w] = Ogre::Quaternion::IDENTITY;
	}
}

void CarPosInfo::fromCar(Car* car) {
	const CarDynamics* cd = &(car->cd);

	Axes::toOgre(pos, cd->getPosition());
	rot = Axes::toOgreD(cd->getRotation());

	// Wheels
	for (int w=0; w < cd->numWheels; ++w) {
		WHEEL_POSITION wp = WHEEL_POSITION(w);
		Axes::toOgre(wheelPos[w], cd->getWheelPosition(wp));
		wheelRot[w] = Axes::toOgreWD(cd->getWheelOrientation(wp));
	}
}
