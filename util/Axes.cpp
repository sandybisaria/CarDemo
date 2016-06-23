#include "Axes.hpp"

Ogre::Quaternion Axes::qFixCar, Axes::qFixWh;

void Axes::init() {
	Ogre::Quaternion qr;
	{ // For qFixCar
		Quaternion<double> fix; fix.rotate(M_PI, 0, 1, 0);
		qr.w = fix.w(); qr.x = fix.x(); qr.y = fix.y(); qr.z = fix.z();
		qFixCar = qr;
	}
	{ // For qFixWh
		Quaternion<double> fix; fix.rotate(M_PI / 2, 0, 1, 0);
		qr.w = fix.w(); qr.x = fix.x(); qr.y = fix.y(); qr.z = fix.z();
		qFixWh = qr;
	}
}

void Axes::vectorToOgre(Ogre::Vector3& vOut, const MathVector<float, 3>& vIn) {
	vOut.x = vIn[0];  vOut.y = vIn[2];  vOut.z = -vIn[1];
}

Ogre::Vector3 Axes::vectorToOgre(const MathVector<float, 3>& vIn) {
	return Ogre::Vector3(vIn[0], vIn[2], -vIn[1]);
}

// For the car
Ogre::Quaternion Axes::flQuatToOgre(const Quaternion<float>& qIn) {
	Ogre::Quaternion q(qIn[0], -qIn[3], qIn[1], qIn[2]);
	return q * qFixCar;
}

Ogre::Quaternion Axes::doQuatToOgre(const Quaternion<double>& qIn) {
	Ogre::Quaternion q(qIn[0], -qIn[3], qIn[1], qIn[2]);
	return q * qFixCar;
}

// For the wheels
Ogre::Quaternion Axes::flWhQuatToOgre(const Quaternion<float>& qIn) {
	Ogre::Quaternion q(qIn[0], -qIn[3], qIn[1], qIn[2]);
	return q * qFixWh;
}

Ogre::Quaternion Axes::doWhQuatToOgre(const Quaternion<double>& qIn) {
	Ogre::Quaternion q(qIn[0], -qIn[3], qIn[1], qIn[2]);
	return q * qFixWh;
}
