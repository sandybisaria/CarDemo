#include "Axes.hpp"

Ogre::Quaternion Axes::qFixCar, Axes::qFixWh;

void Axes::init() {
	Quaternion<float> fix, fix2;
	Ogre::Quaternion qr;

	// For qFixCar
	fix.rotate((float) M_PI, 0, 1, 0);
	qr.w = fix.w(); qr.x = fix.x(); qr.y = fix.y(); qr.z = fix.z();
	qFixCar = qr;

	// For qFixWh
	fix2.rotate((float) M_PI / 2, 0, 1, 0);
	qr.w = fix2.w(); qr.x = fix2.x(); qr.y = fix2.y(); qr.z = fix2.z();
	qFixWh = qr;
}

void Axes::vectorToOgre(Ogre::Vector3& vOut, const MathVector<float, 3>& vIn) {
	vOut.x = vIn[0];  vOut.y = vIn[2];  vOut.z = -vIn[1];
}

Ogre::Vector3 Axes::vectorToOgre(const MathVector<float, 3>& vIn) {
	return Ogre::Vector3(vIn[0], vIn[2], -vIn[1]);
}

MathVector<float, 3> Axes::ogreToMath(const Ogre::Vector3 vIn) {
	return MathVector<float, 3>(vIn.x, -vIn.z, vIn.y);
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


#ifdef COMPILE_UNIT_TESTS
#include <gtest/gtest.h>

#define EXPECT_NEAR_HP(a, b) EXPECT_NEAR(a, b, 0.0001)

TEST(Axes, AxesFunctions) {
	Axes::init();

	EXPECT_NEAR_HP(Axes::qFixCar.w, 5.55111512e-17);
	EXPECT_NEAR_HP(Axes::qFixCar.x, 5.72118892e-18);
	EXPECT_NEAR_HP(Axes::qFixCar.y, 			 1);
	EXPECT_NEAR_HP(Axes::qFixCar.z, 			 0);

	EXPECT_NEAR_HP(Axes::qFixWh.w, 	  0.707106769);
	EXPECT_NEAR_HP(Axes::qFixWh.x, 1.11022302e-16);
	EXPECT_NEAR_HP(Axes::qFixWh.y,	  0.707106769);
	EXPECT_NEAR_HP(Axes::qFixWh.z, 				0);

	Ogre::Vector3 vOut; Axes::vectorToOgre(vOut, MathVector<float, 3>(2, -4, 56));
	EXPECT_EQ(Axes::vectorToOgre(MathVector<float, 3>(2, 3, 4)), Ogre::Vector3(2, 4, -3));
	EXPECT_EQ(Ogre::Vector3(2, 56, 4), vOut);

	Quaternion<float> forCarIn(1, 2, 3, 5), forWheelIn(3, 4, 2, 1);
	Ogre::Quaternion forCar = Axes::flQuatToOgre(forCarIn), forWheel = Axes::flWhQuatToOgre(forWheelIn);
	Ogre::Quaternion fc(-2, -3, 1, -5), fw(-0.707106829, -2.12132025, 4.94974709, 0.707106769);
	EXPECT_NEAR_HP(fc.w, forCar.w);
	EXPECT_NEAR_HP(fc.x, forCar.x);
	EXPECT_NEAR_HP(fc.y, forCar.y);
	EXPECT_NEAR_HP(fc.z, forCar.z);

	EXPECT_NEAR_HP(fw.w, forWheel.w);
	EXPECT_NEAR_HP(fw.x, forWheel.x);
	EXPECT_NEAR_HP(fw.y, forWheel.y);
	EXPECT_NEAR_HP(fw.z, forWheel.z);

	Quaternion<double> forCar1In(1, 2.87, 3.8788, 5), forWheel1In(3.8, 4, 26.68, 14);
	Ogre::Quaternion forCar1 = Axes::doQuatToOgre(forCar1In), forWheel1 = Axes::doWhQuatToOgre(forWheel1In);
	Ogre::Quaternion fc1(-2.86999989, -3.87879992, 1, -5), fw1(-0.141421318, -28.7651024, 5.51543283, 8.96611309);
	EXPECT_NEAR_HP(fc1.w, forCar1.w);
	EXPECT_NEAR_HP(fc1.x, forCar1.x);
	EXPECT_NEAR_HP(fc1.y, forCar1.y);
	EXPECT_NEAR_HP(fc1.z, forCar1.z);

	EXPECT_NEAR_HP(fw1.w, forWheel1.w);
	EXPECT_NEAR_HP(fw1.x, forWheel1.x);
	EXPECT_NEAR_HP(fw1.y, forWheel1.y);
	EXPECT_NEAR_HP(fw1.z, forWheel1.z);
}

#endif
