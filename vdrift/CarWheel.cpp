#include "CarWheel.hpp"

#ifdef COMPILE_UNIT_TESTS
#include <gtest/gtest.h>

TEST(CarWheel, CarWheelFunctions) {
	CarWheel wheel;
	wheel.setInitialConditions();

	// Integrate for 10 seconds
	for (int i = 0; i < 1000; i++) {
		wheel.integrateStep1(0.01);
		wheel.setTorque(100);
		wheel.integrateStep2(0.01);
	}
	MathVector<double, 3> zero(0.f);
	EXPECT_NEAR(wheel.getAngularVelocity(), 100, 0.0001); EXPECT_EQ(wheel.getExtendedPosition(), zero);
	EXPECT_NEAR(wheel.getInertia(), 10, 0.0001); EXPECT_NEAR(wheel.getMass(), 18.1, 0.0001);
	EXPECT_NEAR(wheel.getTorque(), 100, 0.0001); EXPECT_NEAR(wheel.getRPM(), 0, 0.0001);

	Quaternion<double> orient = wheel.getOrientation();
	EXPECT_NEAR(orient.x(), 0, 0.0001); EXPECT_NEAR(orient.y(),  0.985206093, 0.0001);
	EXPECT_NEAR(orient.z(), 0, 0.0001); EXPECT_NEAR(orient.w(), -0.171373725, 0.0001);

	wheel.setCamberDeg(2); wheel.setExtendedPosition(MathVector<double, 3>(0.8345, 1.12, -0.37));
	wheel.setMass(15.0); wheel.setRadius(0.32); wheel.setRollHeight(0.92);
	wheel.setRollingResistance(1.3e-2, 6.5e-5); wheel.setSteerAngle(5);

	// Integrate for 10 seconds
	for (int i = 0; i < 1000; i++) {
		wheel.integrateStep1(0.01);
		wheel.setTorque(wheel.getLockUpTorque(0.01) * 0.5);
		wheel.integrateStep2(0.01);
	}
	MathVector<double, 3> expected(0.8345, 1.12, -0.37), actual(wheel.getExtendedPosition());
	EXPECT_NEAR(wheel.getAngularVelocity(), 9.33263619e-300, 0.0001);
	EXPECT_NEAR(actual[0], expected[0], 0.0001); EXPECT_NEAR(actual[1], expected[1], 0.0001); EXPECT_NEAR(actual[2], expected[2], 0.0001);
	EXPECT_NEAR(wheel.getInertia(), 10, 0.0001); EXPECT_NEAR(wheel.getMass(), 15, 0.0001);
	EXPECT_NEAR(wheel.getTorque(), -9.33263619e-297, 0.0001); EXPECT_NEAR(wheel.getRPM(), 0, 0.0001);

	orient = wheel.getOrientation();
	EXPECT_NEAR(orient.x(), 0, 0.0001); EXPECT_NEAR(orient.y(),  0.618840906, 0.0001);
	EXPECT_NEAR(orient.z(), 0, 0.0001); EXPECT_NEAR(orient.w(), -0.785516348, 0.0001);
}

#endif
