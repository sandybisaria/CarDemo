#include "RotationalFrame.hpp"

#ifdef COMPILE_UNIT_TESTS
#include <gtest/gtest.h>

TEST(RotationalFrame, RotationalFrameIntegration1) {
	RotationalFrame frame;

	Quaternion<double> initOrient;
	frame.setOrientation(initOrient);
	MathVector<double, 3> initV; initV.set(0.0);
	frame.setAngularVelocity(initV);
	MathVector<double, 3> torque; torque.set(0.0);
	frame.setInitialTorque(torque);

	// Integrate for 10 seconds
	for (int i = 0; i < 1000; i++) {
		frame.integrateStep1(0.01);

		torque.set(0, 1, 0);
		torque = torque - frame.getAngularVelocity() * 10.0f;
		frame.applyTorque(torque);

		frame.integrateStep2(0.01);
	}

	EXPECT_NEAR(frame.getAngularVelocity()[1], 0.1, 0.0001);
}

TEST(RotationalFrame, RotationalFrameIntegration2) {
	RotationalFrame frame;

	Quaternion<double> initOrient;
	frame.setOrientation(initOrient);
	MathVector<double, 3> initV; initV.set(0.0);
	frame.setAngularVelocity(initV);
	MathVector<double, 3> torque; torque.set(0, 1, 0);
	frame.setInitialTorque(torque);
	Matrix3<double> inertia; inertia.scale(0.1);
	frame.setInertia(inertia);

	// Integrate for 10 seconds
	for (int i = 0; i < 1000; i++) {
		frame.integrateStep1(0.01);
		frame.applyTorque(torque);
		frame.integrateStep2(0.01);
	}

	EXPECT_NEAR(frame.getAngularVelocity()[1], 100., 0.0001);
}

#endif
