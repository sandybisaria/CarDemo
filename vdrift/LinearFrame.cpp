#include "LinearFrame.hpp"

#ifdef COMPILE_UNIT_TESTS
#include <gtest/gtest.h>

TEST(LinearFrame, LinearFrameFunctions) {
	LinearFrame frame; frame.setMass(1.0);
	MathVector<double, 3> initPos; initPos.set(0.0);
	frame.setPosition(initPos);
	MathVector<double, 3> initV; initV.set(0, 65, 0);
	frame.setVelocity(initV);
	MathVector<double, 3> gravity; gravity.set(0, -9.81, 0);
	frame.setInitialForce(gravity);

	double t = 0.0;
	// Integrate for 10 seconds
	for (int i = 0; i < 1000; i++) {
		frame.integrateStep1(0.01);
		frame.applyForce(gravity);
		frame.integrateStep2(0.01);
		t += 0.01;
	}
	EXPECT_NEAR(frame.getPosition()[1], (initV * t + gravity * t * t * 0.5)[1], 0.0001);

	frame.setMass(1.0);
	initPos.set(0, 0, 0); frame.setPosition(initPos);
	initV.set(0, 0, 0); frame.setVelocity(initV);
	MathVector<double, 3> force; force.set(0.0);
	frame.setInitialForce(force);

	// Integrate for 10 seconds
	for (int i = 0; i < 1000; i++) {
		frame.integrateStep1(0.01);

		force.set(0, 1, 0); force = force - frame.getVelocity() * 10.0f;
		frame.applyForce(force);

		frame.integrateStep2(0.01);
	}

	EXPECT_NEAR(frame.getVelocity()[1], 0.1, 0.0001);
}

#endif
