#include "RigidBody.hpp"

#ifdef COMPILE_UNIT_TESTS
#include <gtest/gtest.h>

#include <cmath>
#define _USE_MATH_DEFINES

TEST(RigidBody, RigidBodyFunctions) {
	RigidBody body;

	MathVector<double, 3> initPos; initPos.set(0, 0, 10);
	body.setPosition(initPos);
	Quaternion<double> quat; quat.rotate(-M_PI * 0.5, 1, 0, 0);
	body.setOrientation(quat);

	MathVector<float, 3> localCoords; localCoords.set(0, 0, 1);
	MathVector<float, 3> expected; expected.set(0, 1, 10);
	MathVector<float, 3> pos = body.transformLocalToWorld(localCoords);

	EXPECT_NEAR(pos[0], expected[0], 0.0001);
	EXPECT_NEAR(pos[1], expected[1], 0.0001);
	EXPECT_NEAR(pos[2], expected[2], 0.0001);

	EXPECT_NEAR(body.transformWorldToLocal(pos)[0], localCoords[0], 0.0001);
	EXPECT_NEAR(body.transformWorldToLocal(pos)[1], localCoords[1], 0.0001);
	EXPECT_NEAR(body.transformWorldToLocal(pos)[2], localCoords[2], 0.0001);
}

#endif
