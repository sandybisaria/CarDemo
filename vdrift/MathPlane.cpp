#include "MathPlane.hpp"

#ifdef COMPILE_UNIT_TESTS
#include <gtest/gtest.h>

TEST(MathPlane, MathPlaneFunctions) {
	EXPECT_NEAR(MathPlane<float> (0,1,0,0).distanceToPoint(MathVector<float,3> (0,0,0)), 0.0f, 0.0001f);
	EXPECT_NEAR(MathPlane<float> (0,1,0,0).distanceToPoint(MathVector<float,3> (1,1,1)), 1.0f, 0.0001f);
	EXPECT_NEAR(MathPlane<float> (0,1,0,0).distanceToPoint(MathVector<float,3> (1,-1,1)), -1.0f, 0.0001f);
	EXPECT_NEAR(MathPlane<float> (0,1,0,-3).distanceToPoint(MathVector<float,3> (100,3,-40)), 0.0f, 0.0001f);
}

#endif
