#include "LinearInterp.hpp"

#ifdef COMPILE_UNIT_TESTS
#include <gtest/gtest.h>

TEST(LinearInterp, LinearInterpFunctions) {
	LinearInterp<float> l1;
	EXPECT_NEAR(l1.interpolate(1), 0, 0.0001);

	LinearInterp<float> l2(3.1);
	EXPECT_NEAR(l2.interpolate(1), 3.1, 0.0001);

	LinearInterp<float> l3; l3.addPoint(2, 1);
	EXPECT_NEAR(l3.interpolate(1), 1, 0.0001);
	EXPECT_NEAR(l3.interpolate(2), 1, 0.0001);
	EXPECT_NEAR(l3.interpolate(3), 1, 0.0001);

	LinearInterp <float> l4; l4.addPoint(2, 1); l4.addPoint(3, 2);
	l4.setBoundaryMode(LinearInterp<float>::CONSTANTSLOPE);
	EXPECT_NEAR(l4.interpolate(1), 0, 0.0001);
	EXPECT_NEAR(l4.interpolate(0), -1, 0.0001);
	EXPECT_NEAR(l4.interpolate(2.5), 1.5, 0.0001);
	EXPECT_NEAR(l4.interpolate(2.75), 1.75, 0.0001);
	EXPECT_NEAR(l4.interpolate(3), 2, 0.0001);
	EXPECT_NEAR(l4.interpolate(3.5), 2.5, 0.0001);
}

#endif
