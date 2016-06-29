#include "CarBrake.hpp"

#ifdef COMPILE_UNIT_TESTS
#include <gtest/gtest.h>

TEST(CarBrake, CarBrakeFunctions) {
	CarBrake brake;
	EXPECT_EQ(brake.getTorque(), 0);

	brake.setBrakeFactor(1.f);
	EXPECT_NEAR(brake.getTorque(), 3066, 0.0001f);

	brake.setHandbrakeFactor(1.f);
	EXPECT_NEAR(brake.getTorque(), 3066, 0.0001f);

	brake.setHandbrake(2.6f);
	EXPECT_NEAR(brake.getTorque(), 7971.59971, 0.0001f);

	brake.setBrakeFactor(0.f);
	EXPECT_NEAR(brake.getTorque(), 7971.59971, 0.0001f);

	brake.setHandbrakeFactor(0.f);
	EXPECT_EQ(brake.getTorque(), 0);
}

#endif
