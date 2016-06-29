#include "CarClutch.hpp"

#ifdef COMPILE_UNIT_TESTS
#include <gtest/gtest.h>

TEST(CarClutch, CarClutchFunctions) {
	CarClutch clutch;
	EXPECT_NEAR(clutch.getTorque(100, 12312), 0, 0.0001);
	EXPECT_NEAR(clutch.getTorque(1123123100, 1.23), 0, 0.0001);

	clutch.setClutch(0.5);
	EXPECT_NEAR(clutch.getTorque(10012, 6237), 168.265, 0.0001);
	EXPECT_NEAR(clutch.getTorque(0.234231, 1.233), -168.057866, 0.0001);

	clutch.setMaxTorque(50);
	EXPECT_NEAR(clutch.getTorque(65, 4567), -25, 0.0001);
	EXPECT_NEAR(clutch.getTorque(0.1423, 314159), -25, 0.0001);
}

#endif
