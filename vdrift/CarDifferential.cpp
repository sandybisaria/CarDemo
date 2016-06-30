#include "CarDifferential.hpp"

#ifdef COMPILE_UNIT_TESTS
#include <gtest/gtest.h>

TEST(CarDifferential, CarDifferentialFunctions) {
	CarDifferential differential;

	EXPECT_NEAR(differential.getDriveshaftSpeed(), 0, 0.0001); EXPECT_NEAR(differential.getFinalDrive(), 4.1, 0.0001);
	EXPECT_NEAR(differential.calculateDriveshaftSpeed(50, 40), 184.5, 0.0001);

	differential.computeWheelTorques(150);
	EXPECT_NEAR(differential.getSide1Speed(), 50, 0.0001); EXPECT_NEAR(differential.getSide2Torque(), 907.5, 0.0001);

	differential.setFinalDrive(4.54); differential.setAntiSlip(600.0, 0, 0);

	EXPECT_NEAR(differential.getDriveshaftSpeed(), 204.3, 0.0001); EXPECT_NEAR(differential.getFinalDrive(), 4.54, 0.0001);
	EXPECT_NEAR(differential.calculateDriveshaftSpeed(60, 70), 295.1, 0.0001);

	differential.computeWheelTorques(200);
	EXPECT_NEAR(differential.getSide2Speed(), 70, 0.0001); EXPECT_NEAR(differential.getSide1Torque(), 1054, 0.0001);
}

#endif
