#include "CarTransmission.hpp"

#ifdef COMPILE_UNIT_TESTS
#include <gtest/gtest.h>

TEST(CarTransmission, CarTransmissionFunctions) {
	CarTransmission transmission;

	transmission.setGearRatio(-1, -3.29);
	transmission.setGearRatio( 1, 3.29);
	transmission.setGearRatio( 2, 2.16);
	transmission.setGearRatio( 3, 1.61);
	transmission.setGearRatio( 4, 1.27);
	transmission.setGearRatio( 5, 1.03);
	transmission.setGearRatio( 6, 0.85);

	EXPECT_EQ(transmission.getForwardGears(), 6); EXPECT_EQ(transmission.getReverseGears(), 1);
	EXPECT_EQ(transmission.getGear(), 0); EXPECT_EQ(transmission.getCurrentGearRatio(), 0);

	transmission.shift(4);
	EXPECT_EQ(transmission.getGear(), 4); EXPECT_NEAR(transmission.getCurrentGearRatio(), 1.27, 0.0001);
	EXPECT_NEAR(transmission.calculateClutchSpeed(100), 127, 0.0001);
	EXPECT_NEAR(transmission.getClutchSpeed(50), 63.5, 0.0001);
	EXPECT_NEAR(transmission.getTorque(20), 25.4, 0.0001);

	transmission.shift(-2);
	EXPECT_EQ(transmission.getGear(), 4); EXPECT_NEAR(transmission.getCurrentGearRatio(), 1.27, 0.0001);

	transmission.shift(-1);
	EXPECT_EQ(transmission.getGear(), -1); EXPECT_NEAR(transmission.getCurrentGearRatio(), -3.29, 0.0001);
	EXPECT_NEAR(transmission.calculateClutchSpeed(50), -164.5, 0.0001);
	EXPECT_NEAR(transmission.getClutchSpeed(100), -329, 0.0001);
	EXPECT_NEAR(transmission.getTorque(2), -6.58, 0.0001);
}

#endif
