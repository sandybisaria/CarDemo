#include "CarSuspension.hpp"

#ifdef COMPILE_UNIT_TESTS
#include <gtest/gtest.h>

TEST(CarSuspension, CarSuspensionFunctions) {
	CarSuspension suspension;

	// Front right suspension from 360
	suspension.setAntiRollK(8000);
	suspension.setBounce(10000.0);
	suspension.setCamber(-1.0);
	suspension.setCaster(6.12);
	suspension.setHinge(MathVector<double, 3>(-0.50, 0.90, -0.20));
	suspension.setRebound(9000.0);
	suspension.setSpringConstant(119865);
	suspension.setToe(0.2);
	suspension.setTravel(0.22);

	std::vector<std::pair<double, double> > damper, spring;
	damper.push_back(std::make_pair(0.06, 1.2));
	suspension.setDamperFactorPoints(damper);
	spring.push_back(std::make_pair(0.02, 0.2));
	spring.push_back(std::make_pair(0.08, 1.0));
	spring.push_back(std::make_pair(0.2, 1.5));
	spring.push_back(std::make_pair(0.4, 5.0));
	spring.push_back(std::make_pair(0.5, 15));
	suspension.setSpringFactorPoints(spring);

	EXPECT_NEAR(suspension.getDisplacement(), 0, 0.0001);
	EXPECT_NEAR(suspension.getOverTravel(), 0, 0.0001);
	EXPECT_NEAR(suspension.getForce(0.25, 0.06), -71889.8438, 0.0001);
	EXPECT_NEAR(suspension.getVelocity(), 0, 0.0001);

	EXPECT_NEAR(suspension.update(0.01, 0.2), 95959.5, 0.0001);
	EXPECT_NEAR(suspension.getDisplacement(), 0.2, 0.0001);
	EXPECT_NEAR(suspension.getOverTravel(), 0, 0.0001);
	EXPECT_NEAR(suspension.getForce(0.01, 0.03), -599.73, 0.0001);
	EXPECT_NEAR(suspension.getVelocity(), 5, 0.0001);

	EXPECT_NEAR(suspension.update(0.01, 0.3), 72785.055, 0.0001);
	EXPECT_NEAR(suspension.getDisplacement(), 0.22, 0.0001);
	EXPECT_NEAR(suspension.getOverTravel(), 0.08, 0.0001);
	EXPECT_NEAR(suspension.getForce(0.1, 0.01), -13105.375, 0.0001);
	EXPECT_NEAR(suspension.getVelocity(), 2, 0.0001);

	EXPECT_NEAR(suspension.update(0.01, -0.21), -54000, 0.0001);
	EXPECT_NEAR(suspension.getDisplacement(), 0, 0.0001);
	EXPECT_NEAR(suspension.getOverTravel(), 0, 0.0001);
	EXPECT_NEAR(suspension.getForce(0.2, 0.06), -36679.5, 0.0001);
	EXPECT_NEAR(suspension.getVelocity(), -5, 0.0001);
}

#endif
