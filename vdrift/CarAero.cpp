#include "CarAero.hpp"

#ifdef COMPILE_UNIT_TESTS
#include <gtest/gtest.h>

TEST(CarAero, CarAeroFunctions) {
	CarAero aero;

	MathVector<double, 3> force(0);
	EXPECT_NEAR(aero.getAerodynamicDragCoefficient(), 0, 0.0001);
	EXPECT_NEAR(aero.getAerodynamicDownforceCoefficient(), 0, 0.0001);
	EXPECT_NEAR(aero.getForce(force)[0], 0, 0.0001);
	EXPECT_NEAR(aero.getForce(force)[1], 0, 0.0001);
	EXPECT_NEAR(aero.getForce(force)[2], 0, 0.0001);

	force.set(1, 1, 1);

	MathVector<double, 3> dragPos(0.0, 0.15, 0.0);
	aero.set(dragPos, 1.6, 0.2, 0, 0, 0);
	EXPECT_NEAR(aero.getAerodynamicDragCoefficient(), 0.192, 0.0001);
	EXPECT_NEAR(aero.getAerodynamicDownforceCoefficient(), 0, 0.0001);
	EXPECT_NEAR(aero.getForce(force)[0], 0.332554, 0.0001);
	EXPECT_NEAR(aero.getForce(force)[1], 0.332554, 0.0001);
	EXPECT_NEAR(aero.getForce(force)[2], 0.332554, 0.0001);

	MathVector<double, 3> wingFrontPos(0.0, 1.45, 0.2);
	aero.set(wingFrontPos, 0.0, 0, 0.8, -3.6, 0.95);
	EXPECT_NEAR(aero.getAerodynamicDragCoefficient(), 0.31104, 0.0001);
	EXPECT_NEAR(aero.getAerodynamicDownforceCoefficient(), -1.728, 0.0001);
	EXPECT_NEAR(aero.getForce(force)[0], -0.0124416, 0.0001);
	EXPECT_NEAR(aero.getForce(force)[1], 		  0, 0.0001);
	EXPECT_NEAR(aero.getForce(force)[2],   -0.06912, 0.0001);
}

#endif
