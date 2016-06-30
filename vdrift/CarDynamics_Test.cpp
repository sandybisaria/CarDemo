#include "CarDynamics.hpp"

#ifdef COMPILE_UNIT_TESTS
#include <gtest/gtest.h>

TEST(CarDynamics, CarDynamicsCalculateMass) {
	CarDynamics cd; cd.massOnlyParticles.clear(); cd.comOfsH = 0; cd.comOfsL = 0;
	cd.addMassParticle(	50, MathVector<double, 3>( 1, 1, 1));
	cd.addMassParticle(510, MathVector<double, 3>(-2, 4, 3));

	cd.fuelTank.setCapacity(95); cd.fuelTank.setDensity(0.8);
	cd.fuelTank.setVolume(80); cd.fuelTank.setPosition(MathVector<double, 3>(0.0, -1.50, -0.10));

	cd.calculateMass();

	// Tests both the CarFuelTank class and addMassParticle method as well
	EXPECT_NEAR(cd.body.getMass(), 624, 0.001);
	double expectedVals[] = {803.50549, 463.209237, 303.023997, 463.209237, 659.915746, -363.258054, 303.023997, -363.258054, 998.581525};
	Matrix3<double> expected; expected.set(expectedVals);
	for (int i = 0; i < 9; i++) EXPECT_NEAR(cd.body.getInertia()[i], expected[i], 0.0001);

	double expectedVals2[] = {803.50549, 463.209237, 303.023997, 463.209237, 659.915746, -363.258054, 303.023997, -363.258054, 998.581525};
	expected.set(expectedVals2);
	for (int i = 0; i < 9; i++) EXPECT_NEAR(cd.body.getInertia()[i], expected[i], 0.0001);

	MathVector<double, 3> expectedCenterOfMass(-1.55448718, 3.19551282, 2.52179487);
	for (int i = 0; i < 3; i++) EXPECT_NEAR(cd.centerOfMass[i], expectedCenterOfMass[i], 0.0001);
}

#endif
