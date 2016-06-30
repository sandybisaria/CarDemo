#include "CarDynamics.hpp"

#ifdef COMPILE_UNIT_TESTS
#include <gtest/gtest.h>

TEST(CarDynamics, CalculateMass) {
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

#define EXPECT_NEAR_HP(a, b) EXPECT_NEAR(a, b, 0.0001)

TEST(CarDynamics, GetSpeedMPS) {
	CarDynamics dyn;

	dyn.setDrive("AWD");
	dyn.setNumWheels(4);
	dyn.wheels[0].setAngularVelocity(123); dyn.wheels[1].setAngularVelocity(184);
	dyn.wheels[2].setAngularVelocity(243); dyn.wheels[3].setAngularVelocity(221);
	dyn.wheels[0].setRadius(2); dyn.wheels[1].setRadius(2);
	dyn.wheels[2].setRadius(2); dyn.wheels[3].setRadius(2);
	EXPECT_NEAR_HP(dyn.getSpeedMPS(), 385.5);

	dyn.setDrive("FWD");
	EXPECT_NEAR_HP(dyn.getSpeedMPS(), 307);

	dyn.setDrive("RWD");
	EXPECT_NEAR_HP(dyn.getSpeedMPS(), 464);
}

TEST(CarDynamics, GetSteerAngle) {
	CarDynamics dyn;

	dyn.maxAngle = 40;
	dyn.setNumWheels(4);
	dyn.wheels[0].setExtendedPosition(MathVector<double, 3>( 1,  1, 0));
	dyn.wheels[1].setExtendedPosition(MathVector<double, 3>( 1, -1, 0));
	dyn.wheels[2].setExtendedPosition(MathVector<double, 3>(-1,  1, 0));
	dyn.wheels[3].setExtendedPosition(MathVector<double, 3>(-1, -1, 0));

	dyn.setSteering(-0.5, .81 * .7); EXPECT_NEAR_HP(dyn.steerValue, -0.5);
	EXPECT_NEAR_HP(dyn.wheels[0].getSteerAngle(), -14.0822552);
	EXPECT_NEAR_HP(dyn.wheels[1].getSteerAngle(), -11.3399994);
	EXPECT_NEAR_HP(dyn.wheels[2].getSteerAngle(),			0);
	EXPECT_NEAR_HP(dyn.wheels[3].getSteerAngle(),			0);

	dyn.setSteering(0.95, .81 * .7); EXPECT_NEAR_HP(dyn.steerValue, 0.95);
	EXPECT_NEAR_HP(dyn.wheels[0].getSteerAngle(), 21.5459989);
	EXPECT_NEAR_HP(dyn.wheels[1].getSteerAngle(), 33.1224614);
	EXPECT_NEAR_HP(dyn.wheels[2].getSteerAngle(),		   0);
	EXPECT_NEAR_HP(dyn.wheels[3].getSteerAngle(),		   0);
}

#endif
