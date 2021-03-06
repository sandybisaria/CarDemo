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

TEST(CarDynamics, SetSteering) {
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

TEST(CarDynamics, ApplyAerodynamicsToBody) {
	CarDynamics dyn; dyn.centerOfMass = MathVector<double, 3>(-1, -1, -1);
	RigidBody& body = dyn.body;
	body.setOrientation(Quaternion<double>(1, 1, 1, 1));
	body.setVelocity(MathVector<double, 3>(2, 2, 2));
	body.setInitialForce(MathVector<double, 3>(0.f));
	body.setInitialTorque(MathVector<double, 3>(0.f));

	CarAero drag, wingFront, wingRear;
	drag.set(MathVector<double, 3>(0, 0.15, 0), 1.6, 0.20, 0, 0, 0);
	wingFront.set(MathVector<double, 3>(0, 1.45, 0.2), 0, 0, 0.8, -3.6, 0.95);
	wingFront.set(MathVector<double, 3>(0, -1.45, 0.2), 0, 0, 0.8, -4.7, 0.95);

	dyn.chassis = new btRigidBody(0, NULL, NULL); dyn.chassis->setAngularVelocity(btVector3(2, 2, 2));
	dyn.rotCoeff[0] = 200.0; dyn.rotCoeff[1] = 400.0; dyn.rotCoeff[2] = 900.0; dyn.rotCoeff[3] = 2.5;

	for (int i = 0; i < 1000; i++) {
		body.integrateStep1(0.01);
		dyn.applyAerodynamicsToBody();
		body.integrateStep2(0.01);
	}
	MathVector<double, 3> angVel = body.getAngularVelocity(), force = body.getForce(), torque = body.getTorque();
	MathVector<double, 3> a(-17718.2757, -14840.0938, -15735.7441), b(0.f), c(-1865.92672, -1972.83195, -994.02445);
	EXPECT_NEAR_HP(angVel[0], a[0]); EXPECT_NEAR_HP(angVel[1], a[1]); EXPECT_NEAR_HP(angVel[2], a[2]);
	EXPECT_NEAR_HP(force[0],  b[0]); EXPECT_NEAR_HP(force[1],  b[1]); EXPECT_NEAR_HP(force[2],  b[2]);
	EXPECT_NEAR_HP(torque[0], c[0]); EXPECT_NEAR_HP(torque[1], c[1]); EXPECT_NEAR_HP(torque[2], c[2]);

	body.setInitialForce(MathVector<double, 3>(0.f)); body.setInitialTorque(MathVector<double, 3>(0.f));
	dyn.aerodynamics.push_back(drag); dyn.aerodynamics.push_back(wingFront); dyn.aerodynamics.push_back(wingRear);
	dyn.chassis->setAngularVelocity(btVector3(2, 2, 2));
	for (int i = 0; i < 1000; i++) {
		body.integrateStep1(0.01);
		dyn.applyAerodynamicsToBody();
		body.integrateStep2(0.01);
	}
	angVel = body.getAngularVelocity(), force = body.getForce(), torque = body.getTorque();
	MathVector<double, 3> d(-34603.9238, -30323.9887, -31650.5396), e(-0.00480414042, -0.0051854746, -0.00856019825), f(-1786.2993, -977.643803, -1996.46943);
	EXPECT_NEAR_HP(angVel[0], d[0]); EXPECT_NEAR_HP(angVel[1], d[1]); EXPECT_NEAR_HP(angVel[2], d[2]);
	EXPECT_NEAR_HP(force[0],  e[0]); EXPECT_NEAR_HP(force[1],  e[1]); EXPECT_NEAR_HP(force[2],  e[2]);
	EXPECT_NEAR_HP(torque[0], f[0]); EXPECT_NEAR_HP(torque[1], f[1]); EXPECT_NEAR_HP(torque[2], f[2]);
}

TEST(CarDynamics, UpdateSuspension) {
	CarDynamics dyn; dyn.body.setOrientation(Quaternion<double>(1, 2, 3, 4));

	dyn.suspension[0].setAntiRollK(8000); dyn.suspension[0].setTravel(0.22);
	dyn.suspension[1].setAntiRollK(8000); dyn.suspension[1].setTravel(0.22);
	dyn.wheels[0].setRadius(0.32);

	TerrainSurface surface; surface.bumpWavelength = 0.17; surface.bumpAmplitude = 16.16;

	CollisionContact contact;
	contact.set(MathVector<double, 3>(0.0757, 0.0757, 0.0757), MathVector<double, 3>(0.f), 0.1, &surface, NULL, NULL);
	dyn.wheelContact[0] = contact;

	MathVector<double, 3> res = dyn.updateSuspension(0, 0.01);
	EXPECT_NEAR_HP(res[0], 565400); EXPECT_NEAR_HP(res[1], 102800); EXPECT_NEAR_HP(res[2], 514000);
}

TEST(CarDynamics, TireForceAndWheelTorque) {
	CarTire* tire = new CarTire();
	// Asphalt
	int i = 0;
	tire->lateral[i++] = 1.61;
	tire->lateral[i++] = -0;
	tire->lateral[i++] = 2775;
	tire->lateral[i++] = 2220;
	tire->lateral[i++] = 19.6;
	tire->lateral[i++] = 0.013;
	tire->lateral[i++] = -0.14;
	tire->lateral[i++] = 0.14;
	tire->lateral[i++] = 0.019;
	tire->lateral[i++] = -0.019;
	tire->lateral[i++] = -0.18;
	tire->lateral[i++] = 0;
	tire->lateral[i++] = 0;
	tire->lateral[i++] = 0;
	tire->lateral[i++] = 0;
	i = 0;
	tire->longitudinal[i++] = 1.73;
	tire->longitudinal[i++] = -0.49;
	tire->longitudinal[i++] = 3439;
	tire->longitudinal[i++] = 279;
	tire->longitudinal[i++] = 470;
	tire->longitudinal[i++] = 0;
	tire->longitudinal[i++] = 0.0008;
	tire->longitudinal[i++] = 0.005;
	tire->longitudinal[i++] = -0.024;
	tire->longitudinal[i++] = 0;
	tire->longitudinal[i++] = 0;
	i = 0;
	tire->aligning[i++] = 2.10;
	tire->aligning[i++] = -3.9;
	tire->aligning[i++] = -3.9;
	tire->aligning[i++] = -1.26;
	tire->aligning[i++] = -8.20;
	tire->aligning[i++] = 0.025;
	tire->aligning[i++] = 0;
	tire->aligning[i++] = 0.044;
	tire->aligning[i++] = -0.58;
	tire->aligning[i++] = 0.18;
	tire->aligning[i++] = 0.043;
	tire->aligning[i++] = 0.048;
	tire->aligning[i++] = -0.0035;
	tire->aligning[i++] = -0.18;
	tire->aligning[i++] = 0.14;
	tire->aligning[i++] = -1.029;
	tire->aligning[i++] = 0.27;
	tire->aligning[i++] = -1.1;
	tire->calculateSigmaHatAlphaHat();

	TerrainSurface* ts = new TerrainSurface(); // Asphalt
	ts->bumpAmplitude = 0.01; ts->bumpWavelength = 40.0; ts->friction = 1.0;
	ts->rollingResist  = 1; ts->rollingDrag = 1; ts->type = TerrainSurface::ASPHALT;
	ts->tireName = "asphalt"; ts->tire = tire;

	CollisionContact* cc = new CollisionContact();

	cc->set(MathVector<float, 3>(1, 1, 1), MathVector<float, 3>(0, 1, 0), 5, ts, NULL, NULL);
	cc->castRay(MathVector<float, 3>(1, 1, 1), MathVector<float, 3>(0, -1, 0), 5);

	CarDynamics dyn;
	dyn.wheelContact[0] = *cc;
	dyn.wheelVels[0] = MathVector<double, 3>(1, 1, 19);
	dyn.wheelPos[0] = MathVector<double, 3>(1, 1.5, 1);

	RigidBody& body = dyn.body; body.setMass(100);
	body.setOrientation(Quaternion<double>(1, 1, 1, 1));
	body.setVelocity(MathVector<double, 3>(2, 2, 2));
	body.setAngularVelocity(MathVector<double, 3>(2, 2, 2));
	body.setInitialForce(MathVector<double, 3>(0.f));
	body.setInitialTorque(MathVector<double, 3>(0.f));
	body.integrateStep1(0.01);

	CarWheel& wheel = dyn.wheels[0];
	wheel.setAngularVelocity(30); wheel.setRadius(0.5);
	wheel.setRollingResistance(1.3e-2, 6.5e-6);
	wheel.setInitialConditions();

	CarBrake& brake = dyn.brakes[0];
	brake.setArea(0.0006); brake.setBias(0.55); brake.setBrakeFactor(0.5);
	brake.setFriction(0.9); brake.setMaxPressure(3.5e7); brake.setRadius(0.335);

	const Quaternion<double> wheelRot = Quaternion<double>(.5, .5, .5, 1);

	MathVector<double, 3> wff = dyn.applyTireForce(0, 250, wheelRot),
						  expected = MathVector<double, 3>(-566.330315, 0, -338.065029);
	for (int i = 0; i < 3; i++) EXPECT_NEAR_HP(wff[i], expected[i]);
	EXPECT_NEAR_HP(wheel.getCamberDeg(), 48.5903779);

	dyn.applyWheelTorque(0.01, 100, 0, wff, wheelRot);

	body.integrateStep2(0.01);

	expected = MathVector<double, 3>(1.94311697, 2.025, 1.9614435);
	for (int i = 0; i < 3; i++) EXPECT_NEAR_HP(body.getVelocity()[i], expected[i]);

	expected = MathVector<double, 3>(-6.25970411, 0.85010876, 13.4400865);
	for (int i = 0; i < 3; i++) EXPECT_NEAR_HP(body.getAngularVelocity()[i], expected[i]);

	expected = MathVector<double, 3>(-568.830315, 250, -385.565029);
	for (int i = 0; i < 3; i++) EXPECT_NEAR_HP(body.getForce()[i], expected[i]);

	EXPECT_NEAR_HP(wheel.getAngularVelocity(), 29.2433381);
	EXPECT_NEAR_HP(wheel.getTorque(), -756.661869);

	EXPECT_FALSE(brake.willLock());
}

TEST(CarDynamics, GetWheelSteeringAndSuspensionOrientation) {
	CarDynamics dyn;

	CarWheel& wheel = dyn.wheels[0]; wheel.setSteerAngle(15);
	CarSuspension& susp = dyn.suspension[0]; susp.setCamber(-1.f); susp.setToe(0.2);

	Quaternion<double> res = dyn.getWheelSteeringAndSuspensionOrientation(WheelPosition(0));
	Quaternion<double> expected(0.0086498776, 0.00115414008, -0.132251354, 0.991177798);
	for (int i = 0; i < 4; i++) { EXPECT_NEAR_HP(res[i], expected[i]); }

	CarWheel& wheel1 = dyn.wheels[1]; wheel1.setSteerAngle(-5);
	CarSuspension& susp1 = dyn.suspension[1]; susp1.setCamber(-1.f); susp1.setToe(0.2);

	res = dyn.getWheelSteeringAndSuspensionOrientation(WheelPosition(1));
	expected = Quaternion<double>(-0.00871755214, 0.000395861726, 0.0453612608, 0.998932532);
	for (int i = 0; i < 4; i++) { EXPECT_NEAR_HP(res[i], expected[i]); }
}

TEST(CarDynamics, GetLocalWheelPosition) {
	CarDynamics dyn;

	CarWheel& wheel = dyn.wheels[0];
	wheel.setExtendedPosition(MathVector<double, 3>(-0.8345, 1.12, -0.37));

	CarSuspension& susp = dyn.suspension[0]; susp.setTravel(0.22);
	susp.setHinge(MathVector<double, 3>(-0.60, 1.12, -0.30));

	MathVector<double, 3> res = dyn.getLocalWheelPosition(WheelPosition(0), 0.5);
	MathVector<double, 3> expected = MathVector<double, 3>(-0.842289085, 1.12, -0.265558027);
	for (int i = 0; i < 3; i++) { EXPECT_NEAR_HP(res[i], expected[i]); }

	res = dyn.getLocalWheelPosition(WheelPosition(0), 0.1);
	expected = MathVector<double, 3>(-0.839653157, 1.12, -0.349564246);
	for (int i = 0; i < 3; i++) { EXPECT_NEAR_HP(res[i], expected[i]); }

	res = dyn.getLocalWheelPosition(WheelPosition(0), 1.f);
	expected = MathVector<double, 3>(-0.805248213, 1.12, -0.166720515);
	for (int i = 0; i < 3; i++) { EXPECT_NEAR_HP(res[i], expected[i]); }
}

#endif
