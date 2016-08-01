#include "CarEngine.hpp"

CarEngine::CarEngine()
	: maxRPM(7800), idle(0.02), fricCoeffB(230),
	  startRPM(1000), stallRPM(350), fuelConsumption(1e-9), friction(0.000328),
	  throttlePosition(0.0), clutchTorque(0.0), outOfFuel(false), mass(200),
	  revLimitExceeded(false), frictionTorque(0), combustionTorque(0), stalled(false) {
	Matrix3<double> inertia;
	inertia.scale(0.25);
	crankshaft.setInertia(inertia);
}

// Called between integration steps 1 and 2
void CarEngine::computeForces() {
	stalled = getRPM() < stallRPM;

	if (throttlePosition < idle) { throttlePosition = idle; } // Must be at least idle

	double curAngVel = crankshaft.getAngularVelocity()[0];

	double fricFactor = 1.0; // Used to make sure friction works even when out of fuel
	double revLimit = maxRPM + 500;
	if (revLimitExceeded) { revLimit -= 400; }

	revLimitExceeded = getRPM() >= revLimit;

	combustionTorque = getTorqueCurve(throttlePosition, getRPM());

	if (outOfFuel || revLimitExceeded || stalled) {
		fricFactor = 0.0;
		combustionTorque = 0.0;
	}

	frictionTorque = getFrictionTorque(curAngVel, fricFactor, throttlePosition);
	if (stalled) { frictionTorque *= 100.0; } // Try to mimic engine static friction
}

// Called between integration steps 1 and 2 (after computing the forces)
void CarEngine::applyForces() {
	MathVector<double, 3> totalTorque(0);
	totalTorque[0] += combustionTorque + frictionTorque - clutchTorque;

	crankshaft.setTorque(totalTorque);
}

void CarEngine::setTorqueCurve(double maxPowerRPM, std::vector<std::pair<double, double> >& curve) {
	torqueCurve.clear();

	// Dyno correction factor removed because it equaled 1

	assert(curve.size() > 1);

	if (curve[0].first != 0)
		torqueCurve.addPoint(0, 0); // Want to always start from 0 RPM

	for (std::vector<std::pair<double, double> >::iterator i = curve.begin(); i != curve.end(); ++i) {
		torqueCurve.addPoint(i->first, i->second);
	}

	// Ensure we have a smooth curve for over-revs
	torqueCurve.addPoint(curve[curve.size()-1].first + 10000, 0);

	double maxPowerAngVel = maxPowerRPM * M_PI / 30.0;
	double maxPower = torqueCurve.interpolate(maxPowerRPM) * maxPowerAngVel;
	friction = maxPower / (maxPowerAngVel * maxPowerAngVel * maxPowerAngVel);

	// Calculate idle throttle position
	for (idle = 0; idle < 1.0; idle += 0.01) {
		if (getTorqueCurve(idle, startRPM) >
			-getFrictionTorque(startRPM * M_PI / 30.0, 1.0, idle)) {
			break;
		}
	}
}

double CarEngine::getTorqueCurve(double curThrottle, double curRPM) const {
	if (curRPM < 1) { return 0.0; }

	double torque = torqueCurve.interpolate(curRPM);
	return torque * curThrottle;
}

double CarEngine::getFrictionTorque(double curAngVel, double fricFactor, double throttlePos) {
	double b = fricCoeffB * friction;

	return (-curAngVel * b) * (1.0 - fricFactor * throttlePos);
}

#ifdef COMPILE_UNIT_TESTS
#include <gtest/gtest.h>

TEST(CarEngine, CarEngineFunctions) {
	CarEngine engine;

	engine.setPosition(MathVector<double, 3>(0, -0.8, 0.15));
	engine.setMass(250);
	engine.setMaxRPM(8000);
	engine.setInertia(0.33);
	engine.setStartRPM(1000);
	engine.setStallRPM(500);
	engine.setFuelConsumption(0.0001);
	engine.setFrictionB(230);

	float torqueValMul = 0.88;
	std::vector<std::pair<double, double> > torqueCurve;
	torqueCurve.push_back(std::make_pair(1000, torqueValMul * 325));
	torqueCurve.push_back(std::make_pair(1500, torqueValMul * 360));
	torqueCurve.push_back(std::make_pair(2200, torqueValMul * 388));
	torqueCurve.push_back(std::make_pair(2700, torqueValMul * 426));
	torqueCurve.push_back(std::make_pair(3200, torqueValMul * 452));
	torqueCurve.push_back(std::make_pair(3800, torqueValMul * 482));
	torqueCurve.push_back(std::make_pair(4200, torqueValMul * 511));
	torqueCurve.push_back(std::make_pair(4800, torqueValMul * 555));
	torqueCurve.push_back(std::make_pair(5200, torqueValMul * 576));
	torqueCurve.push_back(std::make_pair(5600, torqueValMul * 580));
	torqueCurve.push_back(std::make_pair(6000, torqueValMul * 576));
	torqueCurve.push_back(std::make_pair(6500, torqueValMul * 554));
	torqueCurve.push_back(std::make_pair(7000, torqueValMul * 526));
	torqueCurve.push_back(std::make_pair(7500, torqueValMul * 498));
	torqueCurve.push_back(std::make_pair(8000, torqueValMul * 475));
	torqueCurve.push_back(std::make_pair(8500, torqueValMul * 454));
	torqueCurve.push_back(std::make_pair(9000, torqueValMul * 409));
	engine.setTorqueCurve(8000, torqueCurve);

	EXPECT_NEAR(engine.getTorqueCurve(0, 500), 0, 0.0001);
	EXPECT_NEAR(engine.getTorqueCurve(0.5, 4000), 218.163537, 0.0001);
	EXPECT_NEAR(engine.getTorqueCurve(1, 4000), 436.327073, 0.0001);
	EXPECT_NEAR(engine.getTorqueCurve(1, 8000), 418, 0.0001);

	EXPECT_TRUE(engine.isCombusting());

	engine.setInitialConditions();
	EXPECT_TRUE(engine.isCombusting());
	EXPECT_NEAR(engine.fuelRate(), 0, 0.0001);
	EXPECT_NEAR(engine.getRPM(), 1000, 0.0001);
	EXPECT_NEAR(engine.getAngularVelocity(), 104.719755, 0.0001);

	// Integrate for 10 seconds
	for (int i = 0; i < 1000; i++) {
		engine.integrateStep1(0.01);
		engine.computeForces();
		engine.applyForces();
		engine.integrateStep2(0.01);
	}
	EXPECT_NEAR(engine.getTorque(), 0.115216271, 0.0001);
	EXPECT_TRUE(engine.isCombusting());
	EXPECT_NEAR(engine.fuelRate(), 0.000576164852, 0.0001);
	EXPECT_NEAR(engine.getRPM(), 1100.39381, 0.0001);
	EXPECT_NEAR(engine.getAngularVelocity(), 115.23297, 0.0001);

	engine.setThrottle(1.f);
	// Integrate for 10 seconds
	for (int i = 0; i < 1000; i++) {
		engine.integrateStep1(0.01);
		engine.computeForces();
		engine.setClutchTorque(-500);
		engine.applyForces();
		engine.integrateStep2(0.01);
	}
	EXPECT_NEAR(engine.getTorque(), -493.356965, 0.0001);
	EXPECT_TRUE(engine.isCombusting());
	EXPECT_NEAR(engine.fuelRate(), 0.3601792, 0.0001);
	EXPECT_NEAR(engine.getRPM(), 34394.5801, 0.0001);
	EXPECT_NEAR(engine.getAngularVelocity(), 3601.792, 0.0001);
}

#endif
