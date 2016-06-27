#include "CarEngine.hpp"

CarEngine::CarEngine()
	: maxRPM(7800), idle(0.02), fricCoeffB(230), realPowTorqueMul(1.0),
	  startRPM(1000), stallRPM(350), fuelConsumption(1e-9), friction(0.000328),
	  throttlePosition(0.0), clutchTorque(0.0), outOfFuel(false), mass(200),
	  revLimitExceeded(false), frictionTorque(0), combustionTorque(0), stalled(false) {
	Matrix3<double> inertia;
	inertia.scale(0.25);
	crankshaft.setInertia(inertia);
}

double CarEngine::getTorqueCurve(double curThrottle, double curRPM) const {
	if (curRPM < 1)
		return 0.0;

	double torque = torqueCurve.interpolate(curRPM);

	return torque * curThrottle;
}

double CarEngine::getFrictionTorque(double curAngVel, double fricFactor, double throttlePos) {
	double velSign = curAngVel < 0 ? -1.0 : 1.0;
	double b = fricCoeffB * friction;

	return (-curAngVel * b) * (1.0 - fricFactor * throttlePos);
}

void CarEngine::computeForces() {
	if (getRPM() < stallRPM)
		stalled = true;
	else
		stalled = false;

	if (throttlePosition < idle)
		throttlePosition = idle; // Must be at least idle

	double curAngVel = crankshaft.getAngularVelocity()[0];

	double fricFactor = 1.0; // Used to make sure friction works even when out of fuel
	double revLimit = maxRPM + 500;
	if (revLimitExceeded)
		revLimit -= 400;

	if (getRPM() < revLimit)
		revLimitExceeded = false;
	else
		revLimitExceeded = true;

	combustionTorque = getTorqueCurve(throttlePosition, getRPM());

	if (outOfFuel || revLimitExceeded || stalled) {
		fricFactor = 0.0;
		combustionTorque = 0.0;
	}

	frictionTorque = getFrictionTorque(curAngVel, fricFactor, throttlePosition);
	if (stalled)
		frictionTorque *= 100.0; // Try to mimic engine static friction
}

void CarEngine::applyForces() {
	MathVector<double, 3> totalTorque(0);

	totalTorque[0] += combustionTorque + frictionTorque - clutchTorque;

	crankshaft.setTorque(totalTorque);
}

void CarEngine::setTorqueCurve(double maxPowerRPM, std::vector<std::pair<double, double> >& curve) {
	torqueCurve.clear();

	// Accounts for fact that torque curves are usually measured on a dyno,
	// but we're interested in the actual crankshaft power.
	const double dynoCorrectionFactor = 1.0;

	assert(curve.size() > 1);

	if (curve[0].first != 0)
		torqueCurve.addPoint(0, 0); // Want to always start from 0 RPM

	for (std::vector<std::pair<double, double> >::iterator i = curve.begin(); i != curve.end(); ++i) {
		torqueCurve.addPoint(i->first, i->second * dynoCorrectionFactor);
	}

	// Ensure we have a smooth curve for over-revs
	torqueCurve.addPoint(curve[curve.size()-1].first + 10000, 0);

	double maxPowerAngVel = maxPowerRPM * M_PI / 30.0;
	double maxPower = torqueCurve.interpolate(maxPowerRPM) * maxPowerAngVel;
	friction = maxPower / (maxPowerAngVel * maxPowerAngVel * maxPowerAngVel);

	// Calculate idle throttle position
	for (idle = 0; idle < 1.0; idle += 0.01) {
		if (getTorqueCurve(idle, startRPM) > -getFrictionTorque(startRPM * M_PI / 30.0, 1.0, idle))
			break;
	}
}
