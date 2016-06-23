#include "CarDynamics.hpp"

void CarDynamics::shiftGear(int value) {
	if (transmission.getGear() != value && shifted) {
		if (value <= transmission.getForwardGears() && value >= -transmission.getReverseGears()) {
			remShiftTime = shiftTime;
			gearToShift = value;
			shifted = false;
		}
	}
}

void CarDynamics::updateTransmission(double dt) {
	driveshaftRPM = calculateDriveshaftRPM();

	if (autoshift) {
		//TODO Ignoring vehicle damange
		int gear = nextGear();
		double mps = getSpeedMPS();
		if (gear <= 0 && mps > 3)
			gear = 1;
		else if (gear >= 0 && spm < -3)
			gear = -1;
		else if (autorear && shifted && remShiftTime <= 0) {
			double gas = engine.getThrottle() * 0.8;
			gas -= brakes[0].getBrakeFactor();

			double g = gas;
			if (transmission.getGear() == -1) gas *= -1;

			const double spdMarg = 2.0, spd = getSpeed(); //FIXME
			if (g < -0.5 && spd < spdMarg && gear == 1) gear = -1;
			else if (g < -0.5 && spd < spdMarg && gear == -1) gear = 1;
		}

		shiftGear(gear);
	}

	remShiftTime -= dt;
	if (remShiftTime < 0) remShiftTime = 0;

	if (remShiftTime <= shiftTime * 0.5 && !shifted) {
		shifted = true;
		transmission.shift(gearToShift);
	}

	if (autoclutch) {
		if (!engine.isCombusting()) engine.startEngine();

		double throttle = engine.getThrottle();
		throttle = shiftAutoClutchThrottle(throttle, dt); //FIXME
		engine.setThrottle(throttle);

		double newClutch = autoClutch(lastAutoClutch, dt); //FIXME
		clutch.setClutch(newClutch);
		lastAutoClutch = newClutch;
	}
}

void CarDynamics::updateDriveline(double dt, double driveTorque[]) {
	engine.integrateStep1(dt);

	double driveshaftSpeed = calculateDriveshaftSpeed(); //FIXME
	double clutchSpeed = transmission.calculateClutchSpeed(driveshaftSpeed);
	double crankshaftSpeed = engine.getAngularVelocity();
	double engineDrag = clutch.getTorque(crankshaftSpeed, clutchSpeed);
	engineDrag += 0.1; // Fixes clutch stall bug when car velocity is 0 and all wheels are in the air

	engine.computeForces();
	applyClutchTorque(engineDrag, clutchSpeed); //FIXME
	engine.applyForces();
	calculateDriveTorque(driveTorque, engineDrag); //FIXME

	engine.integrateStep2(dt);
}

double CarDynamics::calculateDriveshaftRPM() const {
	for (int i = 0; i < numWheels; ++i) assert(!isnan(wheels[WheelPosition(i)].getAngularVelocity()));

	double driveshaftSpeed = 0.0;
	double whFL = wheels[FRONT_LEFT].getAngularVelocity();
	double whFR = wheels[FRONT_RIGHT].getAngularVelocity();
	if (drive == FWD) {
		driveshaftSpeed = diffFront.getDriveshaftSpeed();
		return transmission.GetClutchSpeed(driveshaftSpeed) * 30.0 / PI_d;
	}
	double whRL = wheels[REAR_LEFT].getAngularVelocity();
	double whRR = wheels[REAR_RIGHT].getAngularVelocity();
	if (drive == RWD)
		driveshaftSpeed = diffRear.getDriveshaftSpeed();
	else if (drive == AWD) {
//		double front = diffFront.getDriveshaftSpeed();
//		double rear = diffRear.getDriveshaftSpeed();
		driveshaftSpeed = diffCenter.getDriveshaftSpeed();
	}

	return transmission.getClutchSpeed(driveshaftSpeed) * 30.0 / M_PI;
}

// 0 for no change, -1 for shift down, 1 for shift up
int CarDynamics::nextGear() const {
	int gear = transmission.getGear();

	double avgSlide = 0;
	float avgWhHeight = 0;

	for (int i = 0; i < numWheels; i++) {
		double sl = fabs(wheels[i].slips.slide);
		//FIXME
	}
}

void CarDynamics::interpolateWheelContacts(double dt) {
	MathVector<float, 3> rayDir = getDownVector();
	for (int i = 0; i < numWheels; i++) {
		MathVector<float, 3> rayStart = localToWorld(wheels[i].getExtendedPosition());
		rayStart = rayStart - rayDir * wheels[i].getRadius();
		//TODO Get the wheel's collision contact
	}
}

void CarDynamics::applyEngineTorqueToBody() {
	MathVector<double, 3> engineTorque(-engine.getTorque(), 0, 0);
	getBodyOrientation().rotateVector(engineTorque);
	applyTorque(engineTorque * 0.1); // Unwanted in jumps... FIXME
}

void CarDynamics::applyAerodynamicsToBody() {
	MathVector<double, 3> windForce(0), windTorque(0), airVelocity = -getVelocity();
	(-getBodyOrientation()).rotateVector(airVelocity);

	for (std::vector<CarAero>::iterator i = aerodynamics.begin(); i != aerodynamics.end(); i++) {
		MathVector<double, 3> force = i->getForce(airVelocity);
		windForce = windForce + force;
		windTorque = windTorque + (i->getPosition() - centerOfMass).cross(force);
	}

	getBodyOrientation().rotateVector(windForce);
	getBodyOrientation().rotateVector(windTorque);

	applyForce(windForce); //FIXME
	applyTorque(windTorque); //FIXME

	// Rotational damping/drag
	if (rotCoeff[0] > 0.0 && chassis) {
		MathVector<double, 3> rotDrag = -toMathVector<double>(chassis->getAngularVelocity());
		(-getBodyOrientation()).rotateVector(rotDrag);

		rotDrag[0] *= rotCoeff[0]; // Roll
		rotDrag[1] *= rotCoeff[1]; // Pitch
		rotDrag[2] *= rotCoeff[2] + rotCoeff[3] * rotDrag[2]; // Yaw

		getBodyOrientation().rotateVector(rotDrag);
		applyTorque(rotDrag); //FIXME
	}
}
