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

double CarDynamics::getSpeedMPS() const {
	for (int i = 0; i < numWheels; ++i) assert(!isnan(wheels[WheelPosition(i)].getAngularVelocity()));

	double whFL = wheels[FRONT_LEFT].getAngularVelocity();
	double whFR = wheels[FRONT_RIGHT].getAngularVelocity();
	if (drive == FWD) return (whFL + whFR) * 0.5 * wheels[FRONT_LEFT].getRadius();

	double whRL = wheels[REAR_LEFT].getAngularVelocity();
	double whRR = wheels[REAR_RIGHT].getAngularVelocity();

	if (drive == RWD) return (whRL + whRR) * 0.5 * wheels[REAR_LEFT].getRadius();
	else {
		assert(drive == AWD);
		return ((whFL + whFR) * 0.5 * wheels[FRONT_LEFT].getRadius() +
				(whRL + whRR) * 0.5 * wheels[REAR_LEFT].getRadius()) * 0.5;
	}
}

void CarDynamics::setSteering(const double val, const float rangeMul) {
	//TODO Dmg

	steerValue = val;
	double steerAngle = val * maxAngle * rangeMul; // Steering angle in degs
	if (numWheels == 2) {
		wheels[FRONT_LEFT].setSteerAngle(steerAngle);
		return;
	}

	// Ackerman steering geometry
	bool ax2 = numWheels >= 6; // Two front steering axles?
	int iMax = ax2 ? 2 : 1;
	for (int i = 0; i < iMax; i++) {
		WheelPosition wl, wr, rear;
		if (i == 0) { wl = FRONT_LEFT; wr = FRONT_RIGHT; rear = REAR_LEFT; }
		else { wl = REAR_LEFT; wr = REAR_RIGHT; rear = REAR2_LEFT; }

		double alpha = std::abs(steerAngle * M_PI / 180.0); // Outside wheel steering angle in rads

		double dW = wheels[wl].getExtendedPosition()[1] - wheels[wr].getExtendedPosition()[1]; // Width between front wheels
		double dL = wheels[wl].getExtendedPosition()[0] - wheels[rear].getExtendedPosition()[0]; // Length between front and rear
		if (i == 1) dL *= 2.f;

		double beta = atan2(1.0, 1.0 / tan(alpha) - dW / fabs(dL)); // Inside wheel steering angle in rads

		double left = 0, right = 0; // Wheel angle
		if (val >= 0) { left = alpha; right = beta; }
		else { left = -alpha; right = -beta; }
		left *= 180.0 / M_PI; right *= 180.0 / M_PI;

		wheels[wl].setSteerAngle(left); wheels[wr].setSteerAngle(right);
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
		else if (gear >= 0 && mps < -3)
			gear = -1;
		else if (autorear && shifted && remShiftTime <= 0) {
			double gas = engine.getThrottle() * 0.8;
			gas -= brakes[0].getBrakeFactor();

			double g = gas;
			if (transmission.getGear() == -1) gas *= -1;

			const double spdMarg = 2.0;
			const double spd = getSpeed();
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
		throttle = shiftAutoClutchThrottle(throttle, dt);
		engine.setThrottle(throttle);

		double newClutch = autoClutch(lastAutoClutch, dt);
		clutch.setClutch(newClutch);
		lastAutoClutch = newClutch;
	}
}

void CarDynamics::updateDriveline(double dt, double driveTorque[]) {
	engine.integrateStep1(dt);

	double driveshaftSpeed = calculateDriveshaftSpeed();
	double clutchSpeed = transmission.calculateClutchSpeed(driveshaftSpeed);
	double crankshaftSpeed = engine.getAngularVelocity();
	double engineDrag = clutch.getTorque(crankshaftSpeed, clutchSpeed);
	engineDrag += 0.1; // Fixes clutch stall bug when car velocity is 0 and all wheels are in the air

	engine.computeForces();
	applyClutchTorque(engineDrag, clutchSpeed);
	engine.applyForces();
	calculateDriveTorque(driveTorque, engineDrag);

	engine.integrateStep2(dt);
}

double CarDynamics::calculateDriveshaftRPM() const {
	for (int i = 0; i < numWheels; ++i) assert(!isnan(wheels[WheelPosition(i)].getAngularVelocity()));

	double driveshaftSpeed = 0.0;
	double whFL = wheels[FRONT_LEFT].getAngularVelocity();
	double whFR = wheels[FRONT_RIGHT].getAngularVelocity();
	if (drive == FWD) {
		driveshaftSpeed = diffFront.getDriveshaftSpeed();
		return transmission.getClutchSpeed(driveshaftSpeed) * 30.0 / M_PI;
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

double CarDynamics::calculateDriveshaftSpeed() {
	for (int i = 0; i < numWheels; ++i) assert(!isnan(wheels[WheelPosition(i)].getAngularVelocity()));

	double driveshaftSpeed = 0.0;
	double whFL = wheels[FRONT_LEFT].getAngularVelocity();
	double whFR = wheels[FRONT_RIGHT].getAngularVelocity();
	if (drive == FWD) {
		driveshaftSpeed = diffFront.calculateDriveshaftSpeed(whFL, whFR);
		return driveshaftSpeed;
	}

	double whRL = wheels[REAR_LEFT].getAngularVelocity();
	double whRR = wheels[REAR_RIGHT].getAngularVelocity();
	if (drive == RWD)
		driveshaftSpeed = diffRear.calculateDriveshaftSpeed(whRL, whRR);
	else if (drive == AWD)
		driveshaftSpeed = diffCenter.calculateDriveshaftSpeed(diffFront.calculateDriveshaftSpeed(whFL, whFR),
															  diffRear.calculateDriveshaftSpeed(whRL, whRR));

	return driveshaftSpeed;
}

void CarDynamics::applyClutchTorque(double engineDrag, double clutchSpeed) {
	//TODO Don't need clutch speed
	engine.setClutchTorque(transmission.getGear() == 0 ? 0.0 : engineDrag);
}

// Calculate the drive torque that the engine applies to each wheel
void CarDynamics::calculateDriveTorque(double driveTorque[], double clutchTorque) {
	double driveshaftTorque = transmission.getTorque(clutchTorque);
	assert(!isnan(driveshaftTorque));

	for (int i = 0; i < numWheels; i++) driveTorque[i] = 0;

	if (drive == RWD) {
		diffRear.computeWheelTorques(driveshaftTorque);
		driveTorque[REAR_LEFT] = diffRear.getSide1Torque();
		driveTorque[REAR_RIGHT] = diffRear.getSide2Torque();
	} else if (drive == FWD) {
		diffFront.computeWheelTorques(driveshaftTorque);
		driveTorque[FRONT_LEFT] = diffFront.getSide1Torque();
		driveTorque[FRONT_RIGHT] = diffFront.getSide2Torque();
	} else if (drive == AWD) {
		diffCenter.computeWheelTorques(driveshaftTorque);
		diffFront.computeWheelTorques(diffCenter.getSide1Torque());
		diffRear.computeWheelTorques(diffCenter.getSide2Torque());
		driveTorque[FRONT_LEFT] = diffFront.getSide1Torque();
		driveTorque[FRONT_RIGHT] = diffFront.getSide2Torque();
		driveTorque[REAR_LEFT] = diffRear.getSide1Torque();
		driveTorque[REAR_RIGHT] = diffRear.getSide2Torque();
	}

	for (int i = 0; i < numWheels; i++) assert(!isnan(driveTorque[WheelPosition(i)]));
}

// 0 for no change, -1 for shift down, 1 for shift up
int CarDynamics::nextGear() const {
	int gear = transmission.getGear();

	double avgSlide = 0;
	float avgWhHeight = 0; //TODO Will leave as 0 as don't have fluids

	for (int i = 0; i < numWheels; i++) {
		double sl = fabs(wheels[i].slips.slide);
		avgSlide += sl;
//		avgWhHeight += wheelHeight[i]; //TODO Uncomment when doing buoyancy
	}

	bool allow = true;
	float nw = 1.f / numWheels; avgSlide *= nw; avgWhHeight *= nw;
	if (avgSlide > 1.0) allow = false;

	if (allow) {
		if (shifted && clutch.getClutch() == 1.0) {
			// Shift up when driveshaft speed > engine red line
			if (driveshaftRPM > engine.getMaxRPM() && gear > 0) return gear + 1;
			// Shift down when driveshaft speed < shift-down point
			if (driveshaftRPM < downshiftRPM(gear, avgWhHeight) && gear > 1) return gear - 1;
		}
	}
	return gear;
}

double CarDynamics::downshiftRPM(int gear, float avgWheelHeight) const {
	double shiftDownPoint = 0.0;
	if (gear > 1) {
		double currentGearRatio = transmission.getGearRatio(gear);
		double lowerGearRatio = transmission.getGearRatio(gear - 1);
		double peakEngineSpeed = engine.getMaxRPM();
		double throttle = engine.getThrottle();

		shiftDownPoint = peakEngineSpeed / lowerGearRatio * currentGearRatio;
		// In mud, shift down only at very low RPM; shift down more easily with less throttle
		shiftDownPoint *= (throttle > 0.5? (avgWheelHeight > 0.5f ? 0.4 : 0.7) : 0.9);
	}

	return shiftDownPoint;
}

double CarDynamics::autoClutch(double lastClutch, double dt) const {
	const double threshold = 1000.0;
	const double margin = 100.0;
	const double gearEffect = 1.0; // 0-1; defines special consideration of first/reverse gear

	// Take into account locked brakes
	bool willLock = true;
	for (int i = 0; i < numWheels; i++) {
		if (wheelDriven(WheelPosition(i))) willLock = willLock && brakes[i].willLock();
	}
	if (willLock) return 0;

	const double rpm = engine.getRPM();
	const double maxRPM = engine.getMaxRPM();
	const double stallRPM = engine.getStallRPM() + margin * (maxRPM / 2000.0);
	const int gear = transmission.getGear();

	double gearFactor = gear <= 1 ? 2.0 : 1.0;
	double thresh = threshold * (maxRPM / 7000.0) * ((1.0 - gearEffect) + gearFactor * gearEffect) + stallRPM;
	if (clutch.isLocked()) thresh *= 0.5;
	double cl = (rpm - stallRPM) / (thresh - stallRPM);
	cl = std::min(std::max(cl, 0.), 1.);

	double newAuto = cl * shiftAutoClutch();

	// Rate limit the auto-clutch
	const double minEngageTime = 0.05; // Fastest time in sec for auto-clutch engagement
	const double engageRateLimit = 1.0 / minEngageTime;
	const double rate = (lastClutch - newAuto) / dt; // Engagement rate in clutch units per sec
	if (rate > engageRateLimit) newAuto = lastClutch - engageRateLimit * dt;

	return newAuto;
}

double CarDynamics::shiftAutoClutch() const {
	double shiftClutch = 1.0;
	if (remShiftTime > shiftTime * 0.5) shiftClutch = 0;
	else if (remShiftTime > 0.0) shiftClutch = 1.0 - remShiftTime / (shiftTime * 0.5);
	return shiftClutch;
}

double CarDynamics::shiftAutoClutchThrottle(double throttle, double dt) {
	if (remShiftTime > 0.0) {
		if (engine.getRPM() <driveshaftRPM && engine.getRPM() < engine.getMaxRPM()) {
			remShiftTime += dt;
			return 1.0;
		} else {
			return throttle * 0.5;
		}
	}
	return throttle;
}

MathVector<double, 3> CarDynamics::updateSuspension(int i, double dt) {
	// Displacement
	//TODO Need wheelContact (CollisionContact) and TerrainSurface!

	double displacement = 2.0 * wheels[i].getRadius(); //TODO Fix with wheel contact and bump offset!

	// Compute suspension force
	double springDampForce = suspension[i].update(dt, displacement);

	// Anti-roll
	int otherI = i;
	if (i % 2 == 0) otherI++;
	else otherI--;

	double antiRollForce = suspension[i].getAntiRollK() *
						   (suspension[i].getDisplacement() - suspension[otherI].getDisplacement());

	if (isnan(antiRollForce)) antiRollForce = 0.f;
	assert(!isnan(antiRollForce));

	MathVector<double, 3> suspForce(0, 0, antiRollForce + springDampForce);
	getBodyOrientation().rotateVector(suspForce);
	return suspForce;
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
	applyTorque(engineTorque * 0.1); // Unwanted in jumps...
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

	applyForce(windForce);
	applyTorque(windTorque);

	// Rotational damping/drag
	if (rotCoeff[0] > 0.0 && chassis) {
		MathVector<double, 3> rotDrag = -toMathVector<double>(chassis->getAngularVelocity());
		(-getBodyOrientation()).rotateVector(rotDrag);

		rotDrag[0] *= rotCoeff[0]; // Roll
		rotDrag[1] *= rotCoeff[1]; // Pitch
		rotDrag[2] *= rotCoeff[2] + rotCoeff[3] * rotDrag[2]; // Yaw

		getBodyOrientation().rotateVector(rotDrag);
		applyTorque(rotDrag);
	}
}
