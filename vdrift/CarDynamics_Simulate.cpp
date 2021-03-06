#include "CarDynamics.hpp"

void CarDynamics::shiftGear(int value) {
	// "shifted" is set true when we initiate a shift but it hasn't occurred (yet)
	if (transmission.getGear() != value && shifted) {
		if (value <= transmission.getForwardGears() &&
			value >= -transmission.getReverseGears()) {
			remShiftTime = shiftTime;
			gearToShift = value;
			shifted = false;
		}
	}
}

// Speed is determined by wheels' angular velocities
double CarDynamics::getSpeedMPS() const {
	for (int i = 0; i < numWheels; ++i) {
		assert(!isnan(wheels[WheelPosition(i)].getAngularVelocity()));
	}

	double whFL = wheels[FRONT_LEFT].getAngularVelocity();
	double whFR = wheels[FRONT_RIGHT].getAngularVelocity();
	if (drive == FWD) { return (whFL + whFR) * 0.5 * wheels[FRONT_LEFT].getRadius(); }

	double whRL = wheels[REAR_LEFT].getAngularVelocity();
	double whRR = wheels[REAR_RIGHT].getAngularVelocity();

	if (drive == RWD) { return (whRL + whRR) * 0.5 * wheels[REAR_LEFT].getRadius(); }
	else {
		// drive == AWD
		return ((whFL + whFR) * 0.25 * wheels[FRONT_LEFT].getRadius()) +
			   ((whRL + whRR) * 0.25 * wheels[ REAR_LEFT].getRadius());
	}
}

void CarDynamics::setSteering(const double val, const double rangeMul) {
	// Effects of damage were ignored
	double steerAngle = val * maxAngle * rangeMul; // Steering angle in degrees
	if (numWheels == 2) {
		wheels[FRONT_LEFT].setSteerAngle(steerAngle);
		return;
	}

	// Ackermann steering geometry
	bool ax2 = numWheels >= 6; // Two front steering axles for 6 wheels or more
	int iMax = ax2 ? 2 : 1; // One iteration per axle
	for (int i = 0; i < iMax; i++) {
		WheelPosition wl, wr, rear;
		if (i == 0) { wl = FRONT_LEFT; wr = FRONT_RIGHT; rear =  REAR_LEFT; } // Under six wheels
		else 		{ wl =  REAR_LEFT; wr =  REAR_RIGHT; rear = REAR2_LEFT; } // Six or more wheels

		double alpha = std::abs(steerAngle * M_PI / 180.0); // Outside wheel steering angle in rads

		// Width between wheels (track)
		double dW = wheels[wl].getExtendedPosition()[1] - wheels[wr].getExtendedPosition()[1];

		// Length between wheels (wheelbase)
		double dL = wheels[wl].getExtendedPosition()[0] - wheels[rear].getExtendedPosition()[0];

		if (i == 1) dL *= 2.f; // Double the length for second axle

		double beta = atan2(1.0, 1.0 / tan(alpha) - dW / fabs(dL)); // Inside wheel steering angle in rads

		double left = 0, right = 0;
		if (val >= 0) {   left = alpha; right = beta; }
		else 		  { right = -alpha; left = -beta; }

		left *= 180.0 / M_PI; right *= 180.0 / M_PI; // Angles must be set as degrees
		wheels[wl].setSteerAngle(left); wheels[wr].setSteerAngle(right);
	}
}

void CarDynamics::updateTransmission(double dt) {
	driveshaftRPM = calculateDriveshaftRPM();

	if (autoshift) {
		//Ignoring vehicle damange
		int gear = nextGear();
		double mps = getSpeedMPS();
		if (gear <= 0 && mps > 3)
			gear = 1;
		else if (gear >= 0 && mps < -3)
			gear = -1;
		else if (autorear && shifted && remShiftTime <= 0) {
			// Auto-rear gear
			double gas = engine.getThrottle() * 0.8;
			gas -= brakes[0].getBrakeFactor();

			double g = gas;
			if (transmission.getGear() == -1) gas *= -1;

			const double spdMarg = 2.0;
			const double spd = getSpeed();

				 if (g < -0.5 && spd < spdMarg && gear ==  1) gear = -1;
			else if (g < -0.5 && spd < spdMarg && gear == -1) gear =  1;
		}

		shiftGear(gear);
	}

	remShiftTime -= dt;
	remShiftTime = std::max(remShiftTime, 0.0);

	if (remShiftTime <= shiftTime * 0.5 && !shifted) {
		shifted = true;
		transmission.shift(gearToShift);
	}

	if (autoclutch) {
		if (!engine.isCombusting()) { engine.startEngine(); }

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
	engineDrag += 0.1;
	// Fixes clutch stall bug when car velocity is 0 and all wheels are in the air,
	// according to the Stuntrally dev

	engine.computeForces();
	applyClutchTorque(engineDrag);
	engine.applyForces();
	calculateDriveTorque(driveTorque, engineDrag);

	engine.integrateStep2(dt);
}

double CarDynamics::calculateDriveshaftRPM() const {
	for (int i = 0; i < numWheels; ++i) {
		assert(!isnan(wheels[WheelPosition(i)].getAngularVelocity()));
	}

	double driveshaftSpeed;
	if (drive == FWD)
		driveshaftSpeed = diffFront.getDriveshaftSpeed();
	else if (drive == RWD)
		driveshaftSpeed = diffRear.getDriveshaftSpeed();
	else // drive == AWD
		driveshaftSpeed = diffCenter.getDriveshaftSpeed();

	return transmission.getClutchSpeed(driveshaftSpeed) * 30.0 / M_PI;
}

double CarDynamics::calculateDriveshaftSpeed() {
	for (int i = 0; i < numWheels; ++i) {
		assert(!isnan(wheels[WheelPosition(i)].getAngularVelocity()));
	}

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

void CarDynamics::applyClutchTorque(double engineDrag) {
	engine.setClutchTorque(transmission.getGear() == 0 ? 0.0 : engineDrag);
}

// Calculate the drive torque that the engine applies to each wheel
void CarDynamics::calculateDriveTorque(double driveTorque[], double clutchTorque) {
	double driveshaftTorque = transmission.getTorque(clutchTorque);
	assert(!isnan(driveshaftTorque));

	for (int i = 0; i < numWheels; i++) { driveTorque[i] = 0; }

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

	for (int i = 0; i < numWheels; i++) { assert(!isnan(driveTorque[i])); }
}

// 0 for no change, -1 for shift down, +1 for shift up
int CarDynamics::nextGear() const {
	int gear = transmission.getGear();

	double avgSlide = 0;
	float avgWhHeight = 0;
	for (int i = 0; i < numWheels; i++) {
		double sl = fabs(wheels[i].slips.slide);
		avgSlide += sl;
		// Wheel height unused (due to no buoyancy)
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
	const double gearEffect = 1.0; // 0 -> 1; defines special consideration of first/reverse gear

	// Check for locked brakes
	bool willLock = true;
	for (int i = 0; i < numWheels; i++) {
		if (wheelDriven(i)) { willLock = willLock && brakes[i].willLock(); }
	}
	if (willLock) { return 0; }

	const double rpm = engine.getRPM();
	const double maxRPM = engine.getMaxRPM();
	const double stallRPM = engine.getStallRPM() + margin * (maxRPM / 2000.0);
	const int gear = transmission.getGear();

	double gearFactor = gear <= 1 ? 2.0 : 1.0;
	double thresh = threshold * (maxRPM / 7000.0) * ((1.0 - gearEffect) + gearFactor * gearEffect) + stallRPM;
	if (clutch.isLocked()) { thresh *= 0.5; }
	double cl = (rpm - stallRPM) / (thresh - stallRPM);
	cl = clamp(cl, 0.0, 1.0);

	double newAuto = cl * shiftAutoClutch();

	// Rate limit the auto-clutch
	const double minEngageTime = 0.05; // Fastest time for auto-clutch engagement
	const double engageRateLimit = 1.0 / minEngageTime;
	const double rate = (lastClutch - newAuto) / dt; // Engagement rate in clutch units per sec
	if (rate > engageRateLimit) { newAuto = lastClutch - engageRateLimit * dt; }

	return newAuto;
}

double CarDynamics::shiftAutoClutch() const {
	double shiftClutch = 1.0;
	if (remShiftTime > shiftTime * 0.5) { shiftClutch = 0; }
	else if (remShiftTime > 0.0) { shiftClutch = 1.0 - remShiftTime / (shiftTime * 0.5); }

	return shiftClutch;
}

double CarDynamics::shiftAutoClutchThrottle(double throttle, double dt) {
	if (remShiftTime > 0.0) {
		if (engine.getRPM() < driveshaftRPM && engine.getRPM() < engine.getMaxRPM()) {
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
	const double posX = wheelContact[i].getPosition()[0];
	const double posY = wheelContact[i].getPosition()[2];
	const TerrainSurface* surface = wheelContact[i].getSurface();

	double phase = 0;
	if (surface->bumpWavelength > 0.0001) { phase = 2 * M_PI * (posX + posY) / surface->bumpWavelength; }
	double shift = 2.0 * sin(phase * sqrt(2));
	double amplitude = 0.25 * surface->bumpAmplitude;
	double bumpOffset = amplitude * (sin(phase + shift) + sin(phase * sqrt(2)) - 2.0);

	double displacement = 2.0 * wheels[i].getRadius() - wheelContact[i].getDepth() + bumpOffset;

	// Compute suspension force
	double springDampForce = suspension[i].update(dt, displacement);

	// Anti-roll
	int otherI = i;
	if (i % 2 == 0) { otherI++; }
	else { otherI--; }
	double antiRollForce = suspension[i].getAntiRollK() *
						   (suspension[i].getDisplacement() - suspension[otherI].getDisplacement());

	if (isnan(antiRollForce)) { antiRollForce = 0.f; }

	MathVector<double, 3> suspForce(0, 0, antiRollForce + springDampForce);
	getBodyOrientation().rotateVector(suspForce);
	return suspForce;
}

void CarDynamics::interpolateWheelContacts() {
	MathVector<float, 3> rayDir = getDownVector();
	for (int i = 0; i < numWheels; i++) {
		MathVector<float, 3> rayStart = localToWorld(wheels[i].getExtendedPosition());
		rayStart = rayStart - rayDir * wheels[i].getRadius();
		wheelContact[i].castRay(rayStart, rayDir, 1.5f); // From "par.cpp" (raylen)
	}
}

void CarDynamics::applyEngineTorqueToBody() {
	MathVector<double, 3> engineTorque(-engine.getTorque(), 0, 0);
	getBodyOrientation().rotateVector(engineTorque);
	applyTorque(engineTorque * 0.1); // Unwanted in jumps...? (Stuntrally comment)
}

void CarDynamics::applyAerodynamicsToBody() {
	MathVector<double, 3> windForce(0.), windTorque(0.), airVelocity = -getVelocity();
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

MathVector<double, 3> CarDynamics::applyTireForce(int i, const double normalForce, const Quaternion<double>& wheelSpace) {
	CarWheel& wheel = this->wheels[i];
	const CollisionContact& wheelCon = this->wheelContact[i];
	const TerrainSurface* surface = wheelCon.getSurface();
	const CarTire* tire = surface->tire;
	const MathVector<double, 3> surfaceNorm = wheelCon.getNormal();

	// Camber relative to surface (clockwise in wheel heading direction)
	MathVector<double, 3> wheelAxis(0, 1, 0);
	wheelSpace.rotateVector(wheelAxis); // Wheel axis in world space (wheel plane normal)
	double camberSin = wheelAxis.dot(surfaceNorm);
	double camberRad = asin(camberSin);
	wheel.setCamberDeg(camberRad * 180.0 / M_PI);

	// Tire space (SAE Tire Coordinate System)
	// Surface normal is z-axis
	// Wheel axis projected on surface plane is y-axis
	MathVector<double, 3> yAxis = wheelAxis - surfaceNorm * camberSin;
	MathVector<double, 3> xAxis = yAxis.cross(surfaceNorm);

	// Wheel center velocity in tire space
	MathVector<double, 3> hubVelocity;
	hubVelocity[0] = xAxis.dot(wheelVels[i]);
	hubVelocity[1] = yAxis.dot(wheelVels[i]);
	hubVelocity[2] = 0; // Unused

	// Rearward speed of the contact patch
	double patchSpeed = wheel.getAngularVelocity() * wheel.getRadius();

	// Friction force in tire space
	double frictionCoeff = surface->friction;
	MathVector<double, 3> frictionForce(0.);
	if (frictionCoeff > 0)
		frictionForce = tire->getForce(normalForce, frictionCoeff, hubVelocity, patchSpeed, camberRad, &wheel.slips);

	// Friction multipliers
	frictionForce[0] *= surface->frictionX;
	frictionForce[1] *= surface->frictionY;

	// No feedback

	// Friction force in world space
	MathVector<double, 3> worldFrictionForce = xAxis * frictionForce[0] + yAxis * frictionForce[1];

	// Fake viscous friction (sand, gravel, mud)
	MathVector<double, 3> wheelDrag = -(xAxis * hubVelocity[0] + yAxis * hubVelocity[1]) *
									  surface->rollingDrag;

	// Apply forces to body
	MathVector<double, 3> wheelNormal(0, 0, 1);
	wheelSpace.rotateVector(wheelNormal);
	MathVector<double, 3> contactPos = wheelPos[i] + wheelNormal * wheel.getRadius() * wheel.getRollHeight();
	applyForce(worldFrictionForce + surfaceNorm * normalForce + wheelDrag,
			   contactPos - getBodyPosition());

	return worldFrictionForce;
}

void CarDynamics::applyWheelTorque(double dt, double driveTorque, int i,
								   MathVector<double, 3> tireFriction,
		  	  	  	  	  	  	   const Quaternion<double>& wheelSpace) {
	CarWheel& wheel = wheels[i];
	CarBrake& brake = brakes[i];

	wheel.integrateStep1(dt);

	(-wheelSpace).rotateVector(tireFriction);

	// Torques acting on wheel
	double frictionTorque = tireFriction[0] * wheel.getRadius();
	double wheelTorque = driveTorque - frictionTorque;
	double lockUpTorque = wheel.getLockUpTorque(dt) - wheelTorque;
	double angVel = fabs(wheel.getAngularVelocity());
	double brakeTorque = brake.getTorque() + wheel.fluidRes * angVel; // Fluid resistance

	// Brake and rolling resistance torque should never exceed lock up torque
	if (lockUpTorque >= 0 && lockUpTorque > brakeTorque) {
		brake.setWillLock(false); wheelTorque += brakeTorque; // Brake torque in same dir as lock up torque
	} else if (lockUpTorque < 0 && lockUpTorque < -brakeTorque) {
		brake.setWillLock(false); wheelTorque -= brakeTorque;
	} else {
		brake.setWillLock(true);  wheelTorque = wheel.getLockUpTorque(dt);
	}

	// Set wheel torque due to tire rolling resistance
	double rollRes = wheel.getRollingResistance(wheel.getAngularVelocity(),
												wheelContact[i].getSurface()->rollingResist);
	double tireRollResTorque = -rollRes * wheel.getRadius();

	wheel.setTorque(wheelTorque * 0.5 + tireRollResTorque);
	wheel.integrateStep2(dt);

	// Two-wheel driving ("bike align straight torque from wheels")
	if (numWheels == 2) {
		double v = getSpeedDir() * 1./50.;
		v = 0.05 + 0.95 * std::min(1.0, v);
		MathVector<float, 3> dn = getDownVector();

		for (int w = 0; w < numWheels; ++w)
		if (wheelContact[w].getCollisionObject()) {
			MathVector<float, 3> n = wheelContact[w].getNormal();
			MathVector<float, 3> t = dn.cross(n);
			(-getBodyOrientation()).rotateVector(t);
			double x = t[0] * -1000. * v * 22;
			MathVector<float, 3> v(x,0,0);
			(-getBodyOrientation()).rotateVector(v);
			applyTorque(v);
		}
	}
}

// Anti-lock braking system
void CarDynamics::doABS(int i, double normalForce) {
	WheelPosition wp = WheelPosition(i);
	double brakeThresh = 0.1;
	double brakeSetting = brakes[i].getBrakeFactor();

	// Brake setting must be past threshold
	if (brakeSetting > brakeThresh) {
		double maxSpeed = 0;
		for (int j = 0; j < numWheels; j++) {
			maxSpeed = std::max(maxSpeed, wheels[i].getAngularVelocity());
		}

		// Wheels must be moving fast enough
		if (maxSpeed > 6.0) {
			double sp(0), ah(0); // sp is the ideal slip ratio given tire loading
			getTire(wp)->lookUpSigmaHatAlphaHat(normalForce, sp, ah);

			double error = -wheels[i].slips.slide - sp;
			double thresholdEng = 0.0, thresholdDis = -sp / 2.0;

			if (error > thresholdEng && !absActive[i]) absActive[i] = true;
			if (error < thresholdDis &&  absActive[i]) absActive[i] = false;
		} else {
			absActive[i] = false;
		}
	} else {
		absActive[i] = false;
	}

	if (absActive[i]) { brakes[wp].setBrakeFactor(0.0); }
}

// Traction control system
void CarDynamics::doTCS(int i, double normalForce) {
	double gasThresh = 0.1;
	double gas = engine.getThrottle();

	// Throttle must be past threshold
	if (gas > gasThresh) {
		// See if we're spinning faster than the rest of the wheels
		double maxSpinDiff = 0;
		double myRotationalSpeed = wheels[i].getAngularVelocity();
		for (int j = 0; j < numWheels; j++) {
			double spinDiff = myRotationalSpeed - wheels[j].getAngularVelocity();
			spinDiff = fabs(spinDiff);
			maxSpinDiff = std::max(spinDiff, maxSpinDiff);
		}

		// Don't engage if all wheels are moving at the same rate
		if (maxSpinDiff > 1.0) {
			double sp(0), ah(0);
			getTire(WheelPosition(i))->lookUpSigmaHatAlphaHat(normalForce, sp, ah);

			double sense = transmission.getGear() < 0 ? -1.0 : 1.0;

			double error = wheels[i].slips.slide * sense - sp;
			double thresholdEng = 0.0, thresholdDis = -sp / 2.0;

			if (error > thresholdEng && !tcsActive[i]) tcsActive[i] = true;
			if (error < thresholdDis &&  tcsActive[i]) tcsActive[i] = false;

			if (tcsActive[i]) {
				double curClutch = clamp(clutch.getClutch(), 0.0, 1.0);
				gas -= error * 10.0 * curClutch;
				gas = clamp(gas, 0.0, 1.0);
				engine.setThrottle(gas);
			}
		} else {
			tcsActive[i] = false;
		}
	} else {
		tcsActive[i] = false;
	}
}
