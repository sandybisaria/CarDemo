#include "CarDynamics.hpp"
#include "CarConstants.hpp"

CarDynamics::CarDynamics() {
	setNumWheels(DEF_WHEEL_COUNT);
}

void CarDynamics::setNumWheels(int nw) {
	numWheels = nw;

	brakes.resize(numWheels);
	suspension.resize(numWheels);
	wheels.resize(numWheels);
}

bool CarDynamics::loadFromConfig(ConfigFile& cf) {
	int nw = 0;
	cf.getParam("wheels", nw);
	if (nw >= MIN_WHEEL_COUNT && nw <= MAX_WHEEL_COUNT)
		setNumWheels(nw);

	int version = 2;
	cf.getParam("version", version);
	if (version > 2)
		return false; //TODO Error: Unsupported version

	float tempVec[ConfigVariable::V_SIZE];

	// Load engine params
	{
		float mass, rpmLimit, inertia, friction, startRPM, stallRPM, fuelCons;
		MathVector<double, 3> position;

		//TODO Ignored engine.sound

		if (!cf.getParam("engine.rpm-limit", rpmLimit)) return false;
		engine.setMaxRPM(rpmLimit);

		if (!cf.getParam("engine.inertia", inertia)) return false;
		engine.setInertia(inertia);

		if (!cf.getParam("engine.friction", friction)) return false;
		engine.setFrictionB(friction);

		if (!cf.getParam("engine.start-rpm", startRPM)) return false;
		engine.setStartRPM(startRPM);

		if (!cf.getParam("engine.stall-rpm", startRPM)) return false;
		engine.setStallRPM(stallRPM);

		if (!cf.getParam("engine.fuel-consumption", fuelCons)) return false;
		engine.setFuelConsumption(fuelCons);

		if (!cf.getParam("engine.mass", mass)) return false;
		if (!cf.getParam("engine.position", tempVec)) return false;

		if (version == 2) versionConvert(tempVec[0], tempVec[1], tempVec[2]);
		position.set(tempVec[0], tempVec[1], tempVec[2]);
		engine.setMass(mass);
		engine.setPosition(position);

		addMassParticle(mass, position);

		float mul(1), maxTorque(0);
		cf.getParam("engine.torque-val-mul", mul);

		float torquePoint[ConfigVariable::V_SIZE];
		std::string torqueStr("engine.torque-curve-00");
		std::vector<std::pair<double, double> > torques;

		int curveNum(0);
		while (cf.getParam(torqueStr, torquePoint)) {
			maxTorque = std::max(maxTorque, torquePoint[1] * mul);
			torques.push_back(std::pair<float, float>(torquePoint[0], torquePoint[1] * mul));

			curveNum++;
			std::stringstream str;
			str << "engine.torque-curve-";
			str.width(2); str.fill('0'); str << curveNum;
			torqueStr = str.str();
		}

		if (torques.size() <= 1) return false;

		engine.setTorqueCurve(rpmLimit, torques);

		// Load clutch params
		{
			float mul;
			if (!cf.getParam("clutch.max-torque-mul", mul)) return false;
			clutch.setMaxTorque(maxTorque * mul);
		}

		mul = 1;
		if (cf.getParam("engine.real-po-tq-mul", mul))
			engine.realPowTorqueMul = mul;

		//TODO Skipped engine.sound-vol-mul
	}

	// Load transmission params
	{
		float time(0), ratio;
		int gears;

		cf.getParam("transmission.shift-delay", time);
		shiftTime = time;

		if (!cf.getParam("transmission.gear-ratio-r", ratio)) return false;
		transmission.setGearRatio(-1, ratio);

		if (!cf.getParam("transmission.gears", gears)) return false;
		for (int i = 1; i <= gears; i++) {
			std::stringstream s;
			s << "transmission.gear-ratio-" << i;
			if (!cf.getParam(s.str(), ratio)) return false;
			transmission.setGearRatio(i, ratio);
		}
	}

	// Load differential params
	{
		std::string driveType;
		if (!cf.getParam("drive", driveType)) return false;
		setDrive(driveType);

		float finalDrive, a, aTq(0), aTqDec(0);
		if (drive == AWD && cf.getParam("diff-center.final-drive", finalDrive)) {
			cf.getParam("diff-rear.anti-slip", a);
			cf.getParam("diff-rear.torque", aTq);
			cf.getParam("diff-rear.torque-dec", aTqDec);
			diffRear.setFinalDrive(1.0); diffRear.setAntiSlip(a, aTq, aTqDec);

			cf.getParam("diff-front.anti-slip", a);
			cf.getParam("diff-front.torque", aTq);
			cf.getParam("diff-front.torque-dec", aTqDec);
			diffFront.setFinalDrive(1.0); diffFront.setAntiSlip(a, aTq, aTqDec);

			cf.getParam("diff-center.torque", aTq);
			cf.getParam("diff-center.torque-dec", aTqDec);
			diffCenter.setFinalDrive(finalDrive); diffCenter.setAntiSlip(a, aTq, aTqDec);
		} else {
			if (!cf.getParam("differential.final-drive", finalDrive)) return false;
			if (!cf.getParam("differential.anti-slip", a)) return false;
			cf.getParam("differential.torque", aTq);
			cf.getParam("differential.torque-dec", aTqDec);

			if (drive == RWD) {
				diffRear.setFinalDrive(finalDrive); diffRear.setAntiSlip(a, aTq, aTqDec);
			} else if (drive == FWD) {
				diffFront.setFinalDrive(finalDrive); diffFront.setAntiSlip(a, aTq, aTqDec);
			} else {
				// drive == AWD
				diffRear.setFinalDrive(1.0); diffRear.setAntiSlip(a, aTq, aTqDec);
				diffFront.setFinalDrive(1.0); diffFront.setAntiSlip(a, aTq, aTqDec);
				diffCenter.setFinalDrive(finalDrive); diffCenter.setAntiSlip(a, aTq, aTqDec);
			}
		}
	}

	// Load brake params
	{
		int iMax = std::max(2, numWheels / 2);
		for (int i = 0; i < iMax; i++) {
			WheelPosition wl, wr;
			std::string pos;
			getWheelPosStr(i, numWheels, wl, wr, pos);
			std::string searchStr = "brakes-" + pos + ".";

			CarBrake b = brakes[wl], b2 = brakes[wr];

			float friction, maxPressure, area, bias, radius, handbrake = 0;

			if (!cf.getParam(searchStr + "friction", friction)) return false;
			b.setFriction(friction); b2.setFriction(friction);

			if (!cf.getParam(searchStr + "area", area)) return false;
			b.setArea(area); b2.setArea(area);

			if (!cf.getParam(searchStr + "radius", radius)) return false;
			b.setRadius(radius); b2.setRadius(radius);

			cf.getParam(searchStr + "handbrake", handbrake);
			b.setHandbrake(handbrake); b2.setHandbrake(handbrake);

			if (!cf.getParam(searchStr + "bias", bias)) return false;
			b.setBias(bias); b2.setBias(bias);

			if (!cf.getParam(searchStr + "max-pressure", maxPressure)) return false;
			b.setMaxPressure(maxPressure); b2.setMaxPressure(maxPressure);
		}
	}

	// Load fuel tank params
	{
		MathVector<double, 3> position;
		float cap, vol, fuelDens;

		if (!cf.getParam("fuel-tank.capacity", cap)) return false;
		fuelTank.setCapacity(cap);

		if (!cf.getParam("fuel-tank.volume", vol)) return false;
		fuelTank.setVolume(vol);

		if (!cf.getParam("fuel-tank.fuel-density", fuelDens)) return false;
		fuelTank.setDensity(fuelDens);

		if (!cf.getParam("fuel-tank.position", tempVec)) return false;
		if (version == 2) versionConvert(tempVec[0], tempVec[1], tempVec[2]);
		position.set(tempVec[0], tempVec[1], tempVec[2]);
		fuelTank.setPosition(position);
	}

	// Load the suspension
	{
		for (int i = 0; i < numWheels / 2; i++) {
			std::string pos = "front", possh = "F";
			WheelPosition wl = FRONT_LEFT, wr = FRONT_RIGHT;
			if (i >= 1) {
				pos = "rear"; possh = "R"; wl = REAR_LEFT; wr = REAR_RIGHT;
			}

			if (i == 2) {
				wl = REAR2_LEFT; wr = REAR2_RIGHT;
			} else if (i == 3) {
				wl = REAR3_LEFT; wr = REAR3_RIGHT;
			}

			std::string searchStr = "suspension-" + pos + ".";
			CarSuspension s = suspension[wl], s2 = suspension[wr];

			float springConstant, bounce, rebound, travel, camber, caster, toe, antiRoll;

			if (!cf.getParam(searchStr + "spring-constant", springConstant)) return false;
			s.setSpringConstant(springConstant); s2.setSpringConstant(springConstant);

			if (!cf.getParam(searchStr + "bounce", bounce)) return false;
			s.setBounce(bounce); s2.setBounce(bounce);

			if (!cf.getParam(searchStr + "rebound", rebound)) return false;
			s.setRebound(rebound); s2.setRebound(rebound);

			//TODO Load from suspension file
			std::vector<std::pair<double, double> > damper, spring;
			cf.getPoints("suspension-" + pos, "damper-factor", damper);
			s.setDamperFactorPoints(damper); s2.setDamperFactorPoints(damper);
			cf.getPoints("suspension-" + pos, "spring-factor", spring);
			s.setSpringFactorPoints(spring); s2.setSpringFactorPoints(spring);

			if (!cf.getParam(searchStr + "travel", travel)) return false;
			s.setTravel(travel); s2.setTravel(travel);

			if (!cf.getParam(searchStr + "camber", camber)) return false;
			s.setCamber(camber); s2.setCamber(camber);

			if (!cf.getParam(searchStr + "caster", caster)) return false;
			s.setCaster(caster); s2.setCaster(caster);

			if (!cf.getParam(searchStr + "toe", toe)) return false;
			s.setToe(toe); s2.setToe(toe);

			if (!cf.getParam(searchStr + "anti-roll", antiRoll)) return false;
			s.setAntiRollK(antiRoll); s2.setAntiRollK(antiRoll);

			// Hinges
			float hinge[ConfigVariable::V_SIZE];
			MathVector<double, 3> vec;

			if (!cf.getParam("suspension-" + possh + "L.hinge", hinge)) return false;
			for (int i = 0; i < 3; i++)
				hinge[i] = std::max(std::min(hinge[i], 100.f), -100.f);
			if (version == 2) versionConvert(hinge[0], hinge[1], hinge[2]);
			vec.set(hinge[0], hinge[1], hinge[2]);
			s.setHinge(vec);

			if (!cf.getParam("suspension-" + possh + "R.hinge", hinge)) return false;
			for (int i = 0; i < 3; i++)
				hinge[i] = std::max(std::min(hinge[i], 100.f), -100.f);
			if (version == 2) versionConvert(hinge[0], hinge[1], hinge[2]);
			vec.set(hinge[0], hinge[1], hinge[2]);
			s2.setHinge(vec);
		}
	}

	// Load wheel params
	{
		for (int i = 0; i < numWheels; i++) {
			std::string wt = WHEEL_TYPE[i];
			WheelPosition wp = WheelPosition(i);

			float rollH, mass;
			float pos[ConfigVariable::V_SIZE];
			MathVector<double, 3> vec;

			std::string searchStr = "wheel-" + wt + ".";
			CarWheel w = wheels[wp];

			if (!cf.getParam(searchStr + "mass", mass)) return false;
			w.setMass(mass);

			if (!cf.getParam(searchStr + "roll-height", rollH)) return false;
			w.setRollHeight(rollH);

			if (!cf.getParam(searchStr + "position", pos)) return false;
			if (version == 2) versionConvert(pos[0], pos[1], pos[2]);
			vec.set(pos[0], pos[1], pos[2]);
			w.setExtendedPosition(vec);

			addMassParticle(mass, vec);
		}

		// Rotational inertia param is located in the tire section
		float front, rear;
		if (cf.getParam("tire-both.rotational-inertia", front)) {
			rear = front;
		} else {
			if (!cf.getParam("tire-front.rotational-inertia", front)) return false;
			if (!cf.getParam("tire-rear.rotational-inertia", rear)) return false;
		}
		wheels[FRONT_LEFT].setInertia(front);
		wheels[FRONT_RIGHT].setInertia(front);
		for (int i = REAR_LEFT; i <= REAR3_RIGHT; i++) {
			if (i < numWheels) wheels[i].setInertia(rear);
		}
	}

	// Load tire params
	{
		float val;
		bool both = cf.getParam("tire-both.radius", val);

		int iMax = std::max(2, numWheels / 2);
		for (int i = 0; i < iMax; i++) {
			WheelPosition wl, wr;
			std::string pos;
			getWheelPosStr(i, numWheels, wl, wr, pos);
			if (both) pos = "both";

			float rollRes[ConfigVariable::V_SIZE];
			if (!cf.getParam("tire-" + pos + ".rolling-resistance", rollRes)) return false;
			wheels[wl].setRollingResistance(rollRes[0], rollRes[1]);
			wheels[wr].setRollingResistance(rollRes[0], rollRes[1]);

			float radius;
			if (!cf.getParam("tire-" + pos + ".radius", radius)) return false;
			wheels[wl].setRadius(radius);
			wheels[wr].setRadius(radius);
		}
	}

	// Load mass-only particles
	{
		MathVector<double, 3> position;
		float pos[3], mass;

		if (cf.getParam("contact-points.mass", mass)) {
			int paramNum(0);
			std::string paramName("contact-points.position-00");
			while (cf.getParam(paramName, pos)) {
				if (version == 2)  versionConvert(pos[0],pos[1],pos[2]);
				position.set(pos[0],pos[1],pos[2]);
				addMassParticle(mass, position);
				paramNum++;
				std::stringstream str;
				str << "contact-points.position-";
				str.width(2);  str.fill('0'); str << paramNum;
				paramName = str.str();
			}
		}

		int paramNum(0);
		std::string paramName("particle-00");
		while (cf.getParam(paramName + ".mass", mass)) {
			if (!cf.getParam(paramName + ".position", pos))
			if (version == 2)  versionConvert(pos[0],pos[1],pos[2]);
			position.set(pos[0],pos[1],pos[2]);
				addMassParticle(mass, position);
			paramNum++;
			std::stringstream str;
			str << "particle-";
			str.width(2);  str.fill('0'); str << paramNum;
			paramName = str.str();
		}
	}

	// Load max steering angle
	{
		float maxAngle = 26.f;
		if (!cf.getParam("steering.max-angle", maxAngle)) return false;
		setMaxSteeringAngle(maxAngle);

		//TODO Skipped steering.flip-pow-mul
	}

	// Load steering angular damping
	{
		float a = 0.4f;
		cf.getParam("steering.angular-damping", a);
		setAngularDamping(a);

		a=0.f; cf.getParam("rot_drag.roll", a); rotCoeff[0] = a;
		a=0.f; cf.getParam("rot_drag.pitch", a); rotCoeff[1] = a;
		a=0.f; cf.getParam("rot_drag.yaw", a); rotCoeff[2] = a;
		a=0.f; cf.getParam("rot_drag.yaw2", a); rotCoeff[3] = a;
	}

	// Load driver params
	{
		float mass, pos[3];
		MathVector<double, 3> vec;

		if (!cf.getParam("driver.mass", mass)) return false;
		if (!cf.getParam("driver.position", pos)) return false;
		if (version == 2) versionConvert(pos[0], pos[1], pos[2]);
		vec.set(pos[0], pos[1], pos[2]);
		addMassParticle(mass, vec);
	}

	// Load aerodynamics
	{
		float dragArea, dragCoeff, liftArea, liftCoeff, liftEff;
		float pos[3];
		MathVector<double, 3> vec;

		if (!cf.getParam("drag.frontal-area", dragArea)) return false;
		if (!cf.getParam("drag.drag-coefficient", dragCoeff)) return false;
		if (!cf.getParam("drag.position", pos)) return false;
		if (version == 2) versionConvert(pos[0], pos[1], pos[2]);
		vec.set(pos[0], pos[1], pos[2]);
//		addAerodynamicDevice(vec, dragArea, dragCoeff, 0, 0, 0); //FIXME

		for (int i = 0; i < 2; i++) {
			std::string type = (i == 1 ? "rear" : "front");
			std::string searchStr = "wing-" + type + ".";
			if (!cf.getParam(searchStr + "frontal-area", dragArea)) return false;
			if (!cf.getParam(searchStr + "drag-coefficient", dragCoeff)) return false;
			if (!cf.getParam(searchStr + "surface-area", liftArea)) return false;
			if (!cf.getParam(searchStr + "lift-coefficient", liftCoeff)) return false;
			if (!cf.getParam(searchStr + "efficiency", liftEff)) return false;
			if (!cf.getParam(searchStr + "position", pos)) return false;
			if (version == 2) versionConvert(pos[0], pos[1], pos[2]);
			vec.set(pos[0], pos[1], pos[2]);
	//		addAerodynamicDevice(vec, dragArea, dragCoeff, liftArea, liftCoeff, liftEff); //FIXME
		}
	}

	//TODO Skipped hover params

//	updateMass(); //FIXME

	return true;
}

void CarDynamics::setDrive(const std::string& newDrive) {
	if (newDrive == "RWD")
		drive = RWD;
	else if (newDrive == "FWD")
		drive = FWD;
	else {
		assert(newDrive == "AWD");
		drive = AWD;
	}
}

void CarDynamics::addMassParticle(double newMass, MathVector<double, 3> newPos) {
	//TODO When collision params are loaded, uncomment and adjust
//	newpos[0] += com_ofs_L;
//	newpos[2] += com_ofs_H;
	massOnlyParticles.push_back(std::pair<double, MathVector<double, 3> >(newMass, newPos));
}
