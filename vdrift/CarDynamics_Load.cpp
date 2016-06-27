#include "CarDynamics.hpp"
#include "CarConstants.hpp"

#include "../terrain/ShapeData.hpp"

CarDynamics::CarDynamics()
	: world(0), chassis(0), whTrigs(0),
	  drive(AWD), prevVel(0, 0, 0),
	  autoclutch(true), autoshift(true), autorear(true), shifted(true),
	  shiftTime(0.2), remShiftTime(0.0), lastAutoClutch(1.0), gearToShift(0),
	  steerValue(0.f), maxAngle(45.0), angularDamping(0.4) {
	setNumWheels(DEF_WHEEL_COUNT);
}

CarDynamics::~CarDynamics() {
	removeBullet();
}

void CarDynamics::setNumWheels(int nw) {
	numWheels = nw;

	brakes.resize(numWheels);
	suspension.resize(numWheels);
	wheels.resize(numWheels);
	wheelVels.resize(numWheels);
	wheelPos.resize(numWheels);
	wheelRots.resize(numWheels);
	wheelContact.resize(numWheels);
}

bool CarDynamics::load(ConfigFile& cf) {
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

			float friction, maxPressure, area, bias, radius, handbrake = 0;

			if (!cf.getParam(searchStr + "friction", friction)) return false;
			brakes[wl].setFriction(friction); brakes[wr].setFriction(friction);

			if (!cf.getParam(searchStr + "area", area)) return false;
			brakes[wl].setArea(area); brakes[wr].setArea(area);

			if (!cf.getParam(searchStr + "radius", radius)) return false;
			brakes[wl].setRadius(radius); brakes[wr].setRadius(radius);

			cf.getParam(searchStr + "handbrake", handbrake);
			brakes[wl].setHandbrake(handbrake); brakes[wr].setHandbrake(handbrake);

			if (!cf.getParam(searchStr + "bias", bias)) return false;
			brakes[wl].setBias(bias); brakes[wr].setBias(bias);

			if (!cf.getParam(searchStr + "max-pressure", maxPressure)) return false;
			brakes[wl].setMaxPressure(maxPressure); brakes[wr].setMaxPressure(maxPressure);
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

			float springConstant, bounce, rebound, travel, camber, caster, toe, antiRoll;

			if (!cf.getParam(searchStr + "spring-constant", springConstant)) return false;
			suspension[wl].setSpringConstant(springConstant); suspension[wr].setSpringConstant(springConstant);

			if (!cf.getParam(searchStr + "bounce", bounce)) return false;
			suspension[wl].setBounce(bounce); suspension[wr].setBounce(bounce);

			if (!cf.getParam(searchStr + "rebound", rebound)) return false;
			suspension[wl].setRebound(rebound); suspension[wr].setRebound(rebound);

			//TODO Load from suspension file
			std::vector<std::pair<double, double> > damper, spring;
			cf.getPoints("suspension-" + pos, "damper-factor", damper);
			suspension[wl].setDamperFactorPoints(damper); suspension[wr].setDamperFactorPoints(damper);
			cf.getPoints("suspension-" + pos, "spring-factor", spring);
			suspension[wl].setSpringFactorPoints(spring); suspension[wr].setSpringFactorPoints(spring);

			if (!cf.getParam(searchStr + "travel", travel)) return false;
			suspension[wl].setTravel(travel); suspension[wr].setTravel(travel);

			if (!cf.getParam(searchStr + "camber", camber)) return false;
			suspension[wl].setCamber(camber); suspension[wr].setCamber(camber);

			if (!cf.getParam(searchStr + "caster", caster)) return false;
			suspension[wl].setCaster(caster); suspension[wr].setCaster(caster);

			if (!cf.getParam(searchStr + "toe", toe)) return false;
			suspension[wl].setToe(toe); suspension[wr].setToe(toe);

			if (!cf.getParam(searchStr + "anti-roll", antiRoll)) return false;
			suspension[wl].setAntiRollK(antiRoll); suspension[wr].setAntiRollK(antiRoll);

			// Hinges
			float hinge[ConfigVariable::V_SIZE];
			MathVector<double, 3> vec;

			if (!cf.getParam("suspension-" + possh + "L.hinge", hinge)) return false;
			for (int i = 0; i < 3; i++)
				hinge[i] = std::max(std::min(hinge[i], 100.f), -100.f);
			if (version == 2) versionConvert(hinge[0], hinge[1], hinge[2]);
			vec.set(hinge[0], hinge[1], hinge[2]);
			suspension[wl].setHinge(vec);

			if (!cf.getParam("suspension-" + possh + "R.hinge", hinge)) return false;
			for (int i = 0; i < 3; i++)
				hinge[i] = std::max(std::min(hinge[i], 100.f), -100.f);
			if (version == 2) versionConvert(hinge[0], hinge[1], hinge[2]);
			vec.set(hinge[0], hinge[1], hinge[2]);
			suspension[wr].setHinge(vec);
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

			if (!cf.getParam(searchStr + "mass", mass)) return false;
			wheels[wp].setMass(mass);

			if (!cf.getParam(searchStr + "roll-height", rollH)) return false;
			wheels[wp].setRollHeight(rollH);

			if (!cf.getParam(searchStr + "position", pos)) return false;
			if (version == 2) versionConvert(pos[0], pos[1], pos[2]);
			vec.set(pos[0], pos[1], pos[2]);
			wheels[wp].setExtendedPosition(vec);

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
		addAerodynamicDevice(vec, dragArea, dragCoeff, 0, 0, 0);

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
			addAerodynamicDevice(vec, dragArea, dragCoeff, liftArea, liftCoeff, liftEff);
		}
	}

	//TODO Skipped hover params

	calculateMass();

	return true;
}

void CarDynamics::init(MathVector<double, 3> pos, Quaternion<double> rot, CollisionWorld& world) {
	this->world = &world;

	MathVector<double, 3> zero((double)0);

	body.setPosition(pos); body.setOrientation(rot);
	body.setInitialForce(zero); body.setInitialTorque(zero);

	engine.setInitialConditions();

	// Init chassis
	btTransform tr; tr.setIdentity();
	AABB<float> box;
	for (int i = 0; i < numWheels; i++) {
		MathVector<float, 3> wheelPos = getLocalWheelPosition(WheelPosition(i), 0);
		AABB<float> wheelAABB;
		wheelAABB.setFromCorners(wheelPos, wheelPos);
		box.combineWith(wheelAABB);
	}

	const MathVector<double, 3> verticalMargin(0, 0, 0.3);
	btVector3 origin = toBulletVector(box.getCenter() + verticalMargin - centerOfMass);
	btVector3 size = toBulletVector(box.getSize() - verticalMargin);

	// Assuming the vehicle is not a sphere... of course
	// y is length, x is width, h is height
	btCollisionShape* chassisShape;
	{
		btScalar w = size.getX() * 0.2, r = size.getZ() * 0.3, h = 0.45;

		//TODO I think this is what the Stuntrally devs meant (based on their formatting)
		btScalar l0 = 0.f, w0 = 0.f, h0 = 0.f;
		if (collR > 0.f) { r = collR; l0 = collLofs; }
		if (collW > 0.f) { w = collW; w0 = collWofs; }
		if (collH > 0.f) { h = collH; h0 = collHofs; }
		origin = btVector3(l0, w0, h0);

		const int numSph = 14; int i = 0;
		btScalar rad[numSph]; btVector3 posi[numSph];

		btScalar r2 = r * collR2m;
		btScalar l1 = collPosLFront, l2 = collPosLBack, l1m = l1 * 0.5, l2m = l2 * 0.5;
		float ww = 1.f;
		float wt = collTopWMul * ww;

		rad[i] = r2;  posi[i] = btVector3( l1 , -w*ww, -h*collFrHMul);  ++i;  // front
		rad[i] = r2;  posi[i] = btVector3( l1 ,  w*ww, -h*collFrHMul);  ++i;
		rad[i] = r;   posi[i] = btVector3( l1m, -w*ww, -h*collFrHMul);  ++i;  // front near
		rad[i] = r;   posi[i] = btVector3( l1m,  w*ww, -h*collFrHMul);  ++i;

		rad[i] = r;   posi[i] = btVector3( l2m, -w,    -h);  ++i;  // rear near
		rad[i] = r;   posi[i] = btVector3( l2m,  w,    -h);  ++i;
		rad[i] = r2;  posi[i] = btVector3( l2 , -w,    -h);  ++i;  // rear
		rad[i] = r2;  posi[i] = btVector3( l2 ,  w,    -h);  ++i;

		rad[i] = r2;  posi[i] = btVector3( collTopFr,  -w*wt, h*collTopFrHm  );  ++i;  // top
		rad[i] = r2;  posi[i] = btVector3( collTopFr,   w*wt, h*collTopFrHm  );  ++i;
		rad[i] = r2;  posi[i] = btVector3( collTopMid, -w*wt, h*collTopMidHm );  ++i;
		rad[i] = r2;  posi[i] = btVector3( collTopMid,  w*wt, h*collTopMidHm );  ++i;
		rad[i] = r2;  posi[i] = btVector3( collTopBack,-w*wt, h*collTopBackHm);  ++i;  // top rear
		rad[i] = r2;  posi[i] = btVector3( collTopBack, w*wt, h*collTopBackHm);  ++i;

		for (i=0; i < numSph; ++i)
			posi[i] += origin;
		chassisShape = new btMultiSphereShape(posi, rad, numSph);
		chassisShape->setMargin(0.02f);
	}

	double chassisMass = body.getMass();
	Matrix3<double> inertia = body.getInertia();
	btVector3 chassisInertia(inertia[0], inertia[4], inertia[8]);

	btTransform transform;
	transform.setOrigin(toBulletVector(pos));
	transform.setRotation(toBulletQuaternion(rot));
	btDefaultMotionState* chassisState = new btDefaultMotionState();
	chassisState->setWorldTransform(transform);

	btRigidBody::btRigidBodyConstructionInfo info(chassisMass, chassisState, chassisShape, chassisInertia);
	info.m_angularDamping = angularDamping;
	info.m_restitution = 0.0;
	info.m_friction = collFriction;  /// 0.4-0.7
	shapes.push_back(chassisShape);

	//TODO The second "true" is hard-coded; assume we allow car collisions!
	chassis = world.addRigidBody(info, true, true); rigids.push_back(chassis);
	chassis->setActivationState(DISABLE_DEACTIVATION);
	chassis->setUserPointer(new ShapeData(ShapeType::Car, this));

	world.getDynamicsWorld()->addAction(this); actions.push_back(this);

	// Join chassis and wheel triggers
	{
		for (int w = 0; w < numWheels; w++) {
			WheelPosition wp; wp = WheelPosition(w);
			double whRad = wheels[wp].getRadius() * 1.2;
			MathVector<float, 3> wheelPos = getWheelPosition(wp, 0);
			wheelPos[0] += collLofs;
			wheelPos[2] += collFlTrigH;

			btSphereShape* whSph = new btSphereShape(whRad);
			whTrigs = new btRigidBody(0.001f, 0, whSph);

			whTrigs->setUserPointer(new ShapeData(ShapeType::Wheel, this));
			whTrigs->setActivationState(DISABLE_DEACTIVATION);
			whTrigs->setCollisionFlags(whTrigs->getCollisionFlags() | btCollisionObject::CF_NO_CONTACT_RESPONSE);

			world.getDynamicsWorld()->addRigidBody(whTrigs); rigids.push_back(whTrigs);
			world.addShape(whSph); shapes.push_back(whSph);

			btTypedConstraint* constr = new btPoint2PointConstraint(*chassis, *whTrigs, toBulletVector(wheelPos), btVector3(0, 0, 0));
			world.getDynamicsWorld()->addConstraint(constr, true); constraints.push_back(constr);
		}

		//TODO Initialization for buoyancy computations
	}

	// Init wheels and suspension
	for (int i = 0; i < numWheels; i++) {
		wheels[i].setInitialConditions();
		wheelVels[i].set(0.0);
		wheelPos[i] = getWheelPositionAtDisplacement(WheelPosition(i), 0);
		wheelRots[i] = rot * getWheelSteeringAndSuspensionOrientation(WheelPosition(i));
	}

	alignWithGround();
}

void CarDynamics::removeBullet() {
	int i, c;
	// Remove constraints first
	for (i = 0; i < constraints.size(); i++) {
		world->getDynamicsWorld()->removeConstraint(constraints[i]);
		delete constraints[i];
	}
	constraints.resize(0);

	for (i = rigids.size() - 1; i >= 0; i--) {
		btRigidBody* body = rigids[i];
//		if (body && body->getMotionState()) delete body->getMotionState(); TODO Commented due to double free error

		world->getDynamicsWorld()->removeRigidBody(body);

//		ShapeData* sd = (ShapeData*)body->getUserPointer(); TODO When ready
//		delete sd;
//		delete body; TODO Commented due to double free error
	}

	for (i = 0; i < shapes.size(); i++) {
		btCollisionShape* shape = shapes[i];
		world->removeShape(shape);

		if (shape->isCompound()) {
			btCompoundShape* cs = (btCompoundShape *)shape;
			for (c = 0; c < cs->getNumChildShapes(); ++c)
				delete cs->getChildShape(c);
		}
//		ShapeData* sd = (ShapeData*)shape->getUserPointer(); TODO
//		delete sd;
//		delete shape; TODO Commented due to double free error
	}
	shapes.resize(0);

	for (i = 0; i < actions.size(); ++i) world->getDynamicsWorld()->removeAction(actions[i]);
	actions.resize(0);
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
	newPos[0] += comOfsL;
	newPos[2] += comOfsH;
	massOnlyParticles.push_back(std::pair<double, MathVector<double, 3> >(newMass, newPos));
}

void CarDynamics::addAerodynamicDevice(const MathVector<double, 3>& newPos, double dragFrontalArea,
									   double dragCoefficient, double liftSurfaceArea, double liftCoefficient, double liftEfficiency) {
	CarAero aero;
	aero.set(newPos, dragFrontalArea, dragCoefficient, liftSurfaceArea, liftCoefficient, liftEfficiency);
	aerodynamics.push_back(aero);
}

void CarDynamics::calculateMass() {
	typedef std::pair<double, MathVector<double, 3> > MassPair;

	double totalMass(0);
	centerOfMass.set((double)0);

	//
	for (std::list< MassPair >::iterator i = massOnlyParticles.begin(); i != massOnlyParticles.end(); i++) {
		totalMass += i->first;
		centerOfMass = centerOfMass + i->second * i->first;
	}

	// Account for fuel
	totalMass += fuelTank.getMass();
	centerOfMass = centerOfMass + fuelTank.getPosition() * fuelTank.getMass();

	body.setMass(totalMass);
	centerOfMass = centerOfMass * (1.0 / totalMass);

	// Calculate inertia tensor
	Matrix3<double> inertia;
	for (int i = 0; i < 9; i++) { inertia[i] = 0; }

	for (std::list< MassPair >::iterator i = massOnlyParticles.begin(); i != massOnlyParticles.end(); i++) {
		// Transform into RigidBody coordinates
		MathVector<double, 3> pos = i->second - centerOfMass;
		double mass = i->first;
		inertia[0] += mass * (pos[1] * pos[1] + pos[2] * pos[2]);
		inertia[1] -= mass * (pos[0] * pos[1]);
		inertia[2] -= mass * (pos[0] * pos[2]);
		inertia[3] = inertia[1];
		inertia[4] += mass * (pos[2] * pos[2] + pos[0] * pos[0]);
		inertia[5] -= mass * (pos[1] * pos[2]);
		inertia[6] = inertia[2];
		inertia[7] = inertia[5];
		inertia[8] += mass * (pos[0] * pos[0] + pos[1] * pos[1]);
	}
	body.setInertia(inertia);
}
