#include "CarDynamics.hpp"

#include "cardefs.h"

CarDynamics::CarDynamics() {
	setNumWheels(DEF_WHEELS);
}

CarDynamics::~CarDynamics() {

}

void CarDynamics::setNumWheels(int n) {
	numWheels = n;

	suspension.resize(n); wheel.resize(n);
	wheelVelocity.resize(n); wheelPosition.resize(n); wheelOrientation.resize(n);
	wheelContact.resize(n); brake.resize(n);
	absActive.resize(n); tcsActive.resize(n);

	iWheelOnRoad.resize(n); wheelTerrMtr.resize(n); wheelRoadMtr.resize(n);
	wheelSubmersion.resize(n); wheelFluidParticle.resize(n); wheelFluidDmg.resize(n);

	//TODO inFluidsWh.resize(n);

	for (int i = 0; i < n; i++) { wheelFluidParticle[i] = -1; } // Reset particle IDs
}

bool CarDynamics::load(CONFIGFILE& c) {
	int nw = 0;
	c.GetParam("wheels", nw);
	if (nw >= MIN_WHEELS && nw <= MAX_WHEELS)
		setNumWheels(nw);

	int version = 2;
	c.GetParam("version", version);
	if (version > 2) {
		//TODO Error: Unsupported .car version
		return false;
	}

	float temp_vec3[CONF_VEC_SIZE]; // For loading float arrays from config

	// Load the engine
	float mass, rpm_limit, inertia, friction, start_rpm, stall_rpm, fuel_consumption;
	MATHVECTOR<Dbl, 3> position;

	if (!c.GetParamE("engine.sound", engine.sound_name)) engine.sound_name = "engine";

	if (!c.GetParamE("engine.rpm-limit", rpm_limit))  return false;
	engine.SetRpmMax(rpm_limit);

	if (!c.GetParamE("engine.inertia", inertia))  return false;
	engine.SetInertia(inertia);

	if (!c.GetParamE("engine.friction", friction))  return false;
	engine.SetFrictionB(friction);

	if (!c.GetParamE("engine.start-rpm", start_rpm))  return false;
	engine.SetStartRPM(start_rpm);

	if (!c.GetParamE("engine.stall-rpm", stall_rpm))  return false;
	engine.SetStallRPM(stall_rpm);

	if (!c.GetParamE("engine.fuel-consumption", fuel_consumption))  return false;
	engine.SetFuelConsumption(fuel_consumption);

	if (!c.GetParamE("engine.mass", mass))  return false;
	if (!c.GetParamE("engine.position", temp_vec3))  return false;
	if (version == 2) convertV2to1(temp_vec3[0],temp_vec3[1],temp_vec3[2]);
	position.Set(temp_vec3[0],temp_vec3[1],temp_vec3[2]);
	engine.SetMass(mass);
	engine.SetPosition(position);
	addMassParticle(mass, position);

	float mul = 1.f, max_torque = 0;
	c.GetParam("engine.torque-val-mul", mul);

	float torque_point[3];
	std::string torque_str("engine.torque-curve-00");
	std::vector<std::pair<double, double> > torques;
	int curve_num = 0;
	while (c.GetParam(torque_str, torque_point)) {
		max_torque = std::max(max_torque, torque_point[1] * mul);
		torques.push_back(std::pair<float, float>(torque_point[0], torque_point[1] * mul));

		curve_num++;
		std::stringstream str;
		str << "engine.torque-curve-";
		str.width(2);  str.fill('0');
		str << curve_num;
		torque_str = str.str();
	}
	if (torques.size() <= 1) {
		//TODO Error: You must define at least 2 torque curve points.
		return false;
	}
	engine.SetTorqueCurve(rpm_limit, torques);

	// Load the clutch
	//max_torque = sliding * radius * area * max_pressure;
	if (!c.GetParamE("clutch.max-torque-mul", mul))  return false;
	clutch.SetMaxTorque(max_torque * mul);

	// Factor for stats  -
	mul = 1.f;
	if (c.GetParam("engine.real-pow-tq-mul", mul))
		engine.real_pow_tq_mul = mul;

	mul = 1.f;
	if (c.GetParam("engine.sound-vol-mul", mul))
		engineVolMul = mul;

	// Load the transmission
	float time = 0;
	float ratio;
	int gears;

	c.GetParam("transmission.shift-delay", time);
	shiftTime = time;

	if (!c.GetParamE("transmission.gear-ratio-r", ratio))  return false;
	transmission.SetGearRatio(-1, ratio);

	if (!c.GetParamE("transmission.gears", gears))  return false;

	for (int i = 0; i < gears; ++i) {
		std::stringstream s;
		s << "transmission.gear-ratio-" << i+1;
		if (!c.GetParamE(s.str(), ratio))  return false;
		transmission.SetGearRatio(i+1, ratio);
	}

	// Load the differential(s)
	std::string drivetype;
	if (!c.GetParamE("drive", drivetype))  return false;

	if (drivetype == "hover")  //>
	{	vtype = V_Spaceship;  drivetype = "AWD";  }
	else if (drivetype == "sphere")
	{	vtype = V_Sphere;  drivetype = "AWD";  }

	setDrive(drivetype);

	float final_drive, a, a_tq(0), a_tq_dec(0);
	/// New 3 sets
	if (drivetype == "AWD" &&
		c.GetParam("diff-center.final-drive", a)) {
		c.GetParamE("diff-rear.anti-slip", a);
		c.GetParam("diff-rear.torque", a_tq);  c.GetParam("diff-rear.torque-dec", a_tq_dec);
		diffRear.SetFinalDrive(1.0);  diffRear.SetAntiSlip(a, a_tq, a_tq_dec);

		c.GetParamE("diff-front.anti-slip", a);
		c.GetParam("diff-front.torque", a_tq);  c.GetParam("diff-front.torque-dec", a_tq_dec);
		diffFront.SetFinalDrive(1.0);  diffFront.SetAntiSlip(a, a_tq, a_tq_dec);

		c.GetParamE("diff-center.final-drive", final_drive);
		c.GetParamE("diff-center.anti-slip", a);
		c.GetParam("diff-center.torque", a_tq);  c.GetParam("diff-center.torque-dec", a_tq_dec);
		diffCenter.SetFinalDrive(final_drive);  diffCenter.SetAntiSlip(a, a_tq, a_tq_dec);
	} else { // old 1 for all
		if (!c.GetParamE("differential.final-drive", final_drive))  return false;
		if (!c.GetParamE("differential.anti-slip", a))  return false;
		c.GetParam("differential.torque", a_tq);
		c.GetParam("differential.torque-dec", a_tq_dec);

		if (drivetype == "RWD") {
			diffRear.SetFinalDrive(final_drive);
			diffRear.SetAntiSlip(a, a_tq, a_tq_dec);
		} else if (drivetype == "FWD") {
			diffFront.SetFinalDrive(final_drive);
			diffFront.SetAntiSlip(a, a_tq, a_tq_dec);
		} else if (drivetype == "AWD") {
			diffRear.SetFinalDrive(1.0);
			diffRear.SetAntiSlip(a, a_tq, a_tq_dec);

			diffFront.SetFinalDrive(1.0);
			diffFront.SetAntiSlip(a, a_tq, a_tq_dec);

			diffCenter.SetFinalDrive(final_drive);
			diffCenter.SetAntiSlip(a, a_tq, a_tq_dec);
		} else {
			//TODO Error: Unknown drive type
			return false;
		}
	}

	// Load the brake
	int ii = std::max(2, numWheels/2);
	for (int i = 0; i < ii; ++i) {
		WHEEL_POSITION wl, wr;  std::string pos;
		getWheelPosStr(i, numWheels, wl, wr, pos);

		float friction, max_pressure, area, bias, radius, handbrake = 0.f;

		if (!c.GetParamE("brakes-"+pos+".frictionAddMassParticle", friction))  return false;
		brake[wl].SetFriction(friction);  brake[wr].SetFriction(friction);

		if (!c.GetParamE("brakes-"+pos+".area", area))  return false;
		brake[wl].SetArea(area);  brake[wr].SetArea(area);

		if (!c.GetParamE("brakes-"+pos+".radius", radius))  return false;
		brake[wl].SetRadius(radius);  brake[wr].SetRadius(radius);

		c.GetParam("brakes-"+pos+".handbrake", handbrake);
		brake[wl].SetHandbrake(handbrake);  brake[wr].SetHandbrake(handbrake);

		if (!c.GetParamE("brakes-"+pos+".bias", bias))  return false;
		brake[wl].SetBias(bias);  brake[wr].SetBias(bias);

		if (!c.GetParamE("brakes-"+pos+".max-pressure", max_pressure))  return false;
		brake[wl].SetMaxPressure(max_pressure*bias);  brake[wr].SetMaxPressure(max_pressure*bias);
	}

	// Load the fuel tank
	float pos[3];
	float capacity, volume, fuel_density;

	if (!c.GetParamE("fuel-tank.capacity", capacity))  return false;
	fuelTank.SetCapacity(capacity);

	if (!c.GetParamE("fuel-tank.volume", volume))  return false;
	fuelTank.SetVolume(volume);

	if (!c.GetParamE("fuel-tank.fuel-density", fuel_density))  return false;
	fuelTank.SetDensity(fuel_density);

	if (!c.GetParamE("fuel-tank.position", pos))  return false;
	if (version == 2)  convertV2to1(pos[0],pos[1],pos[2]);
	position.Set(pos[0],pos[1],pos[2]);
	fuelTank.SetPosition(position);

	// Load the suspension
	for (int i = 0; i < numWheels/2; ++i) {
		std::string pos = "front", possh = "F";  WHEEL_POSITION wl = FRONT_LEFT, wr = FRONT_RIGHT;
		if (i >= 1){  pos = "rear";  possh = "R";  wl = REAR_LEFT;  wr = REAR_RIGHT;  }

		if (i == 2){  wl = REAR2_LEFT;  wr = REAR2_RIGHT;  } else
		if (i == 3){  wl = REAR3_LEFT;  wr = REAR3_RIGHT;  }

		float spring_constant, bounce, rebound, travel, camber, caster, toe, anti_roll;//, maxcompvel;

		if (!c.GetParamE("suspension-"+pos+".spring-constant", spring_constant))  return false;
		suspension[wl].SetSpringConstant(spring_constant);  suspension[wr].SetSpringConstant(spring_constant);

		if (!c.GetParamE("suspension-"+pos+".bounce", bounce))  return false;
		suspension[wl].SetBounce(bounce);  suspension[wr].SetBounce(bounce);

		if (!c.GetParamE("suspension-"+pos+".rebound", rebound))  return false;
		suspension[wl].SetRebound(rebound);  suspension[wr].SetRebound(rebound);

		std::string file;
		//TODO Hard-coded to avoid file read
		if (false && c.GetParam("suspension-"+pos+".factors-file", file)) {
			//TODO Add suspension map; refer to GAME::LoadSusp
//			int id = game->suspS_map[file]-1;
//			if (id == -1)  {  id = 0;
//				LogO(".car Error: Can't find suspension spring factors file: "+file);  }
//
//			suspension[wl].SetSpringFactorPoints(game->suspS[id]);  suspension[wr].SetSpringFactorPoints(game->suspS[id]);
//
//			id = game->suspD_map[file]-1;
//			if (id == -1)  {  id = 0;
//				LogO(".car Error: Can't find suspension damper factors file: "+file);  }
//
//			suspension[wl].SetDamperFactorPoints(game->suspD[id]);  suspension[wr].SetDamperFactorPoints(game->suspD[id]);
		} else {
			// Factor points
			std::vector<std::pair<double, double> > damper, spring;
			c.GetPoints("suspension-"+pos, "damper-factor", damper);
			suspension[wl].SetDamperFactorPoints(damper);  suspension[wr].SetDamperFactorPoints(damper);

			c.GetPoints("suspension-"+pos, "spring-factor", spring);
			suspension[wl].SetSpringFactorPoints(spring);  suspension[wr].SetSpringFactorPoints(spring);
		}

		if (!c.GetParamE("suspension-"+pos+".travel", travel))  return false;
		suspension[wl].SetTravel(travel);  suspension[wr].SetTravel(travel);

		if (!c.GetParamE("suspension-"+pos+".camber", camber))  return false;
		suspension[wl].SetCamber(camber);  suspension[wr].SetCamber(camber);

		if (!c.GetParamE("suspension-"+pos+".caster", caster))  return false;
		suspension[wl].SetCaster(caster);  suspension[wr].SetCaster(caster);

		if (!c.GetParamE("suspension-"+pos+".toe", toe))  return false;
		suspension[wl].SetToe(toe);  suspension[wr].SetToe(toe);

		if (!c.GetParamE("suspension-"+pos+".anti-roll", anti_roll))  return false;
		suspension[wl].SetAntiRollK(anti_roll);  suspension[wr].SetAntiRollK(anti_roll);

		// Hinge
		float hinge[3];  MATHVECTOR<Dbl,3> vec;

		if (!c.GetParamE("suspension-"+possh+"L.hinge", hinge))  return false;
		// Cap hinge to reasonable values
		for (int i = 0; i < 3; ++i)
		{
			if (hinge[i] < -100)	hinge[i] = -100;
			if (hinge[i] > 100)		hinge[i] = 100;
		}
		if (version == 2)  convertV2to1(hinge[0],hinge[1],hinge[2]);
		vec.Set(hinge[0],hinge[1], hinge[2]);
		suspension[wl].SetHinge(vec);

		if (!c.GetParamE("suspension-"+possh+"R.hinge", hinge))  return false;
		for (int i = 0; i < 3; ++i)
		{
			if (hinge[i] < -100)	hinge[i] = -100;
			if (hinge[i] > 100)		hinge[i] = 100;
		}
		if (version == 2)  convertV2to1(hinge[0],hinge[1],hinge[2]);
		vec.Set(hinge[0],hinge[1], hinge[2]);
		suspension[wr].SetHinge(vec);
	}

	// Load the wheels
	for (int i = 0; i < numWheels; ++i) {
		std::string sPos = sCfgWh[i];
		WHEEL_POSITION wp = WHEEL_POSITION(i);

		float roll_h, mass;
		float pos[3];  MATHVECTOR<Dbl,3> vec;

		if (!c.GetParamE("wheel-"+sPos+".mass", mass))  return false;
		wheel[wp].SetMass(mass);

		if (!c.GetParamE("wheel-"+sPos+".roll-height", roll_h))  return false;
		wheel[wp].SetRollHeight(roll_h);

		if (!c.GetParamE("wheel-"+sPos+".position", pos))  return false;
		if (version == 2)  convertV2to1(pos[0],pos[1],pos[2]);
		vec.Set(pos[0],pos[1], pos[2]);
		wheel[wp].SetExtendedPosition(vec);

		addMassParticle(mass, vec);
	}

	// Load the rotational inertia parameter from the tire section
	float front,rear;
	if (c.GetParamE("tire-both.rotational-inertia", front))
		rear = front;
	else {
		if (!c.GetParamE("tire-front.rotational-inertia", front))  return false;
		if (!c.GetParamE("tire-rear.rotational-inertia", rear))  return false;
	}
	wheel[FRONT_LEFT].SetInertia(front);
	wheel[FRONT_RIGHT].SetInertia(front);

	for (int i=REAR_LEFT; i <= REAR3_RIGHT; ++i)
		if (i < numWheels)	wheel[i].SetInertia(rear);

	// Load the tires
	float val;
	bool both = c.GetParam("tire-both.radius", val);

	ii = std::max(2, numWheels/2);
	for (int i = 0; i < ii; ++i) {
		WHEEL_POSITION wl, wr;  std::string pos;
		getWheelPosStr(i, numWheels, wl, wr, pos);
		if (both)  pos = "both";

		float rolling_resistance[3];
		if (!c.GetParamE("tire-"+pos+".rolling-resistance", rolling_resistance))  return false;
		wheel[wl].SetRollingResistance(rolling_resistance[0], rolling_resistance[1]);
		wheel[wr].SetRollingResistance(rolling_resistance[0], rolling_resistance[1]);

		float radius;
		if (!c.GetParamE("tire-"+pos+".radius", radius))  return false;
		wheel[wl].SetRadius(radius);
		wheel[wr].SetRadius(radius);
	}

	// Load the mass-only particles

	if (c.GetParam("contact-points.mass", mass)) {
		int paramnum(0);
		std::string paramname("contact-points.position-00");
		std::stringstream output_supression;
		while (c.GetParam(paramname, pos))
		{
			if (version == 2)  convertV2to1(pos[0],pos[1],pos[2]);
			position.Set(pos[0],pos[1],pos[2]);
			addMassParticle(mass, position);
			paramnum++;
			std::stringstream str;
			str << "contact-points.position-";  str.width(2);  str.fill('0');
			str << paramnum;
			paramname = str.str();
		}
	}

	std::string paramname = "particle-00";
	int paramnum = 0;
	while (c.GetParam(paramname+".mass", mass))
	{
		if (!c.GetParamE(paramname+".position", pos))  return false;
		if (version == 2)  convertV2to1(pos[0],pos[1],pos[2]);
		position.Set(pos[0],pos[1],pos[2]);
		addMassParticle(mass, position);
		paramnum++;
		std::stringstream str;
		str << "particle-";  str.width(2);  str.fill('0');
		str << paramnum;
		paramname = str.str();
	}

	// Load max steering angle
	float maxangle = 26.f;
	if (!c.GetParamE("steering.max-angle", maxangle))  return false;
	setMaxSteeringAngle( maxangle );

	a = 1.f;
	c.GetParam("steering.flip-pow-mul", a);	 flipMul = a;

	// Load the angular damping
	a = 0.4f;
	c.GetParamE("steering.angular-damping", a);
	setAngDamp(a);

	a=0.f;  c.GetParam("rot_drag.roll", a);  rotCoeff[0] = a;
	a=0.f;  c.GetParam("rot_drag.pitch", a); rotCoeff[1] = a;
	a=0.f;  c.GetParam("rot_drag.yaw", a);	 rotCoeff[2] = a;
	a=0.f;  c.GetParam("rot_drag.yaw2", a);	 rotCoeff[3] = a;

	// Load the driver
	if (!c.GetParamE("driver.mass", mass))  return false;
	if (!c.GetParamE("driver.position", pos))  return false;
	if (version == 2)  convertV2to1(pos[0],pos[1],pos[2]);
	position.Set(pos[0], pos[1], pos[2]);
	addMassParticle(mass, position);

	// Load the aerodynamics
	float drag_area, drag_c, lift_area, lift_c, lift_eff;

	if (!c.GetParamE("drag.frontal-area", drag_area))  return false;
	if (!c.GetParamE("drag.drag-coefficient", drag_c))  return false;
	if (!c.GetParamE("drag.position", pos))  return false;
	if (version == 2)  convertV2to1(pos[0],pos[1],pos[2]);
	position.Set(pos[0], pos[1], pos[2]);
	addAerodynamicDevice(position, drag_area, drag_c, 0,0,0);

	for (int i = 0; i < 2; ++i) {
		std::string wingpos = i==1 ? "rear" : "front";
		if (!c.GetParamE("wing-"+wingpos+".frontal-area", drag_area))  return false;
		if (!c.GetParamE("wing-"+wingpos+".drag-coefficient", drag_c))  return false;
		if (!c.GetParamE("wing-"+wingpos+".surface-area", lift_area))  return false;
		if (!c.GetParamE("wing-"+wingpos+".lift-coefficient", lift_c))  return false;
		if (!c.GetParamE("wing-"+wingpos+".efficiency", lift_eff))  return false;
		if (!c.GetParamE("wing-"+wingpos+".position", pos))  return false;
		if (version == 2)  convertV2to1(pos[0],pos[1],pos[2]);
		position.Set(pos[0], pos[1], pos[2]);
		addAerodynamicDevice(position, drag_area, drag_c, lift_area, lift_c, lift_eff);
	}

	//TODO Hover params?.. Probably not

	updateMass();
}

void CarDynamics::init(const MATHVECTOR<Dbl, 3> position, const QUATERNION<Dbl> orientation) {
	//TODO Receive Settings(?), Scene, FluidsXml, COLLISION_WORLD
	MATHVECTOR<Dbl, 3> zero(0);

	body.SetPosition(position); body.SetOrientation(orientation);
	body.SetInitialForce(zero); body.SetInitialTorque(zero);
	// camBody.SetPosition(zero); camBody.setInitialForce(zero);

	engine.SetInitialConditions();

	btTransform tr; tr.setIdentity();
	AABB<float> box;
	for (int i = 0; i < numWheels; i++) {
		MATHVECTOR<float, 3> wheelPos = getLocalWheelPosition(WHEEL_POSITION(i), 0);
		AABB<float> wheelAABB;
		wheelAABB.SetFromCorners(wheelPos, wheelPos);
		box.CombineWith(wheelAABB);
	}

	// Chassis shape
	const MATHVECTOR<Dbl,3> verticalMargin(0, 0, 0.3);
	btVector3 origin = ToBulletVector(box.GetCenter() + verticalMargin - centerOfMass);
	btVector3 size = ToBulletVector(box.GetSize() - verticalMargin);

	btCollisionShape* chassisShape;
	// y| length  x- width  z^ height
	btScalar w = size.getX()*0.2, r = size.getZ()*0.3, h = 0.45;

	///  spheres
	btScalar l0 = 0.f, w0 = 0.f, h0 = 0.f;
	if (collR > 0.f)  r = collR;  l0 = collLofs;
	if (collW > 0.f)  w = collW;  w0 = collWofs;
	if (collH > 0.f)  h = collH;	h0 = collHofs;
	origin = btVector3(l0, w0, h0);

	const int numSph = 14;  int i = 0;
	btScalar rad[numSph];  btVector3 pos[numSph];

	btScalar r2 = r * collR2m;
	btScalar l1 = collPosLFront, l2 = collPosLBack, l1m = l1*0.5, l2m = l2*0.5;
	float ww = 1.f;
	float wt = collTopWMul * ww;

	rad[i] = r2;  pos[i] = btVector3( l1 , -w*ww, -h*collFrHMul);  ++i;  // front
	rad[i] = r2;  pos[i] = btVector3( l1 ,  w*ww, -h*collFrHMul);  ++i;
	rad[i] = r;   pos[i] = btVector3( l1m, -w*ww, -h*collFrHMul);  ++i;  // front near
	rad[i] = r;   pos[i] = btVector3( l1m,  w*ww, -h*collFrHMul);  ++i;

	rad[i] = r;   pos[i] = btVector3( l2m, -w,    -h);  ++i;  // rear near
	rad[i] = r;   pos[i] = btVector3( l2m,  w,    -h);  ++i;
	rad[i] = r2;  pos[i] = btVector3( l2 , -w,    -h);  ++i;  // rear
	rad[i] = r2;  pos[i] = btVector3( l2 ,  w,    -h);  ++i;

	rad[i] = r2;  pos[i] = btVector3( collTopFr,  -w*wt, h*collTopFrHm  );  ++i;  // top
	rad[i] = r2;  pos[i] = btVector3( collTopFr,   w*wt, h*collTopFrHm  );  ++i;
	rad[i] = r2;  pos[i] = btVector3( collTopMid, -w*wt, h*collTopMidHm );  ++i;
	rad[i] = r2;  pos[i] = btVector3( collTopMid,  w*wt, h*collTopMidHm );  ++i;
	rad[i] = r2;  pos[i] = btVector3( collTopBack,-w*wt, h*collTopBackHm);  ++i;  // top rear
	rad[i] = r2;  pos[i] = btVector3( collTopBack, w*wt, h*collTopBackHm);  ++i;

	for (i=0; i < numSph; ++i)
		pos[i] += origin;
	chassisShape = new btMultiSphereShape(pos, rad, numSph);
	chassisShape->setMargin(0.02f);


	Dbl chassisMass = body.GetMass();
	MATRIX3 <Dbl> inertia = body.GetInertia();
	btVector3 chassisInertia(inertia[0], inertia[4], inertia[8]);

	btTransform transform;
	transform.setOrigin(ToBulletVector(position));
	transform.setRotation(ToBulletQuaternion(orientation));
	btDefaultMotionState * chassisState = new btDefaultMotionState();
	chassisState->setWorldTransform(transform);

	btRigidBody::btRigidBodyConstructionInfo info(chassisMass, chassisState, chassisShape, chassisInertia);
	info.m_angularDamping = angDamp;
	info.m_restitution = 0.0;  //...
	info.m_friction = collFriction;  /// 0.4~ 0.7
	shapes.push_back(chassisShape);

	//TODO Add COLLISION_WORLD and ShapeData; uncomment when implemented
//	chassis = world.AddRigidBody(info, true, pSet->game.collis_cars);
	rigids.push_back(chassis);
	chassis->setActivationState(DISABLE_DEACTIVATION);
//	chassis->setUserPointer(new ShapeData(ST_Car, this, 0));
//	world.world->addAction(this);
	actions.push_back(this);


	// Join chassis and wheel triggers
	{
		for (int w=0; w < numWheels; ++w) {
			WHEEL_POSITION wp = WHEEL_POSITION(w);
			Dbl whR = getWheel(wp).GetRadius() * 1.2;  //par bigger
			MATHVECTOR<float,3> wheelpos = getWheelPosition(wp, 0);  //par
			wheelpos[0] += collLofs;
			wheelpos[2] += collFlTrigH;

			btSphereShape* whSph = new btSphereShape(whR);
			whTrigs = new btRigidBody(0.001f, 0, whSph);

			// Uncomment when ready
//			whTrigs->setUserPointer(new ShapeData(ST_Wheel, this, 0, w));
			whTrigs->setActivationState(DISABLE_DEACTIVATION);
			whTrigs->setCollisionFlags(whTrigs->getCollisionFlags() | btCollisionObject::CF_NO_CONTACT_RESPONSE);

//			world.world->addRigidBody(whTrigs);  rigids.push_back(whTrigs);
//			world.shapes.push_back(whSph);  shapes.push_back(whSph);

			btTypedConstraint* constr = new btPoint2PointConstraint(*chassis, *whTrigs,
				ToBulletVector(wheelpos), btVector3(0,0,0));

//			world.world->addConstraint(constr, true);
			constraints.push_back(constr);
		}

		// Init poly for buoyancy computations
		if (poly == NULL) {
			poly = new Polyhedron();
			poly->numVerts = 8;  poly->numFaces = 12;
			poly->verts = new Vec3[8];
			poly->faces = new Face[12];

			float hx = 1.2f, hy = 0.7f, hz = 0.4f;  // box dim
			poly->verts[0] = Vec3(-hx,-hy,-hz);	poly->verts[1] = Vec3(-hx,-hy, hz);
			poly->verts[2] = Vec3(-hx, hy,-hz);	poly->verts[3] = Vec3(-hx, hy, hz);
			poly->verts[4] = Vec3( hx,-hy,-hz);	poly->verts[5] = Vec3( hx,-hy, hz);
			poly->verts[6] = Vec3( hx, hy,-hz);	poly->verts[7] = Vec3( hx, hy, hz);

			poly->faces[0] = Face(0,1,3);	poly->faces[1] = Face(0,3,2);	poly->faces[2] = Face(6,3,7);	poly->faces[3] = Face(6,2,3);
			poly->faces[4] = Face(4,6,5);	poly->faces[5] = Face(6,7,5);	poly->faces[6] = Face(4,5,0);	poly->faces[7] = Face(0,5,1);
			poly->faces[8] = Face(5,7,1);	poly->faces[9] = Face(7,3,1);	poly->faces[10]= Face(0,6,4);	poly->faces[11]= Face(0,2,6);

			poly->length = 1.0f;  //  approx. length-?
			poly->volume = ComputeVolume(*poly);

			bodyMass = 1900.0f * 2.688;  //poly->volume;  // car density
			bodyInertia = (4.0f * bodyMass / 12.0f) * btVector3(hy*hz, hx*hz, hx*hy);
		}
	}
	//-------------------------------------------------------------


	// Init wheels, suspension
	for (int i = 0; i < numWheels; ++i) {
		wheel[WHEEL_POSITION(i)].SetInitialConditions();
		wheelVelocity[i].Set(0.0);
		wheelPosition[i] = getWheelPositionAtDisplacement(WHEEL_POSITION(i), 0);
		wheelOrientation[i] = orientation * getWheelSteeringAndSuspensionOrientation(WHEEL_POSITION(i));
	}

	alignWithGround();//--
}

void CarDynamics::setDrive(const std::string& newDrive) {
	if (newDrive == "RWD")
		drive = RWD;
	else if (newDrive == "FWD")
		drive = FWD;
	else if (newDrive == "AWD")
		drive = AWD;
	else
		assert(false);
}

void CarDynamics::addMassParticle(Dbl newMass, MATHVECTOR<Dbl, 3>& newPos) {
	newPos[0] += comOfsL;
	newPos[2] += comOfsH;
	massOnlyParticle.push_back(std::pair<Dbl, MATHVECTOR<Dbl, 3> >(newMass, newPos));
}

void CarDynamics::addAerodynamicDevice(const MATHVECTOR<Dbl, 3>& newPos, Dbl dragFrontArea, Dbl dragCoeff,
									   Dbl liftSurfArea, Dbl liftCoeff, Dbl liftEff) {
	aerodynamics.push_back(CARAERO());
	aerodynamics.back().Set(newPos, dragFrontArea, dragCoeff, liftSurfArea, liftCoeff, liftEff);
}

void CarDynamics::updateAction(btCollisionWorld* collisionWorld, btScalar dt) {

}

void CarDynamics::update() {
	if (!chassis) return;

	btTransform tr;
	chassis->getMotionState()->getWorldTransform(tr);
	chassisRotation = ToMathQuaternion<Dbl>(tr.getRotation());
	chassisCenterOfMass = ToMathVector<Dbl>(tr.getOrigin());

	MATHVECTOR<Dbl,3> com = centerOfMass;
	chassisRotation.RotateVector(com);
	chassisPosition = chassisCenterOfMass - com;

	updateBuoyancy();
}

void CarDynamics::updateBuoyancy() {
	//TODO Add fluid classes...
//	if (!pScene || (pScene->fluids.size() == 0) || !poly || !pFluids)  return;
//
//	//float bc = /*sinf(chassisPosition[0]*20.3f)*cosf(chassisPosition[1]*30.4f) +*/
//	//	sinf(chassisPosition[0]*0.3f)*cosf(chassisPosition[1]*0.32f);
//	//LogO("pos " + toStr((float)chassisPosition[0]) + " " + toStr((float)chassisPosition[1]) + "  b " + toStr(bc));
//
//	for (std::list<FluidBox*>::const_iterator i = inFluids.begin();
//		i != inFluids.end(); ++i)  // 0 or 1 is there
//	{
//		const FluidBox* fb = *i;
//		if (fb->id >= 0)
//		{
//			const FluidParams& fp = pFluids->fls[fb->id];
//
//			WaterVolume water;
//			//float bump = 1.f + 0.7f * sinf(chassisPosition[0]*fp.bumpFqX)*cosf(chassisPosition[1]*fp.bumpFqY);
//			water.density = fp.density /* (1.f + 0.7f * bc)*/;  water.angularDrag = fp.angularDrag;
//			water.linearDrag = fp.linearDrag;  water.linearDrag2 = 0.f;  //1.4f;//fp.linearDrag2;
//			water.velocity.SetZero();
//			water.plane.offset = fb->pos.y;  water.plane.normal = Vec3(0,0,1);
//			//todo: fluid boxes rotation yaw, pitch ?-
//
//			RigidBody body;  body.mass = body_mass;
//			body.inertia = Vec3(body_inertia.getX(),body_inertia.getY(),body_inertia.getZ());
//
//			///  body initial conditions
//			//  pos & rot
//			body.x.x = chassisPosition[0];  body.x.y = chassisPosition[1];  body.x.z = chassisPosition[2];
//			if (vtype == V_Sphere)
//			{	body.q.x = 0.f;  body.q.y = 0.f;  body.q.z = 0.f;  body.q.w = 1.f;  // no rot
//			}else
//			{	body.q.x = chassisRotation[0];  body.q.y = chassisRotation[1];  body.q.z = chassisRotation[2];  body.q.w = chassisRotation[3];
//				body.q.Normalize();//
//			}
//			//LogO(fToStr(body.q.x,2,4)+" "+fToStr(body.q.y,2,4)+" "+fToStr(body.q.z,2,4)+" "+fToStr(body.q.w,2,4));
//			//  vel, ang vel
//			btVector3 v = chassis->getLinearVelocity();
//			btVector3 a = chassis->getAngularVelocity();
//			body.v.x = v.getX();  body.v.y = v.getY();  body.v.z = v.getZ();
//			body.omega.x = a.getX();  body.omega.y = a.getY();  body.omega.z = a.getZ();
//			body.F.SetZero();  body.T.SetZero();
//
//			//  damp from height vel
//			body.F.z += fp.heightVelRes * -1000.f * body.v.z;
//
//			///  add buoyancy force
//			if (ComputeBuoyancy(body, *poly, water, 9.8f))
//			{
//				if (vtype != V_Car)
//				{	body.F.x *= 0.15f;  body.F.y *= 0.15f;  }
//				chassis->applyCentralForce( btVector3(body.F.x,body.F.y,body.F.z) );
//				chassis->applyTorque(       btVector3(body.T.x,body.T.y,body.T.z) );
//			}
//		}
//	}
//
//	///  wheel spin force (for mud)
//	//_______________________________________________________
//	for (int w=0; w < numWheels; ++w)
//	{
//		if (inFluidsWh[w].size() > 0)  // 0 or 1 is there
//		{
//			MATHVECTOR<Dbl,3> up(0,0,1);
//			Orientation().RotateVector(up);
//			float upZ = std::max(0.f, (float)up[2]);
//
//			const FluidBox* fb = *inFluidsWh[w].begin();
//			if (fb->id >= 0)
//			{
//				const FluidParams& fp = pFluids->fls[fb->id];
//
//				WHEEL_POSITION wp = WHEEL_POSITION(w);
//				float whR = GetWheel(wp).GetRadius() * 1.2f;  //bigger par
//				MATHVECTOR<float,3> wheelpos = GetWheelPosition(wp, 0);
//				wheelpos[2] -= whR;
//				whP[w] = fp.idParticles;
//				whDmg[w] = fp.fDamage;
//
//				//  height in fluid:  0 just touching surface, 1 fully in fluid
//				//  wheel plane distance  water.plane.normal.z = 1  water.plane.offset = fl.pos.y;
//				whH[w] = (wheelpos[2] - fb->pos.y) * -0.5f / whR;
//				whH[w] = std::max(0.f, std::min(1.f, whH[w]));
//
//				if (fp.bWhForce)
//				{
//					//bool inAir = GetWheelContact(wp).col == NULL;
//
//					//  bump, adds some noise
//					MATHVECTOR<Dbl,3> whPos = GetWheelPosition(wp) - chassisPosition;
//					float bump = sinf(whPos[0]*fp.bumpFqX)*cosf(whPos[1]*fp.bumpFqY);
//
//					float f = std::min(fp.whMaxAngVel, std::max(-fp.whMaxAngVel, (float)wheel[w].GetAngularVelocity() ));
//					QUATERNION<Dbl> steer;
//					float ba = numWheels==2 && w==0 ? 2.f : 1.f;  //bike
//					float angle = -wheel[wp].GetSteerAngle() * fp.whSteerMul * ba  + bump * fp.bumpAng;
//					steer.Rotate(angle * PI_d/180.f, 0, 0, 1);
//
//					//  forwards, side, up
//					MATHVECTOR<Dbl,3> force(whH[w] * fp.whForceLong * f, 0, /*^ 0*/100.f * whH[w] * fp.whForceUp * upZ);
//					(Orientation()*steer).RotateVector(force);
//
//					//  wheel spin resistance
//					wheel[w].fluidRes = whH[w] * fp.whSpinDamp  * (1.f + bump * fp.bumpAmp);
//
//					if (whH[w] > 0.01f /*&& inAir*/)
//						chassis->applyForce( ToBulletVector(force), ToBulletVector(whPos) );
//				}
//			}
//		}
//		else
//		{	whH[w] = 0.f;  wheel[w].fluidRes = 0.f;  whP[w] = -1;	}
//	}
}

MATHVECTOR<Dbl, 3> CarDynamics::getWheelPosition(WHEEL_POSITION wp) const {
	return getWheelPosition(wp, suspension[wp].GetDisplacementPercent());
}

MATHVECTOR<Dbl, 3> CarDynamics::getWheelPosition(WHEEL_POSITION wp, Dbl displacementPercent) const {
	MATHVECTOR<Dbl, 3> pos = getLocalWheelPosition(wp, displacementPercent);
	chassisRotation.RotateVector(pos);
	return pos + chassisPosition;
}

void CarDynamics::alignWithGround() {
	updateWheelTransforms();
	updateWheelContacts();
}

MATHVECTOR<Dbl, 3> CarDynamics::getDownVector() const {
	MATHVECTOR<Dbl, 3> v(0, 0, -1);
	body.GetOrientation().RotateVector(v);
	return v;
}

QUATERNION<Dbl> CarDynamics::getWheelSteeringAndSuspensionOrientation(WHEEL_POSITION wp) const {
	QUATERNION<Dbl> steer;
	steer.Rotate( -wheel[wp].GetSteerAngle() * PI_d/180.0, 0,0,1);

	QUATERNION<Dbl> camber;
	Dbl camber_rotation = -suspension[wp].GetCamber() * PI_d/180.0;
	if (wp%2 == 1)
		camber_rotation = -camber_rotation;
	camber.Rotate( camber_rotation, 1,0,0);

	QUATERNION<Dbl> toe;
	Dbl toe_rotation = suspension[wp].GetToe() * PI_d/180.0;
	if (wp%2 == 0)
		toe_rotation = -toe_rotation;
	toe.Rotate( toe_rotation, 0,0,1);

	return camber * toe * steer;
}

MATHVECTOR<Dbl, 3> CarDynamics::getWheelPositionAtDisplacement(WHEEL_POSITION wp, Dbl displacementPercent) const {
	return localToWorld(getLocalWheelPosition(wp, displacementPercent));
}

void CarDynamics::updateWheelTransforms() {
	for (int i = 0; i < numWheels; ++i) {
		wheelPosition[i] = getWheelPositionAtDisplacement(WHEEL_POSITION(i), suspension[i].GetDisplacementPercent());
		wheelOrientation[i] = body.GetOrientation() * getWheelSteeringAndSuspensionOrientation(WHEEL_POSITION(i));
	}
}

void CarDynamics::updateWheelContacts() {
	MATHVECTOR<float,3> raydir = getDownVector();
	for (int i = 0; i < numWheels; ++i)
	{
		COLLISION_CONTACT & wc = wheelContact[WHEEL_POSITION(i)];
		MATHVECTOR<float,3> raystart = localToWorld(wheel[i].GetExtendedPosition());
		raystart = raystart - raydir * wheel[i].GetRadius();  //*!
		float raylen = 1.5f;  // !par

		//TODO COLLISION_WORLD!
//		world->CastRay( raystart, raydir, raylen, chassis, wc, this,i, !pSet->game.collis_cars, false );
	}
}

void CarDynamics::updateMass() {
	typedef std::pair<Dbl, MATHVECTOR<Dbl, 3> > MASS_PAIR;

	Dbl totalMass(0);
	centerOfMass.Set(0,0,0);

	// Calculate the total mass, and center of mass
	for (std::list <MASS_PAIR>::iterator i = massOnlyParticle.begin(); i != massOnlyParticle.end(); ++i) {
		// add the current mass to the total mass
		totalMass += i->first;

		// incorporate the current mass into the center of mass
		centerOfMass = centerOfMass + i->second * i->first;
	}

	// Account for fuel
	totalMass += fuelTank.GetMass();
	centerOfMass = centerOfMass + fuelTank.GetPosition() * fuelTank.GetMass();

	body.SetMass(totalMass);
//	cam_body.SetMass(total_mass * gPar.camBncMass); //TODO Use camera?
	fBncMass = 1350.0 / totalMass;

	centerOfMass = centerOfMass * (1.0 / totalMass);

	// Calculate the inertia tensor
	MATRIX3 <Dbl> inertia;
	for (int i = 0; i < 9; ++i)
		inertia[i] = 0;

	for (std::list <MASS_PAIR>::iterator i = massOnlyParticle.begin(); i != massOnlyParticle.end(); ++i) {
		// Transform into the rigid body coordinates
		MATHVECTOR<Dbl,3> pos = i->second - centerOfMass;
		Dbl mass = i->first;

		// Add the current mass to the inertia tensor
		inertia[0] += mass * ( pos[1] * pos[1] + pos[2] * pos[2] );
		inertia[1] -= mass * ( pos[0] * pos[1] );
		inertia[2] -= mass * ( pos[0] * pos[2] );
		inertia[3] = inertia[1];
		inertia[4] += mass * ( pos[2] * pos[2] + pos[0] * pos[0] );
		inertia[5] -= mass * ( pos[1] * pos[2] );
		inertia[6] = inertia[2];
		inertia[7] = inertia[5];
		inertia[8] += mass * ( pos[0] * pos[0] + pos[1] * pos[1] );
	}

	body.SetInertia( inertia );
}

MATHVECTOR<Dbl, 3> CarDynamics::localToWorld(const MATHVECTOR<Dbl, 3>& local) const {
	MATHVECTOR<Dbl, 3> position = local - centerOfMass;
	body.GetOrientation().RotateVector(position);
	return position + body.GetPosition();
}

MATHVECTOR<Dbl, 3> CarDynamics::getLocalWheelPosition(WHEEL_POSITION wp, Dbl displacementPercent) const {
	const MATHVECTOR<Dbl,3> & wheelext = wheel[wp].GetExtendedPosition();
	const MATHVECTOR<Dbl,3> & hinge = suspension[wp].GetHinge();
	MATHVECTOR<Dbl,3> relwheelext = wheelext - hinge;
	MATHVECTOR<Dbl,3> up(0,0,1);
	MATHVECTOR<Dbl,3> rotaxis = up.cross ( relwheelext.Normalize() );
	Dbl hingeradius = relwheelext.Magnitude();
	Dbl travel = suspension[wp].GetTravel();

	Dbl displacement = displacementPercent * travel;
	Dbl displacementradians = displacement / hingeradius;
	QUATERNION<Dbl> hingerotate;
	hingerotate.Rotate ( -displacementradians, rotaxis[0], rotaxis[1], rotaxis[2] );
	MATHVECTOR<Dbl,3> localwheelpos = relwheelext;
	hingerotate.RotateVector ( localwheelpos );

	return localwheelpos + hinge;
}
