#pragma once

#include "carsuspension.h"
#include "carwheel.h"
#include "collision_contact.h"
#include "carbrake.h"
#include "configfile.h"
#include "carengine.h"
#include "carclutch.h"
#include "cartransmission.h"
#include "carfueltank.h"
#include "cardifferential.h"
#include "caraero.h"
#include "rigidbody.h"
#include "cardefs.h"
#include "aabb.h"
#include "tobullet.h"
#include "Buoyancy.h"

#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>

#include <vector>

// Analog to Stuntrally's CARDYNAMICS class (in vdrift/cardynamics.h)
class CarDynamics : public btActionInterface {
public:
	//TODO May need references to Sim
	CarDynamics();
	~CarDynamics();

	VehicleType vtype; // Should be V_CAR in this application (no spaceships allowed!)

	// Copied from Car class
	int numWheels;
	void setNumWheels(int n);

	// Load from .car file; return true on success
	bool load(CONFIGFILE& carConf);
	static void convertV2to1(float& x, float& y, float& z); // v2 .car files store arrays in different order..
	static void getWheelPosStr(int i, int numWheels, WHEEL_POSITION& wl, WHEEL_POSITION& wr, std::string& pos);

	// Init after loading
	void init(const MATHVECTOR<Dbl, 3> position, const QUATERNION<Dbl> orientation);

	// CarDynamics initialization?
	void setDrive(const std::string& newDrive);
	void addMassParticle(Dbl newMass, MATHVECTOR<Dbl, 3>& newPos);
	void setMaxSteeringAngle(Dbl newAngle) { maxAngle = newAngle; }
	void setAngDamp(Dbl newDamp) { angDamp = newDamp; }
	void addAerodynamicDevice(const MATHVECTOR<Dbl, 3>& newPos, Dbl dragFrontArea, Dbl dragCoeff,
							  Dbl liftSurfArea, Dbl liftCoeff, Dbl liftEff);

	// Graphics interface (interpolated)
	void update();
	void updateBuoyancy();

	MATHVECTOR<Dbl, 3> getWheelPosition(WHEEL_POSITION wp) const;
	MATHVECTOR<Dbl, 3> getWheelPosition(WHEEL_POSITION wp, Dbl displacementPercent) const;


	// Driveline state access
	const CARWHEEL& getWheel(WHEEL_POSITION pos) const { return wheel[pos]; }

	// CarDynamics
	// Move the car along the z-axis until it touches the ground
	void alignWithGround();

	// Chassis
	MATHVECTOR<Dbl, 3> getDownVector() const;

	// Orientation of wheel solely due to steering and suspension
	QUATERNION<Dbl> getWheelSteeringAndSuspensionOrientation(WHEEL_POSITION wp) const;
	// World-space position of wheel center when suspension is compressed by the displacementPercent (1.0 = full compression)
	MATHVECTOR<Dbl,3> getWheelPositionAtDisplacement(WHEEL_POSITION wp, Dbl displacementPercent) const;

	void updateWheelTransforms();
	void updateWheelContacts();

	void updateMass(); Dbl fBncMass;

	MATHVECTOR<Dbl, 3> localToWorld(const MATHVECTOR<Dbl, 3>& local) const;
	MATHVECTOR<Dbl, 3> getLocalWheelPosition(WHEEL_POSITION wp, Dbl displacementPercent) const;

	// Interpolated chassis state
	MATHVECTOR<Dbl, 3> chassisPosition, chassisCenterOfMass;
	QUATERNION<Dbl> chassisRotation;

	// CarDynamics state
	std::vector<MATHVECTOR<Dbl, 3> > wheelVelocity, wheelPosition;
	std::vector<QUATERNION<Dbl> > wheelOrientation;
	std::vector<COLLISION_CONTACT> wheelContact;

	std::vector<CARSUSPENSION> suspension;
	std::vector<CARAERO> aerodynamics;

	std::list< std::pair<Dbl, MATHVECTOR<Dbl, 3> > > massOnlyParticle;

	Dbl maxAngle, flipMul, angDamp;
	Dbl rotCoeff[4];

	// Bullet objects (must be deleted)
	btAlignedObjectArray<btCollisionShape*> shapes;
	btAlignedObjectArray<btActionInterface*> actions;
	btAlignedObjectArray<btTypedConstraint*> constraints;
	btAlignedObjectArray<btRigidBody*> rigids;

	// Chassis state
	RIGIDBODY body; btRigidBody *chassis, *whTrigs;
	MATHVECTOR<Dbl, 3> centerOfMass;

	// Driveline state
	std::vector<CARBRAKE> brake;
	std::vector<CARWHEEL> wheel;
	CARENGINE engine;
	CARCLUTCH clutch;
	CARTRANSMISSION transmission;
	CARDIFFERENTIAL diffFront, diffCenter, diffRear;
	CARFUELTANK fuelTank;

	enum { FWD = 3, RWD = 12, AWD = 15 } drive;
	float engineVolMul;

	Dbl shiftTime;

	// Traction control state
	std::vector<int> absActive, tcsActive;

	// Terrain blendmap variables
	std::vector<int> iWheelOnRoad, wheelTerrMtr, wheelRoadMtr;

	// Buoyancy
	std::vector<float> wheelSubmersion; // 0 = touching surface -> 1 = fully in fluid
	std::vector<int> wheelFluidParticle; // ID of fluid particles
	std::vector<float> wheelFluidDmg; // Fluid damage
	struct Polyhedron* poly;
	float bodyMass;
	btVector3 bodyInertia;
	//TODO std::vector<std::list<FluidBox*>> inFluidsWh;


	// Custom collision params (loaded from CAR::Load)
	float collR, collR2m, collW, collH,
		  collHofs, collWofs, collLofs,
		  collFlTrigH, comOfsH, comOfsL,
		  collPosLFront, collPosLBack, collFriction,
		  collFrWMul, collFrHMul, collTopWMul,
		  collTopFr, collTopMid, collTopBack,
		  collTopFrHm, collTopMidHm, collTopBackHm;
};
