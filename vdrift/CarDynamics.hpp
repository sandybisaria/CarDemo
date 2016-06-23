#pragma once

#include "MathVector.hpp"
#include "CarEngine.hpp"
#include "CarClutch.hpp"
#include "CarTransmission.hpp"
#include "CarDifferential.hpp"
#include "CarBrake.hpp"
#include "CarFuelTank.hpp"
#include "CarSuspension.hpp"
#include "CarWheel.hpp"
#include "CarAero.hpp"
#include "RigidBody.hpp"
#include "AABB.hpp"
#include "CollisionWorld.hpp"
class CollisionWorld;

#include "../util/ConfigFile.hpp"
#include "../util/ToBullet.hpp"

#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>

#include <sstream>
#include <cassert>
#include <vector>

class CarDynamics
	: public btActionInterface {
public:
	CarDynamics();

	// Initialization
	bool load(ConfigFile& cf);
	void init(MathVector<double, 3> pos, Quaternion<double> rot, CollisionWorld& world);

	// Bullet interface
	virtual void updateAction(btCollisionWorld* collisionWorld, btScalar deltaTimeStep) { }
	virtual void debugDraw(btIDebugDraw* debugDrawer) { }

	// Collision params
	float collR, collR2m, collW, collH,
		  collHofs, collWofs, collLofs,
		  collFlTrigH, comOfsH, comOfsL,
		  collPosLFront, collPosLBack, collFriction,
		  collFrWMul, collFrHMul, collTopWMul,
		  collTopFr, collTopMid, collTopBack,
		  collTopFrHm, collTopMidHm, collTopBackHm;

private:
	// Initialization
	void setNumWheels(int nw);
	void setDrive(const std::string& newDrive);
	void addMassParticle(double newMass, MathVector<double, 3> newPos);
	void setMaxSteeringAngle(double newAngle) { maxAngle = newAngle; }
	void setAngularDamping(double newDamping) { angularDamping = newDamping; }
	void addAerodynamicDevice(const MathVector<double, 3>& newPos, double dragFrontalArea,
							  double dragCoefficient, double liftSurfaceArea, double liftCoefficient, double liftEfficiency);
	void calculateMass(); // Substitute for CARDYNAMICS::UpdateMass()

	// Driveline state
	CarFuelTank fuelTank;
	CarEngine engine;
	CarClutch clutch;
	CarTransmission transmission;
	CarDifferential diffFront, diffRear, diffCenter;

	enum { FWD = 3, RWD = 12, AWD = 15 } drive;

	double shiftTime;

	// Wheel state
	int numWheels;
	std::vector<CarWheel> wheels;
	std::vector<CarBrake> brakes;
	std::vector<CarSuspension> suspension;

	// Aerodynamics
	std::vector<CarAero> aerodynamics;
	double rotCoeff[4];

	// Steering
	double maxAngle;
	double angularDamping;

	// Chassis state
	btRigidBody* chassis;
	RigidBody body;
	MathVector<double, 3> centerOfMass;
	std::list<std::pair<double, MathVector<double, 3> > > massOnlyParticles;

	CollisionWorld* world;

	// Bullet objects for deletion
	btAlignedObjectArray<btCollisionShape*> shapes;
	btAlignedObjectArray<btRigidBody*> rigids;
	btAlignedObjectArray<btActionInterface*> actions;
};
