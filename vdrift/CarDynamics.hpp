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
#include "CarConstants.hpp"

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
	~CarDynamics();

	// Initialization
	bool load(ConfigFile& cf);
	void init(MathVector<double, 3> pos, Quaternion<double> rot, CollisionWorld& world);

	void alignWithGround();

	// Bullet interface
	virtual void updateAction(btCollisionWorld* collisionWorld, btScalar deltaTimeStep); //FIXME
	virtual void debugDraw(btIDebugDraw* debugDrawer) { }

	void removeBullet();

	// Graphics interface
	void update();

	// Driveline input
	void shiftGear(int value);

	// Speedometer output based on driveshaft RPM
	double getSpeedMPS() const;

	// Chassis state access
	const Quaternion<double>& getOrientation() const { return chassisRotation; }
	MathVector<double, 3> getVelocity() const { return body.getVelocity(); }
	double getSpeed() const { return body.getVelocity().magnitude(); }

	// Wheel state access
	const CarWheel& getWheel(WheelPosition wp) const { return wheels[wp]; }

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
	double driveshaftRPM;

	bool autoclutch, autoshift, autorear, shifted;
	double shiftTime, remShiftTime, lastAutoClutch;
	int gearToShift; // Analog to shift_gear

	// Wheel state
	int numWheels;
	std::vector<CarWheel> wheels;
	std::vector<CarBrake> brakes;
	std::vector<CarSuspension> suspension;

	std::vector<MathVector<double, 3> > wheelVels, wheelPos;
	std::vector<Quaternion<double> > wheelRots;

	MathVector<double, 3> localToWorld(const MathVector<double, 3>& local) const;

	MathVector<double, 3> getWheelPosition(WheelPosition wp, double displacementPercent) const; // For internal use
	MathVector<double, 3> getLocalWheelPosition(WheelPosition wp, double displacementPercent) const;
	MathVector<double, 3> getWheelPositionAtDisplacement(WheelPosition wp, double displacementPercent) const;
	Quaternion<double> getWheelSteeringAndSuspensionOrientation(WheelPosition wp) const;

	bool wheelDriven(int i) const { return (1 << i) & drive; } // Sorry for the magic function

	// Updater methods
	void synchronizeBody();
	void updateWheelContacts();
	void tick(double dt); // Update simulation
	void synchronizeChassis();

	void updateBody(double dt, double driveTorque[]); // Advance chassis (body, suspension, wheels) sim by dt

	void updateTransmission(double dt);
	void updateDriveline(double dt, double driveTorque[]); // Update engine, return wheel drive torque
	double calculateDriveshaftRPM() const; // Calculate clutch driveshaft RPM
	double calculateDriveshaftSpeed(); // Calculate driveshaft speed given wheel angular velocity

	void applyClutchTorque(double engineDrag, double clutchSpeed); // Apply clutch torque to engine
	void calculateDriveTorque(double driveTorque[], double clutchTorque); // Calculate wheel drive torque

	int nextGear() const; // Calculate next gear based on engine RPM
	double downshiftRPM(int gear, float avgWhHeight = 0.f) const; // Calculate downshift point based on gear and engine RPM

	double autoClutch(double lastClutch, double dt) const;
	double shiftAutoClutch() const;
	double shiftAutoClutchThrottle(double throttle, double dt);

	void updateWheelTransform();
	void updateWheelVelocity();

	void interpolateWheelContacts(double dt);

	void applyEngineTorqueToBody();
	void applyAerodynamicsToBody();

	void applyForce(const MathVector<double, 3>& force) { body.applyForce(force); } //TODO Skip camera body
	void applyTorque(const MathVector<double, 3>& torque) { body.applyTorque(torque); }

	// Aerodynamics
	std::vector<CarAero> aerodynamics;
	double rotCoeff[4];

	// Steering
	double maxAngle;
	double angularDamping;

	// Chassis state
	btRigidBody* chassis;
	btRigidBody* whTrigs;
	RigidBody body;
	MathVector<double, 3> centerOfMass;
	std::list<std::pair<double, MathVector<double, 3> > > massOnlyParticles;

	Quaternion<double> getBodyOrientation() const { return body.getOrientation(); }
	MathVector<double, 3> getDownVector() const;

	// Interpolated chassis state, using Bullet sim
	MathVector<double, 3> chassisPosition;
	Quaternion<double> chassisRotation;

	CollisionWorld* world;

	// Bullet objects for deletion
	btAlignedObjectArray<btCollisionShape*> shapes;
	btAlignedObjectArray<btRigidBody*> rigids;
	btAlignedObjectArray<btActionInterface*> actions;
	btAlignedObjectArray<btTypedConstraint*> constraints;
};
