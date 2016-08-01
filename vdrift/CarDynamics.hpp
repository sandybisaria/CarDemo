#pragma once

#include "AABB.hpp"
#include "CarAero.hpp"
#include "CarBrake.hpp"
#include "CarClutch.hpp"
#include "CarConstants.hpp"
#include "CarDifferential.hpp"
#include "CarEngine.hpp"
#include "CarFuelTank.hpp"
#include "CarSuspension.hpp"
#include "CarTransmission.hpp"
#include "CarWheel.hpp"
#include "CollisionContact.hpp"
#include "CollisionWorld.hpp"
#include "MathVector.hpp"
#include "RigidBody.hpp"

#include "../util/ConfigFile.hpp"
#include "../util/ToBullet.hpp"

#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>

#include <cassert>
#include <sstream>
#include <vector>

#ifdef COMPILE_UNIT_TESTS
#include <gtest/gtest.h>
#endif

// Stuntrally's CARDYNAMICS class; by far the hardest to re-implement.
// There is no car damage in this simulation, so some methods were altered
class CarDynamics : public btActionInterface {
public:
	CarDynamics();
	~CarDynamics();

//---- Initialization
	bool load(ConfigFile& cf);
	void init(MathVector<double, 3> pos, Quaternion<double> rot, CollisionWorld& world);

	void alignWithGround();

//---- Bullet interface
	virtual void updateAction(btCollisionWorld* collisionWorld, btScalar deltaTimeStep);
	virtual void debugDraw(btIDebugDraw* debugDrawer) { }

	void removeBullet();

//---- CollisionWorld interface
	const CollisionContact& getWheelContact(WheelPosition(wp)) const {
		return wheelContact[wp];
	}
	CollisionContact& getWheelContact(WheelPosition(wp)) {
		return wheelContact[wp];
	}

//---- Update method
	void update();

//---- Driveline input
	void shiftGear(int value);
	void setThrottle(double value) { engine.setThrottle(value); }
	void setClutch(double value) { clutch.setClutch(value); }
	void setBrake(double value) { for (int i = 0; i < brakes.size(); i++) brakes[i].setBrakeFactor(value); }
	void setHandBrake(double value) { for (int i = 0; i < brakes.size(); i++) brakes[i].setHandbrakeFactor(value); }


	// Speedometer output based on driveshaft RPM
	double getSpeedMPS() const; // What the car sensors would report as the speed

//---- Chassis state access
	// These methods used when rendering the car model
	const MathVector<double, 3>& getPosition() const { return chassisPosition; }
	const Quaternion<double>& getOrientation() const { return chassisRotation; }

	MathVector<double, 3> getVelocity() const { return body.getVelocity(); }
	double getSpeed() const {
		// What Bullet physics says the speed "actually" is
		return body.getVelocity().magnitude();
	}
	double getSpeedDir() const; // The speed of the car in the dir it's facing (I think)
	MathVector<double, 3> getDownVector() const;
	MathVector<double, 3> getForwardVector() const;

//---- Driveline state access
	const CarTransmission& getTransmission() const { return transmission; }

//---- Wheel state access
	Quaternion<double> getWheelOrientation(WheelPosition wp) const;
	MathVector<double, 3> getWheelPosition(WheelPosition wp) const;
	double getWheelSteerAngle(WheelPosition wp) const { return wheels[wp].getSteerAngle(); }

	// Set steering angle to "val", where -1.0 = maximum left and 1.0 = maximum right
	void setSteering(const double val, const double rangeMul);
	double getMaxAngle() const { return maxAngle; }

//---- Collision params
	float collR, collR2m, collW, collH,
		  collHofs, collWofs, collLofs,
		  collFlTrigH, comOfsH, comOfsL,
		  collPosLFront, collPosLBack, collFriction,
		  collFrWMul, collFrHMul, collTopWMul,
		  collTopFr, collTopMid, collTopBack,
		  collTopFrHm, collTopMidHm, collTopBackHm;

private:
//---- Initialization
	void setNumWheels(int nw);
	void setDrive(const std::string& newDrive);
	void addMassParticle(double newMass, MathVector<double, 3> newPos);
	void setMaxSteeringAngle(double newAngle) { maxAngle = newAngle; }
	void setAngularDamping(double newDamping) { angularDamping = newDamping; }
	void addAerodynamicDevice(const MathVector<double, 3>& newPos, double dragFrontalArea,
							  double dragCoefficient, double liftSurfaceArea, double liftCoefficient, double liftEfficiency);
	void calculateMass(); // Substitute for CARDYNAMICS::UpdateMass()

//---- Driveline state
	CarFuelTank fuelTank;
	CarEngine engine;
	CarClutch clutch;
	CarTransmission transmission;
	CarDifferential diffFront, diffRear, diffCenter;

	enum { FWD = 3, RWD = 12, AWD = 15 } drive;
	double driveshaftRPM;

	// "shifted" is false if the transmission has not shifted to "gearToShift" yet.
	// It is set to true when it has
	bool autoclutch, autoshift, autorear, shifted;
	double shiftTime, remShiftTime, lastAutoClutch;
	int gearToShift;

//---- Wheel state
	int numWheels;
	std::vector<CarWheel> wheels;
	std::vector<CarBrake> brakes;
	std::vector<CarSuspension> suspension;

	std::vector<MathVector<double, 3> > wheelVels, wheelPos;
	std::vector<Quaternion<double> > wheelRots;
	std::vector<CollisionContact> wheelContact;

	MathVector<double, 3> getWheelPosition(WheelPosition wp, double displacementPercent) const;
	MathVector<double, 3> getLocalWheelPosition(WheelPosition wp, double displacementPercent) const;
	MathVector<double, 3> getWheelPositionAtDisplacement(WheelPosition wp, double displacementPercent) const;
	Quaternion<double> getWheelSteeringAndSuspensionOrientation(WheelPosition wp) const;

	MathVector<double, 3> localToWorld(const MathVector<double, 3>& local) const;

	// Sorry for the magic function; 'twas from Stuntrally
	bool wheelDriven(int i) const { return (1 << i) & drive; }

//---- CollisionWorld updater methods
	void synchronizeBody(); // Pull in pos, vel, rot, etc. from Bullet
	void tick(float dt); // Update simulation
	void updateWheelContacts(); // Check for Bullet collisions
	void synchronizeChassis(); // Report new velocities to Bullet

	// Called during tick()
	void updateTransmission(double dt);
	void updateDriveline(double dt, double driveTorque[]);
	void updateBody(double dt, double driveTorque[]);


	double calculateDriveshaftRPM() const;
	double calculateDriveshaftSpeed();

	void applyClutchTorque(double engineDrag); // Applied to engine
	void calculateDriveTorque(double driveTorque[], double clutchTorque);

	int nextGear() const; // Calculate next gear based on engine RPM
	double downshiftRPM(int gear, float avgWhHeight = 0.f) const; // Calculate downshift point based on gear and engine RPM

	double autoClutch(double lastClutch, double dt) const;
	double shiftAutoClutch() const;
	double shiftAutoClutchThrottle(double throttle, double dt);

	MathVector<double, 3> updateSuspension(int i, double dt); //Returns suspension force

	void updateWheelTransform();
	void updateWheelVelocity();

	void interpolateWheelContacts();

	void applyEngineTorqueToBody();
	void applyAerodynamicsToBody();

	void applyForce(const MathVector<double, 3>& force) { body.applyForce(force); }
	void applyForce(const MathVector<double, 3>& force,	const MathVector<double, 3>& offset) {
		body.applyForce(force, offset);
	}
	void applyTorque(const MathVector<double, 3>& torque) { body.applyTorque(torque); }

	// Apply tire friction to body; return friction in world space
	MathVector<double, 3> applyTireForce(int i, const double normalForce,
										 const Quaternion<double>& wheelSpace);
	// Apply wheel torque to chassis
	void applyWheelTorque(double dt, double driveTorque, int i,
						  MathVector<double, 3> tireFriction,
						  const Quaternion<double>& wheelSpace);

//---- Aerodynamics variables
	std::vector<CarAero> aerodynamics;
	double rotCoeff[4];

//---- Steering variables
	double maxAngle;
	double angularDamping;

//---- Traction control state
	bool absOn, tcsOn;
	std::vector<int> absActive, tcsActive;

	void doABS(int i, double normalForce);
	void doTCS(int i, double normalForce);

	CarTire* getTire(WheelPosition wp) const { return wheelContact[wp].getSurface()->tire; }

//---- Chassis state variables
	btRigidBody* chassis;
	btRigidBody* whTrigs;
	RigidBody body;
	MathVector<double, 3> centerOfMass;
	std::list<std::pair<double, MathVector<double, 3> > > massOnlyParticles;

	Quaternion<double> getBodyOrientation() const { return body.getOrientation(); } // Replaces CARDYNAMICS::Orientation
	MathVector<double, 3> getBodyPosition() const { return body.getPosition(); } // Replaces CARDYNAMICS::Position

	// Interpolated chassis state, using Bullet sim; used in model rendering
	MathVector<double, 3> chassisPosition;
	Quaternion<double> chassisRotation;

	CollisionWorld* world;

	// Bullet objects for deletion
	btAlignedObjectArray<btCollisionShape*> shapes;
	btAlignedObjectArray<btRigidBody*> rigids;
	btAlignedObjectArray<btActionInterface*> actions;
	btAlignedObjectArray<btTypedConstraint*> constraints;

#ifdef COMPILE_UNIT_TESTS
	FRIEND_TEST(CarDynamics, CalculateMass);
	FRIEND_TEST(CarDynamics, GetSpeedMPS);
	FRIEND_TEST(CarDynamics, SetSteering);
	FRIEND_TEST(CarDynamics, ApplyAerodynamicsToBody);
	FRIEND_TEST(CarDynamics, UpdateSuspension);
	FRIEND_TEST(CarDynamics, TireForceAndWheelTorque);
	FRIEND_TEST(CarDynamics, GetWheelSteeringAndSuspensionOrientation);
	FRIEND_TEST(CarDynamics, GetLocalWheelPosition);
#endif
};
