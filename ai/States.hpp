#pragma once

#include "BasicController.hpp"

#include "../vdrift/MathVector.hpp"

class BaseState {
public:
	virtual ~BaseState() { };
	virtual BaseState* update(float dt) { return NULL; };
};

// Maintain a given velocity and direction
class ConstantState : public BaseState {
public:
	ConstantState(ControllerInterface* interface, double speed, double angle);
	virtual ~ConstantState() { }

	virtual BaseState* update(float dt);

private:
	ControllerInterface* mInterface;
	double mSpeed;
	double mAngle;
};

// Drive to the specified waypoint
class WaypointState : public BaseState {
public:
	// Ground waypoint (using Bullet's XY plane)
	WaypointState(ControllerInterface* interface, MathVector<double, 2> waypoint, double radius);
	virtual ~WaypointState() { }

	virtual BaseState* update(float dt);
private:
	ControllerInterface* mInterface;
	MathVector<double, 2> mWaypoint;
	double mRadius;
};