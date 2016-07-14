#pragma once

#include "BasicController.hpp"

#include "../vdrift/MathVector.hpp"

#include <queue>

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
	// Ground waypoint (using Bullet's XY plane); radius defines how close the car must be within the waypoint
	WaypointState(ControllerInterface* interface, MathVector<double, 2> waypoint, double radius);
	virtual ~WaypointState() { }

	virtual BaseState* update(float dt);

private:
	ControllerInterface* mInterface;
	MathVector<double, 2> mWaypoint;
	double mRadius;
};

// Perform a 90-degree turn left or right
class TurnState : public BaseState {
public:
	TurnState(ControllerInterface* interface, bool isLeftTurn, double turnRadius, int subdivisions);
	virtual ~TurnState() { }

	virtual BaseState* update(float dt);

private:
	ControllerInterface* mInterface;

	WaypointState* mWaypointState;
	std::queue< MathVector<double, 2> > mWaypoints;

	double startSpeed;
	MathVector<double, 2> desiredFinalDir;
};