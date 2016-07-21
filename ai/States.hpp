#pragma once

#include "BasicController.hpp"

#include "../vdrift/MathVector.hpp"

#include <queue>

// Base class for all states
class BaseState {
public:
	BaseState(ControllerInterface* interface) : mInterface(interface) { };
	virtual ~BaseState() { };
	virtual BaseState* update(float dt) { return NULL; };

protected:
	ControllerInterface* mInterface;
};

// Maintain a given velocity and direction
class ConstantState : public BaseState {
public:
	ConstantState(ControllerInterface* interface, double speed, double angle);
	virtual ~ConstantState() { }

	virtual BaseState* update(float dt);

private:
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
	MathVector<double, 2> mWaypoint;
	double mRadius;
};

// Perform a turn left or right
class TurnState : public BaseState {
public:
	// Angle passed in as degrees
	TurnState(ControllerInterface* interface, bool isLeftTurn, double turnRadius, double angle = 90);
	virtual ~TurnState() { }

	virtual BaseState* update(float dt);

private:
	double turn, startSpeed;
	MathVector<double, 2> finalPoint, finalDir;
	double lastDist;
};

// Debug state used for setting a constant steering input
class ConstantTurnState : public BaseState {
public:
	ConstantTurnState(ControllerInterface* interface, double turn, double startSpeed = 0);
	virtual ~ConstantTurnState() { }

	virtual BaseState* update(float dt);

	bool hasLooped() { return looped; }
	double getAverageRadius() { return (radiusX + radiusY) / 2; }

private:
	double turn, startSpeed;

	// For debugging and testing
	bool canHaveLooped, looped; MathVector<double, 2> startDir;
	double minX, minY, maxX, maxY, radiusX, radiusY;
};

// Perform a lane change
class LaneChangeState : public BaseState {
public:
	LaneChangeState(ControllerInterface* interface, bool isLeft, double laneWidth);
	virtual ~LaneChangeState() { }

	virtual BaseState* update(float dt);

private:
	bool mIsLeft, halfwayDone;
	double theta, radius, startSpeed;
	MathVector<double, 2> finalDir, startPos;
	TurnState* currState;
};