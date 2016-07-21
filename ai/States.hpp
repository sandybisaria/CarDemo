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

// Perform a turn left or right
class TurnState : public BaseState {
public:
	// Angle passed in as degrees
	TurnState(ControllerInterface* interface, bool isLeftTurn, double turnRadius, double angle = 90);
	virtual ~TurnState() { }

	virtual BaseState* update(float dt);

private:
	ControllerInterface* mInterface;

	double turn, startSpeed;
	static double getTurn(double turnRadius, double speed);

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
	ControllerInterface* mInterface;

	double turn, startSpeed;

	// For debugging and testing
	bool canHaveLooped, looped; MathVector<double, 2> startDir;
	double minX, minY, maxX, maxY, radiusX, radiusY;
};