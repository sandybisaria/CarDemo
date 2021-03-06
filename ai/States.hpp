#pragma once

#include "BasicController.hpp"

#include "../vdrift/MathVector.hpp"
#include "../terrain/SceneObject.hpp"

#include <queue>

// Base class for all states
class BaseState {
public:
	BaseState(ControllerInterface* interface)
		: mInterface(interface), mActive(false) { };
	virtual ~BaseState() { };
	virtual BaseState* update(float dt) { return NULL; };
	bool isActive() const { return mActive; }

protected:
	ControllerInterface* mInterface;
	bool mActive;
};

//---- Environment-agnostic states
// Maintain a given velocity and direction
class ConstantState : public BaseState {
public:
	ConstantState(ControllerInterface* interface, double speed, double angle = 0);
	virtual ~ConstantState() { }

	virtual BaseState* update(float dt);

private:
	double mSpeed;
	double mAngle;
};

// Drive to the specified waypoint
class WaypointState : public BaseState {
public:
	// Ground waypoint (using Bullet's XY plane)
	// Radius defines how close the car must be within the waypoint
	WaypointState(ControllerInterface* interface, MathVector<double, 2> waypoint,
				  double radius);
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
	TurnState(ControllerInterface* interface, bool isLeftTurn, double turnRadius,
			  double angle = 90);
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
	ConstantTurnState(ControllerInterface* interface, double turn,
					  double startSpeed = 0);
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

//---- Environment-aware states
// Essentially a ConstantState that stops at stop signs
class StopSignState : public BaseState {
public:
	StopSignState(ControllerInterface* interface, double speed, double angle = 0);
	virtual ~StopSignState() { }

	virtual BaseState* update(float dt);

private:
	double initSpeed, initAngle;

	BaseState* currState;

	enum { ST_DRIVE, ST_BRAKE, ST_WAIT } stage;
	std::string lastStopSign;
	double countdown;
};

// ConstantState that obeys traffic lights
class TrafficLightState : public BaseState {
public:
	TrafficLightState(ControllerInterface* interface, double speed, double angle = 0);
	virtual ~TrafficLightState() { }

	virtual BaseState* update(float dt);

private:
	double initSpeed, initAngle;

	BaseState* currState;

	TrafficLight* lastLight;
	TrafficLightStatus lastStatus;

	void checkLight();
};

//---- Composite states
// ConstantState that obeys all signage (ideally)
class CompositeState : public BaseState {
public:
	CompositeState(ControllerInterface* interface, double speed, double angle = 0);
	virtual ~CompositeState() { }

	virtual BaseState* update(float dt);

private:
	double initSpeed, initAngle;

	StopSignState* ssState;
	TrafficLightState* tlState;
};