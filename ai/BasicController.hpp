#pragma once

class BaseState;
class ControllerInterface;

#include "../vdrift/Car.hpp"
#include "../vdrift/CarConstants.hpp"

class BasicController {
	friend class ControllerInterface;
public:
	BasicController(Car* car);
	~BasicController();

	void reset();

	const std::vector<double>& updateInputs(float dt);

//	void setTargetPoint(MathVector<double, 2> newPoint);
//	void turn(bool isLeft, double turnRadius);

private:
	Car* mCar;
	std::vector<double> inputs;

	BaseState* currentState;

	double getAngle(MathVector<double, 2> fromDir, MathVector<double, 2> toDir);
	MathVector<double, 2> toFlatVector(MathVector<double, 3> v, bool normalize = true);

	double kPSpeed,
		kISpeed, iSpeedAcc,
		kDSpeed, dLastESpeed;
	double targetSpeed; // Target speed in m/s
	void setTargetSpeed(double newSpeed);
	void updateSpeed(float dt);

	double kPAngle,
		kIAngle, iAngleAcc,
		kDAngle, dLastEAngle;
	double targetAngle; // Target angle in rad (note that negative angles point to the LEFT or CCW)
	MathVector<double, 3> initDir; // The initial direction (when targetAngle was set)
	void setTargetAngle(double newAngle, bool resetDir);
	void updateDirection(float dt);

//	bool isTargetPointEnabled;
//	MathVector<double, 2> targetPoint; // Target point on ground in Bullet coords (x-y plane)
//	void updatePointTargeting();
};

// Interface that exposes BasicController methods to its states
class ControllerInterface {
	friend class BasicController;
public:
	// Tell the car to drive at this speed (in m/s)
	void setTargetSpeed(double newSpeed) { mController->setTargetSpeed(newSpeed); }

	// Tell the car to drive at an angle.
	// If resetDir is true, then angle is relative to current dir. Otherwise, use the previously-stored dir.
	// Note that angle signs are reversed (left/CCW is negative).
	void setTargetAngle(double newAngle, bool resetDir = true) { mController->setTargetAngle(newAngle, resetDir); }

private:
	ControllerInterface(BasicController* controller)
		: mController(controller) { }

	BasicController* mController;
};