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

//---- Update methods
	const std::vector<double>& updateInputs(float dt);

//---- Control methods
	void goToPoint(MathVector<double, 2> waypoint, double radius);
	void setSpeed(double speed);

private:
	Car* mCar;
	std::vector<double> inputs;

	ControllerInterface* myInterface;
	BaseState* currentState;

	double kPSpeed,
		kISpeed, iSpeedAcc,
		kDSpeed, dLastESpeed;
	double targetSpeed; // Target speed in m/s
	double lastSpeed;
	void setTargetSpeed(double newSpeed); // Tell the car to drive at this speed (in m/s)
	void updateSpeed(float dt);

	double kPAngle,
		kIAngle, iAngleAcc,
		kDAngle, dLastEAngle;
	double targetAngle; // Target angle in rad (note that negative angles point to the LEFT or CCW)
	MathVector<double, 3> initDir; // The initial direction (when targetAngle was set)
	/* Tell the car to drive at an angle.
	 * If resetDir is true, then angle is relative to current dir. Otherwise, use the previously-stored dir.
	 * Note that angle signs are reversed (left/CCW is negative). */
	void setTargetAngle(double newAngle, bool resetDir);
	void updateDirection(float dt); bool dirAlreadyUpdated;

	static double getAngle(MathVector<double, 2> fromDir, MathVector<double, 2> toDir);
	// "Flat" in that height is ignored; normalized by default
	static MathVector<double, 2> toFlatVector(MathVector<double, 3> vec, bool normalize = true);
};

// Interface that exposes BasicController methods to its states
class ControllerInterface {
	friend class BasicController;
public:
//---- Setter methods
	void setTargetSpeed(double newSpeed) { mController->setTargetSpeed(newSpeed); }
	void setTargetAngle(double newAngle, bool resetDir = true) { mController->setTargetAngle(newAngle, resetDir); }
	void setSteering(double steering); // -1 (full left) to 1 (full right); overrides the Controller's updateDirection

//---- Getter methods
	MathVector<double, 2> getCarPosition();
	MathVector<double, 2> getCarDirection(); // Direction is normalized
	double getCarSpeed();

//---- Utility methods
	static double getAngle(MathVector<double, 2> fromDir, MathVector<double, 2> toDir) {
		return BasicController::getAngle(fromDir, toDir);
	}
	static MathVector<double, 2> toFlatVector(MathVector<double, 3> vec, bool normalize = true) {
		return BasicController::toFlatVector(vec, normalize);
	}

private:
	ControllerInterface(BasicController* controller)
		: mController(controller) { }

	BasicController* mController;
};