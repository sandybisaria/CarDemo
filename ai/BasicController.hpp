#pragma once

#include "../vdrift/Car.hpp"
#include "../vdrift/CarConstants.hpp"

class BasicController {
public:
	BasicController(Car* car);
	virtual ~BasicController();

	void reset();

	const std::vector<double>& updateInputs(float dt);

private:
	Car* mCar;
	std::vector<double> inputs;

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
	void setTargetAngle(double newAngle);
	void updateDirection(float dt);

	bool isTargetPointEnabled;
	MathVector<double, 2> targetPoint; // Target point on ground in Bullet coords (x-y plane)
public:
	void setTargetPoint(MathVector<double, 2> newPoint);
};
