#pragma once

#include "../vdrift/Car.hpp"
#include "../vdrift/CarConstants.hpp"

class BasicController {
public:
	BasicController(Car* car);
	virtual ~BasicController();

	void reset();

	const std::vector<double>& updateInputs(float dt);
	void setTargetAngle(double newAngle);

private:
	Car* mCar;
	std::vector<double> inputs;

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

	MathVector<double, 2> toFlatVector(MathVector<double, 3> v);

	void updateDirection(float dt);
};
