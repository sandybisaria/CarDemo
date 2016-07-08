#pragma once

#include "../vdrift/Car.hpp"
#include "../vdrift/CarConstants.hpp"

class BasicController {
public:
	BasicController(Car* car);
	virtual ~BasicController();

	void reset();

	void setTargetSpeed(double newSpeed);

	const std::vector<double>& updateInputs(float dt);

private:
	Car* mCar;
	std::vector<double> inputs;

	double kPSpeed,
		   kDSpeed, dLastESpeed,
		   kISpeed, iSpeedAcc;
	double targetSpeed; // Target speed in m/s
};
