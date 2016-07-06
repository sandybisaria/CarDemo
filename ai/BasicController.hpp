#pragma once

#include "../vdrift/Car.hpp"

class BasicController {
public:
	BasicController(Car* car);
	virtual ~BasicController();

	void reset();

private:
	Car* mCar;

	double targetSpeed; // Target speed in m/s
};
