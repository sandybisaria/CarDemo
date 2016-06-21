#pragma once

#include "../CarModel.hpp"
class CarModel;
#include "cardefs.h"
#include "configfile.h"
#include "CarDynamics.hpp"

// Analog to Stuntrally's CAR class (vdrift/car.h)
class Car {
public:
	Car();
	~Car();

	// True on success
	bool load(std::string carType, CONFIGFILE& cf, const MATHVECTOR<float, 3> pos, const QUATERNION<float> rot,
			  int id);

	std::string mCarType;
	int mId; // To uniquely identify the Car instance (and match with corresponding CarModel)

	CarModel* mCarModel;

	CarDynamics cd;

	int numWheels;
	void setNumWheels(int n);
};
