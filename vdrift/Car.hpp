#pragma once

#include "../CarModel.hpp"
class CarModel;
#include "cardefs.h"
#include "configfile.h"

// Analog to Stuntrally's CAR class (vdrift/car.h)
class Car {
public:
	Car();
	~Car();

	int mId; // To uniquely identify the Car instance (and match with corresponding CarModel)

	//TODO May need refs to Sim, App, and CarModel
	CarModel* mCarModel;

	int numWheels;
	void setNumWheels(int n);
};
