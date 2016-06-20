#pragma once

#include "cardefs.h"
#include "configfile.h"

// Analog to Stuntrally's CAR class (vdrift/car.h)
class Car {
public:
	//TODO May need refs to Sim, App, and CarModel
	Car();
	~Car();

	int numWheels;
	void setNumWheels(int n);
};
