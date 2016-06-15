#pragma once

#include "vdrift/Car.hpp"

class Sim {
public:
	Sim();
	~Sim();

	void setup();
	void update(float dt);

private:
	Car* mCar;
};
