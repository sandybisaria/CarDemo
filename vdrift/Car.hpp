#pragma once

#include <iostream>

class Car {
public:
	Car();
	~Car();

	void setup(std::string carName);

private:
	void setNumWheels(int n);

	std::string mCarName;
	int numWheels;
};
