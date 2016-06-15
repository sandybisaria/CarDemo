#pragma once

class Car {
public:
	Car();
	~Car();

	void setup();

private:
	void setNumWheels(int n);

	int numWheels;
};
