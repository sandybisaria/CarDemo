#pragma once

#include "MathVector.hpp"

class CarFuelTank {
public:
	// Default makes an S2000-like car
	CarFuelTank()
		: capacity(0.0492), density(730.0), mass(0), volume(0) {
		fill();
		updateMass();
	}

	void setCapacity(double c) { capacity = c; }
	void setDensity(double d) { density = d; }
	void setVolume(double v) { volume = v; }
	void setPosition(const MathVector<double, 3>& p) { position = p; }
	MathVector<double, 3> getPosition() const { return position; }
	double getMass() const { return mass; }

	void fill() { volume = capacity; }
	bool isEmpty() const { return volume <= 0.0; }
	double fuelPercent() const { return volume / capacity; }
	void consume(double amt) {
		volume -= amt;
		volume = std::max(0.0, volume);
		updateMass();
	}

private:
	// Constants
	double capacity;
	double density;
	MathVector<double, 3> position;

	// Variables
	double mass;
	double volume;

	// For info only
	void updateMass() { mass = density * volume; }
};
