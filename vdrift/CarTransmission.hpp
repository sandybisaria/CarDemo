#pragma once

#include <cmath>
#define _USE_MATH_DEFINES

#include <iostream> //FIXME DELETE

class CarTransmission {
public:
	// Default makes an S2000-like car
	CarTransmission()
		: forwardGears(1), reverseGears(0), gear(0),
		  driveshaftRPM(0), crankshaftRPM(0) { gearRatios[0] = 0; } // I missed this important line!

	int getGear() const { return gear; }
	int getForwardGears() const { return forwardGears;}
	int getReverseGears() const { return reverseGears; }

	void shift(int newGear) {
		if (newGear <= forwardGears && newGear >= -reverseGears)
			gear = newGear;
	}

	// Ratio is (driveshaft speed / crankshaft speed)
	void setGearRatio(int gear, double ratio) {
		gearRatios[gear] = ratio;

		// Determine the number of consecutive forward and reverse gears
		forwardGears = 0;
		int key = 1;
		while(gearRatios.find(key) != gearRatios.end()) {
			forwardGears++;
			key++;
		}
		reverseGears = 0;
		key = -1;
		while(gearRatios.find(key) != gearRatios.end()) {
			reverseGears++;
			key--;
		}

		std::cout << gearRatios[gear] << " " << gear << " " << ratio << std::endl;
	}
	double getGearRatio(int gear) const {
		double ratio = 1.0;
		std::map<int, double>::const_iterator i = gearRatios.find(gear);
		if (i != gearRatios.end())
			ratio = i->second;
		return ratio;
	}
	double getCurrentGearRatio() const { return getGearRatio(gear); }

	// Get the torque on the driveshaft due to given clutch torque
	double getTorque(double clutchTorque) { return clutchTorque * gearRatios[gear]; }

	// Get the rotational speed of the clutch given the rotational speed of the driveshaft
	double calculateClutchSpeed(double driveshaftSpeed) {
		driveshaftRPM = driveshaftSpeed * 30.0 / M_PI;
		crankshaftRPM = driveshaftSpeed * gearRatios[gear] * 30.0 / M_PI;

		return driveshaftSpeed * gearRatios[gear];
	}
	double getClutchSpeed(double driveshaftSpeed) const {
		std::map<int, double>::const_iterator i = gearRatios.find(gear);
		assert(i != gearRatios.end());
		return driveshaftSpeed * i->second;
	}

private:
	// Constants
	std::map<int, double> gearRatios; // Gear number and ratio; reverse gears are negative.
	int forwardGears; // Number of consecutive forward gears
	int reverseGears; // Number of consecutive reverse gears

	// Variables
	int gear; // Current gear

	// For info only;
	double driveshaftRPM;
	double crankshaftRPM;
};
