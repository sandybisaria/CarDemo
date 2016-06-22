#pragma once

#include "MathVector.hpp"
#include "CarEngine.hpp"
#include "CarClutch.hpp"
#include "CarTransmission.hpp"
#include "CarDifferential.hpp"
#include "CarBrake.hpp"
#include "CarFuelTank.hpp"
#include "CarSuspension.hpp"
#include "CarWheel.hpp"

#include "../util/ConfigFile.hpp"

#include <sstream>
#include <cassert>
#include <vector>

class CarDynamics {
public:
	CarDynamics();

	// Initialization
	void setNumWheels(int nw);
	bool loadFromConfig(ConfigFile& cf);
	void setDrive(const std::string& newDrive);
	void addMassParticle(double newMass, MathVector<double, 3> newPos);
	void setMaxSteeringAngle(double newAngle) { maxAngle = newAngle; }
	void setAngularDamping(double newDamping) { angularDamping = newDamping; }

private:
	// Driveline state
	CarFuelTank fuelTank;
	CarEngine engine;
	CarClutch clutch;
	CarTransmission transmission;
	CarDifferential diffFront, diffRear, diffCenter;

	enum { FWD = 3, RWD = 12, AWD = 15 } drive;

	double shiftTime;

	// Wheel state
	int numWheels;
	std::vector<CarWheel> wheels;
	std::vector<CarBrake> brakes;
	std::vector<CarSuspension> suspension;

	// Aerodynamics
	double rotCoeff[4];

	// Steering
	double maxAngle;
	double angularDamping;

	// Mass
	std::list<std::pair<double, MathVector<double, 3> > > massOnlyParticles;
};
