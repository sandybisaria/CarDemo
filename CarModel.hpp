#pragma once

#include "vdrift/Car.hpp"
class Car;
#include "vdrift/mathvector.h"

#include <OgreSceneNode.h>

// "Ogre" part of the car, mostly handling rendering/visualization
// Based on Stuntrally's CarModel class
class CarModel {
public:
	CarModel(int id, std::string carModelName);
	~CarModel();

	int mId; // To uniquely identify the CarModel instance (and match with corresponding Car)
	std::string mCarModelName; // Car model identifier

	// Creation
	void load();

	Car* mCar; // The vdrift Car, constructed during load()

	int numWheels;
	void setNumWheels(int n);

	// Model params from .car config
	std::vector<MATHVECTOR<float, 3> > wheelPos;
	std::vector<float> wheelRadius, wheelWidth;

	// Wheel nodes
	std::vector<Ogre::SceneNode*> wheelNodes, brakeNodes;
};
