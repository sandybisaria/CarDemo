#pragma once

#include "CarTire.hpp"

#include <string>

// Based on Stuntrally's TRACKSURFACE class
class TerrainSurface {
public:
	enum Type { NONE, ASPHALT, GRASS, GRAVEL, CONCRETE, SAND, COBBLES, NumTypes };

	// Omitted the "2" versions of variables
	float friction, frictionX, frictionY; // frictionX and frictionY are multipliers
	float bumpWavelength, bumpAmplitude;
	float rollingDrag, rollingResist;

	Type type;
	std::string name;
	std::string tireName;
	CarTire* tire;

//	static CarTire* TIRE_DEFAULT; // Assigned during App::loadSurfaces

	TerrainSurface()
		: friction(1.0f), frictionX(1.0f), frictionY(1.0f),
		  bumpWavelength(10.0f), bumpAmplitude(0.0f),
		  rollingDrag(1.0f), rollingResist(1.0f),
		  type(GRASS), tireName("DEFAULT"), tire(CarTire::none()) {}

	void setType(unsigned int i) { type = i < NumTypes ? (Type)i : NumTypes; }

	bool operator==(const TerrainSurface& other) const {
		return (friction == other.friction) && (frictionX == other.frictionX) && (frictionY == other.frictionY) &&
			   (bumpWavelength == other.bumpWavelength) && (bumpAmplitude == other.bumpAmplitude) &&
			   (rollingDrag == other.rollingDrag) && (rollingResist == other.rollingResist) &&
			   (tire == other.tire) && (type == other.type);
	}

	static TerrainSurface* none() {
		static TerrainSurface s;
		return &s;
	}
};

// Add the terrain type names when you need them...
