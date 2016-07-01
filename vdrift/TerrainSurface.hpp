#pragma once

#include "CarTire.hpp"

#include <string>

// Based on TRACKSURFACE in vdrift/tracksurface.h
class TerrainSurface {
public:
	enum TYPE { NONE, ASPHALT, GRASS, GRAVEL, CONCRETE, SAND, COBBLES, NumTypes };


	float friction, frictionX, frictionY; // frictionX and frictionY are multipliers
	float bumpWavelength, bumpAmplitude; //TODO Do we need "2" versions too?
	float rollingDrag, rollingResist;

	TYPE type;
	std::string name;
	std::string tireName;
	CarTire* tire;

	static CarTire* TIRE_DEFAULT; //TODO Set in GAME::LoadTires (or its analog)

	TerrainSurface()
		: friction(1.0f), frictionX(1.0f), frictionY(1.0f),
		  bumpWavelength(10.0f), bumpAmplitude(0.0f),
		  rollingDrag(1.0f), rollingResist(1.0f),
		  type(GRASS), tireName("DEFAULT"), tire(CarTire::none()) {}

	void setType(unsigned int i) { type = i < NumTypes ? (TYPE)i : NumTypes; }

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

//TODO Will making char[][] fix this?
//const static std::string terrTypeName[TerrainSurface::NumTypes + 1] {
//	"[none]", "Asphalt", "Grass", "Gravel", "Concrete", "Sand", "Cobbles", "[all]"
//};
