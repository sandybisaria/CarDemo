#pragma once

// Based on TRACKSURFACE in vdrift/tracksurface.h
class TerrainSurface {
public:
	//TODO Do we need the TYPE enum?

	float friction, frictionX, frictionY; // frictionX and frictionY are multipliers
	float bumpWavelength, bumpAmplitude; //TODO Do we need "2" versions too?
	float rollingDrag, rollingResist;

	//TODO Add CARTIRE fields

	TerrainSurface()
		: friction(1.0f), frictionX(1.0f), frictionY(1.0f),
		  bumpWavelength(10.0f), bumpAmplitude(0.0f),
		  rollingDrag(1.0f), rollingResist(1.0f) {}

	bool operator==(const TerrainSurface& other) const {
		return (friction == other.friction) && (frictionX == other.frictionX) && (frictionY == other.frictionY) &&
			   (bumpWavelength == other.bumpWavelength) && (bumpAmplitude == other.bumpAmplitude) &&
			   (rollingDrag == other.rollingDrag) && (rollingResist == other.rollingResist);
	}
};
