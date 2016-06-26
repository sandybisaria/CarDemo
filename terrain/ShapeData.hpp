#pragma once

const static int
	SU_Road			= 0x100,
	SU_Pipe			= 0x200,
	SU_RoadWall		= 0x300, // Road column consolidated with this
	SU_Terrain		= 0x500,
	SU_Vegetation	= 0x600, // Trees, rocks, etc.
	SU_Border		= 0x700, // World border planes
	SU_ObjectStatic	= 0x800,
	SU_Fluid 		= 0x900
;

// Info for special collision objects
namespace ShapeType {
	enum ShapeType { Car, Fluid, Wheel, Other };
}

class CarDynamics;
//TODO Add FluidBox class
struct ShapeData {
	ShapeType::ShapeType type;
	CarDynamics* dyn;
	int wheelNum;

	ShapeData(ShapeType::ShapeType t)
		: type(t), dyn(0), wheelNum(0) { }

	ShapeData(ShapeType::ShapeType t, CarDynamics* d, int whNum = 0)
		: type(t), dyn(d), wheelNum(whNum) { }
};
