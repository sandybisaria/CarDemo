#include "CarModel.hpp"

#include "vdrift/cardefs.h"

CarModel::CarModel(int id)
	: mId(id), mCar(0) {
	setNumWheels(DEF_WHEELS);

	//TODO Implement CarModel::Defaults?
}

CarModel::~CarModel() {

}

void CarModel::load() {

}

void CarModel::setNumWheels(int n) {
	numWheels = n;
	wheelPos.resize(n); wheelRadius.resize(n); wheelWidth.resize(n);
	//TODO whTrail and whTemp, for when implementing trails/particles
	wheelNodes.resize(n); brakeNodes.resize(n); //TODO wheelEmitNodes?
}
