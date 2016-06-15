#include "Sim.hpp"

Sim::Sim()
	: mCar(0) {
}

Sim::~Sim() {
	delete mCar;
}

void Sim::setup() {
	mCar = new Car();
	// Right now, will be using the Ferrari 360 Modena
	mCar->setup("360");
}

void Sim::update(float dt) {

}
