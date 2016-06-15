#include "Sim.hpp"

Sim::Sim()
	: mCar(0) {
}

Sim::~Sim() {
	delete mCar;
}

void Sim::setup() {
	mCar = new Car();
}

void Sim::update(float dt) {

}
