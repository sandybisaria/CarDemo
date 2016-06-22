#include "Sim.hpp"

Sim::Sim()
	: mCar(0),
	  mSceneMgr(0) {
}

Sim::~Sim() {
	delete mCar;
}

void Sim::setup(Ogre::SceneManager* sceneMgr) {
	mSceneMgr = sceneMgr;

	// Right now, will be using the Ferrari 360 Modena
	mCar = new Car(0);
	mCar->setup("360", mSceneMgr);
}

void Sim::update(float dt) {
	mCar->update(dt);
}
