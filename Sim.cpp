#include "Sim.hpp"

Sim::Sim()
	: mCar(0),
	  mSceneMgr(0) {
}

Sim::~Sim() {
	world.clear();

	delete mCar;
}

void Sim::setup(Ogre::SceneManager* sceneMgr) {
	mSceneMgr = sceneMgr;

	// Right now, will be using the Ferrari 360 Modena
	mCar = new Car(0);
	mCar->setup("360", mSceneMgr, world);
}

void Sim::update(float dt) {
	if (dt > 0) world.update(dt);

	mCar->update(dt);
}
