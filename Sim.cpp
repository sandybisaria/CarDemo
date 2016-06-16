#include "Sim.hpp"

Sim::Sim()
	: mCar(0),
	  mSceneMgr(0) {
}

Sim::~Sim() {
	delete mCar;
}

void Sim::setup(Ogre::SceneManager* sceneMgr) {
	mCar = new Car();
	mSceneMgr = sceneMgr;

	// Right now, will be using the Ferrari 360 Modena
	mCar->setup("360", mSceneMgr);
}

void Sim::update(float dt) {

}
