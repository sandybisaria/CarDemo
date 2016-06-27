#include "Sim.hpp"

Sim::Sim()
	: mSceneMgr(0),
	  world(0) {
}

Sim::~Sim() {
	delete world;
}

void Sim::setup(Ogre::SceneManager* sceneMgr) {
	mSceneMgr = sceneMgr;

	world = new CollisionWorld();

	car = new Car(0);
	car->setup("360", mSceneMgr, *world);
}

void Sim::update(float dt) {
	if (dt > 0) world->update(dt);

	car->update(dt);
}
