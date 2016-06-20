#include "Sim.hpp"

Sim::Sim()
	: mSceneMgr(0) {
}

Sim::~Sim() {
}

void Sim::setup(Ogre::SceneManager* sceneMgr) {
	mSceneMgr = sceneMgr;
}

void Sim::update(float dt) {
}
