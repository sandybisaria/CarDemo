#pragma once

#include "vdrift/Car.hpp"

#include "OgreSceneManager.h"

class Sim {
public:
	Sim();
	~Sim();

	void setup(Ogre::SceneManager* sceneMgr);
	void update(float dt);

private:
	Ogre::SceneManager* mSceneMgr;

	Car* mCar;
};
