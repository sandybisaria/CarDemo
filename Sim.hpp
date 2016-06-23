#pragma once

#include "vdrift/Car.hpp"
#include "vdrift/CollisionWorld.hpp"

#include "OgreSceneManager.h"

class Sim {
public:
	Sim();
	~Sim();

	void setup(Ogre::SceneManager* sceneMgr);
	void update(float dt);

private:
	Ogre::SceneManager* mSceneMgr;

	CollisionWorld world;
	Car* mCar;
};
