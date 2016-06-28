#pragma once

#include "terrain/Scene.hpp"
class Scene;

#include "vdrift/CollisionWorld.hpp"
#include "vdrift/Car.hpp"

#include "CarInput.hpp"

#include "OgreSceneManager.h"

class Sim {
public:
	Sim();
	~Sim();

	void setup(Ogre::SceneManager* sceneMgr);
	void update(float dt);

	CollisionWorld* getCollisionWorld() { return world; }

	Scene* scene;

private:
	Ogre::SceneManager* mSceneMgr;

	CollisionWorld* world;
	Car* car;
	CarInput* carInput;

	int frameRate; //TODO Configurable
	int targetTime, frame;
};
