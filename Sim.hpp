#pragma once

#include "terrain/Scene.hpp"
class Scene;

#include "vdrift/CollisionWorld.hpp"
#include "vdrift/Car.hpp"
#include "vdrift/CarControlMap.hpp"

#include "CInput.hpp"

#include "OgreSceneManager.h"

#include <OIS.h>

class Sim {
public:
	Sim();
	~Sim();

	void setup(Ogre::SceneManager* sceneMgr);
	void update(float dt);

	CollisionWorld* getCollisionWorld() { return world; }

	Scene* scene;

	void keyPressed(const OIS::KeyEvent& ke);
	void keyReleased(const OIS::KeyEvent& ke);

private:
	Ogre::SceneManager* mSceneMgr;

	CollisionWorld* world;
	Car* car;
	CInput* carInput;
	CarControlMapLocal localMap;

	int frameRate; //TODO Configurable
	int targetTime, frame;
};
