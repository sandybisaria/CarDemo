#pragma once

class App;
class Scene;
class Car;

#include "terrain/Scene.hpp"

#include "vdrift/CollisionWorld.hpp"
#include "vdrift/Car.hpp"

#include "vdrift/CarControlMap.hpp"

#include "btOgre/BtOgreDebug.h"

#include "CInput.hpp"
#include "App.hpp"

#include "OgreSceneManager.h"

#include <OIS.h>

class Sim {
public:
	Sim(App* app);
	~Sim();

	void setup(Ogre::SceneManager* sceneMgr);
	void update(float dt);

	CollisionWorld* getCollisionWorld() { return world; }

	Scene* scene;

	TerrainSurface* getTerrainSurface(std::string name);

	void keyPressed(const OIS::KeyEvent& ke);
	void keyReleased(const OIS::KeyEvent& ke);

private:
	App* mApp;

	Ogre::SceneManager* mSceneMgr;

	CollisionWorld* world;
	Car* car;
	CInput* carInput;
	CarControlMapLocal localMap;

	BtOgre::DebugDrawer* debugDraw;

	const double frameRate; //TODO Configurable
	int targetTime, frame;
};
