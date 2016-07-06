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

#include <vector>

class Sim {
public:
	Sim(App* app);
	~Sim();

	void setup(Ogre::SceneManager* sceneMgr);
	void update(float dt);

	Ogre::Vector3 getCameraPosition();
	Ogre::Quaternion getCameraOrientation();

	CollisionWorld* getCollisionWorld() { return world; }

	Scene* scene;

	TerrainSurface* getTerrainSurface(std::string name);

	// Explicitly called by the App
	void keyPressed(const OIS::KeyEvent& ke);
	void keyReleased(const OIS::KeyEvent& ke);

private:
	App* mApp;

	Ogre::SceneManager* mSceneMgr;

	CollisionWorld* world;

	const double frameRate;
	int targetTime;

	std::vector<Car*> cars;
	const int numCars; // Number of cars to load

	// Keyboard control
	CInput* carInput;
	CarControlMapLocal localMap;

	BtOgre::DebugDrawer* debugDraw;
};
