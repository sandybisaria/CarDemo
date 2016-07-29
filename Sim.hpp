#pragma once

#include "vdrift/CarControlMap.hpp"

#include "btOgre/BtOgreDebug.h"

#include "OgreSceneManager.h"

#include <OIS.h>

#include <vector>

// The Sim class is responsible for managing aspects of the simulation, like
// the vehicles, scene, and physics engine.
class Sim {
public:
	Sim(class App* app);
	~Sim();

	void setup(Ogre::SceneManager* sceneMgr);

	void update(float dt);

	Ogre::Vector3 getCameraPosition();
	Ogre::Quaternion getCameraOrientation();

	class CollisionWorld* getCollisionWorld() { return world; }

	class Scene* scene;

	class TerrainSurface* getTerrainSurface(std::string name);

	void keyPressed(const OIS::KeyEvent& ke);
	void keyReleased(const OIS::KeyEvent& ke);

private:
	class App* mApp;

	Ogre::SceneManager* mSceneMgr;

	CollisionWorld* world;

	std::vector<class Car*> cars;
	int numCars; // Number of cars to load
	int idCarToWatch, idCarToControl;

	std::vector<class BasicController*> controllers;

	// Keyboard control
	CInput* carInput;
	CarControlMapLocal localMap;

	// Bullet-Ogre debugging
	BtOgre::DebugDrawer* debugDraw;
	bool enableDebug;
};
