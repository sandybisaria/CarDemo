#pragma once

class Sim;
class Road;

#include "../Sim.hpp"

#include "ShapeData.hpp"

#include <OgreSceneManager.h>

#include <Terrain/OgreTerrain.h>
#include <Terrain/OgreTerrainGroup.h>

#include <BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>

class Scene {
public:
	Scene(Ogre::SceneManager* sceneMgr);
	~Scene();

	void setupTerrain(Sim* sim);

	void update();

	int terrSize;
	float worldSize;

private:
	void configureTerrainDefaults();
	void defineTerrain();

	void createBulletTerrain();

	Sim* mSim;

	Ogre::SceneManager* mSceneMgr;
	Ogre::Light* sun;

	Ogre::TerrainGlobalOptions* mTerrainGlobals;
	Ogre::TerrainGroup* mTerrainGroup;

	void setupRoad();
	std::vector<Road*> mRoads;
};
