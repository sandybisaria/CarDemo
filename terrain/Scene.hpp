#pragma once

class Sim;

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

	const static int terrSize = 513;
	const static float worldSize = 10000.0;

private:
	void configureTerrainDefaults();
	void defineTerrain();

	void createBulletTerrain();

	Sim* mSim;

	Ogre::SceneManager* mSceneMgr;
	Ogre::Light* sun;

	Ogre::TerrainGlobalOptions* mTerrainGlobals;
	Ogre::TerrainGroup* mTerrainGroup;
};
