#pragma once

#include "SceneObject.hpp"
#include "ShapeData.hpp"

#include <BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>

#include <OgreSceneManager.h>

#include <Terrain/OgreTerrain.h>
#include <Terrain/OgreTerrainGroup.h>

class Scene {
public:
	Scene(Ogre::SceneManager* sceneMgr);
	~Scene();

	void setup(class Sim* sim);

	void update(float dt);

	int terrSize;
	float worldSize;

private:
	void configureTerrainDefaults();
	void defineTerrain();

	void createBulletTerrain();

	class Sim* mSim;

	Ogre::SceneManager* mSceneMgr;
	Ogre::Light* sun;

	Ogre::TerrainGlobalOptions* mTerrainGlobals;
	Ogre::TerrainGroup* mTerrainGroup;

	void setupRoad();
	std::vector<class Road*> mRoads;

	void setupObjects();
	std::vector<SceneObject* > mObjs;
};
