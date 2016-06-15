#pragma once

#include <OgreSceneManager.h>

#include <Terrain/OgreTerrain.h>
#include <Terrain/OgreTerrainGroup.h>

class Scene {
public:
	Scene(Ogre::SceneManager* sceneMgr);
	~Scene();

	void setupTerrain();
	void configureTerrainDefaults();
	void defineTerrain();
	void initBlendMaps(Ogre::Terrain* terrain);

private:
	Ogre::SceneManager* mSceneMgr;
	Ogre::Light* sun;

	Ogre::TerrainGlobalOptions* mTerrainGlobals;
	Ogre::TerrainGroup* mTerrainGroup;
};
