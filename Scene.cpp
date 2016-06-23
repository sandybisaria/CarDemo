#include "Scene.hpp"

Scene::Scene(Ogre::SceneManager* sceneMgr)
	: mSceneMgr(sceneMgr), sun(0),
	  mTerrainGlobals(0), mTerrainGroup(0) {
}

Scene::~Scene() {
	OGRE_DELETE mTerrainGlobals;
	OGRE_DELETE mTerrainGroup;
}

void Scene::setupTerrain() {
	mSceneMgr->setAmbientLight(Ogre::ColourValue(0.4, 0.4, 0.4));

	Ogre::Vector3 lightDir(0.55, -0.3, 0.75);
	lightDir.normalise();

	sun = mSceneMgr->createLight("SunLight");
	sun->setType(Ogre::Light::LT_DIRECTIONAL);
	sun->setDirection(lightDir);
	sun->setDiffuseColour(Ogre::ColourValue::White);
	sun->setSpecularColour(Ogre::ColourValue(0.4, 0.4, 0.4));

	mTerrainGlobals = OGRE_NEW Ogre::TerrainGlobalOptions();

	mTerrainGroup = OGRE_NEW Ogre::TerrainGroup(mSceneMgr,
			Ogre::Terrain::ALIGN_X_Z, 513, 10000.0); //TODO Store world size somewhere!

	configureTerrainDefaults();
	defineTerrain();

	mTerrainGroup->loadAllTerrains(true);

	mTerrainGroup->freeTemporaryResources();

	mSceneMgr->setSkyDome(true, "CloudySky");
}

void Scene::configureTerrainDefaults() {
	mTerrainGlobals->setMaxPixelError(8);
	mTerrainGlobals->setCompositeMapDistance(3000);

	mTerrainGlobals->setLightMapDirection(sun->getDerivedDirection());
	mTerrainGlobals->setCompositeMapAmbient(mSceneMgr->getAmbientLight());
	mTerrainGlobals->setCompositeMapDiffuse(sun->getDiffuseColour());

	Ogre::Terrain::ImportData& importData = mTerrainGroup->getDefaultImportSettings();
	importData.terrainSize = 513;
	importData.worldSize = 10000.0;
	importData.inputScale = 600;
	importData.minBatchSize = 33;
	importData.maxBatchSize = 65;

	importData.layerList.resize(1);
	importData.layerList[0].worldSize = 100;
	importData.layerList[0].textureNames.push_back("asphalt_ds.dds");
	importData.layerList[0].textureNames.push_back("asphalt_nh.dds");
}

void Scene::defineTerrain() {
	Ogre::Image img;
	img.load("terrain.png", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
	mTerrainGroup->defineTerrain(0, 0, &img);
}
