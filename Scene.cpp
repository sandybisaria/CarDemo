#include "Scene.hpp"

Scene::Scene(Ogre::SceneManager* sceneMgr)
	: mSceneMgr(sceneMgr), sun(0),
	  mTerrainGlobals(0), mTerrainGroup(0) {
}

Scene::~Scene() {
	OGRE_DELETE mTerrainGlobals;
}

void Scene::setupTerrain() {
	mSceneMgr->setAmbientLight(Ogre::ColourValue(0.2, 0.2, 0.2));

	Ogre::Vector3 lightDir(0.55, -0.3, 0.75);
	lightDir.normalise();

	sun = mSceneMgr->createLight("SunLight");
	sun->setType(Ogre::Light::LT_DIRECTIONAL);
	sun->setDirection(lightDir);
	sun->setDiffuseColour(Ogre::ColourValue::White);
	sun->setSpecularColour(Ogre::ColourValue(0.4, 0.4, 0.4));

	mTerrainGlobals = OGRE_NEW Ogre::TerrainGlobalOptions();

	mTerrainGroup = OGRE_NEW Ogre::TerrainGroup(mSceneMgr,
			Ogre::Terrain::ALIGN_X_Z, 513, 12000.0);

	configureTerrainDefaults();
	defineTerrain();

	mTerrainGroup->loadAllTerrains(true);

	Ogre::TerrainGroup::TerrainIterator ti = mTerrainGroup->getTerrainIterator();
	while (ti.hasMoreElements()) {
		Ogre::Terrain* t = ti.getNext()->instance;
		initBlendMaps(t);
	}

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
	importData.worldSize = 12000.0;
	importData.inputScale = 600;
	importData.minBatchSize = 33;
	importData.maxBatchSize = 65;

	importData.layerList.resize(3);
	importData.layerList[0].worldSize = 100;
	importData.layerList[0].textureNames.push_back("dirt_grayrocky_diffusespecular.dds");
	importData.layerList[0].textureNames.push_back("dirt_grayrocky_normalheight.dds");
	importData.layerList[1].worldSize = 30;
	importData.layerList[1].textureNames.push_back("grass_green-01_diffusespecular.dds");
	importData.layerList[1].textureNames.push_back("grass_green-01_normalheight.dds");
	importData.layerList[2].worldSize = 200;
	importData.layerList[2].textureNames.push_back("growth_weirdfungus-03_diffusespecular.dds");
	importData.layerList[2].textureNames.push_back("growth_weirdfungus-03_normalheight.dds");
}

void Scene::defineTerrain() {
	Ogre::Image img;
	img.load("terrain.png", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
	mTerrainGroup->defineTerrain(0, 0, &img);
}

void Scene::initBlendMaps(Ogre::Terrain* terrain) {
	Ogre::Real minHeight0 = 70;
	Ogre::Real fadeDist0 = 40;
	Ogre::Real minHeight1 = 70;
	Ogre::Real fadeDist1 = 15;

	Ogre::TerrainLayerBlendMap* blendMap0 = terrain->getLayerBlendMap(1);
	Ogre::TerrainLayerBlendMap* blendMap1 = terrain->getLayerBlendMap(2);

	float* pBlend0 = blendMap0->getBlendPointer();
	float* pBlend1 = blendMap1->getBlendPointer();

	for (Ogre::uint16 y = 0; y < terrain->getLayerBlendMapSize(); ++y) {
	  for (Ogre::uint16 x = 0; x < terrain->getLayerBlendMapSize(); ++x) {
		Ogre::Real tx, ty;

		blendMap0->convertImageToTerrainSpace(x, y, &tx, &ty);
		Ogre::Real height = terrain->getHeightAtTerrainPosition(tx, ty);
		Ogre::Real val = (height - minHeight0) / fadeDist0;
		val = Ogre::Math::Clamp(val, (Ogre::Real)0, (Ogre::Real)1);
		*pBlend0++ = val;

		val = (height - minHeight1) / fadeDist1;
		val = Ogre::Math::Clamp(val, (Ogre::Real)0, (Ogre::Real)1);
		*pBlend1++ = val;
	  }
	}

	blendMap0->dirty();
	blendMap1->dirty();
	blendMap0->update();
	blendMap1->update();
}
