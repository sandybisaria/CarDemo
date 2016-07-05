#include "Scene.hpp"

Scene::Scene(Ogre::SceneManager* sceneMgr)
	: mSim(NULL), mSceneMgr(sceneMgr), sun(0),
	  mTerrainGlobals(0), mTerrainGroup(0) {
	terrSize = 513; worldSize = 10000.0;
}

Scene::~Scene() {
	OGRE_DELETE mTerrainGlobals;
	OGRE_DELETE mTerrainGroup;
}

void Scene::setupTerrain(Sim* sim) {
	mSim = sim;
	mSim->scene = this;

	mSceneMgr->setAmbientLight(Ogre::ColourValue(0.4, 0.4, 0.4));

	Ogre::Vector3 lightDir(0.55, -0.3, 0.75);
	lightDir.normalise();

	sun = mSceneMgr->createLight("SunLight");
	sun->setType(Ogre::Light::LT_DIRECTIONAL);
	sun->setDirection(lightDir);
	sun->setDiffuseColour(Ogre::ColourValue::White);
	sun->setSpecularColour(Ogre::ColourValue(0.4, 0.4, 0.4));

	mTerrainGlobals = OGRE_NEW Ogre::TerrainGlobalOptions();

	mTerrainGroup = OGRE_NEW Ogre::TerrainGroup(mSceneMgr, Ogre::Terrain::ALIGN_X_Z, terrSize, worldSize);

	configureTerrainDefaults();
	defineTerrain();

	mTerrainGroup->loadAllTerrains(true);

	mTerrainGroup->freeTemporaryResources();

	mSceneMgr->setSkyDome(true, "CloudySky");

	createBulletTerrain();
}

void Scene::configureTerrainDefaults() {
	mTerrainGlobals->setMaxPixelError(8);
	mTerrainGlobals->setCompositeMapDistance(3000);

	mTerrainGlobals->setLightMapDirection(sun->getDerivedDirection());
	mTerrainGlobals->setCompositeMapAmbient(mSceneMgr->getAmbientLight());
	mTerrainGlobals->setCompositeMapDiffuse(sun->getDiffuseColour());

	Ogre::Terrain::ImportData& importData = mTerrainGroup->getDefaultImportSettings();
	importData.terrainSize = terrSize;
	importData.worldSize = worldSize;
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

void Scene::createBulletTerrain() {
	Ogre::Terrain* baseTerrain = mTerrainGroup->getTerrain(0, 0);
	btHeightfieldTerrainShape* hfShape = new btHeightfieldTerrainShape(
			baseTerrain->getSize(), baseTerrain->getSize(),
			baseTerrain->getHeightData(), 1, 0, 0,
			2, PHY_FLOAT, false);
	// Min and max height were set to 0 to force the terrain to be completely flat
	hfShape->setUseDiamondSubdivision(true);

//	float metersBetweenVerts = baseTerrain->getWorldSize() / (baseTerrain->getSize() - 1);
	btVector3 scale(1, 1, 1); //FIXME Just using what Stuntrally does
	hfShape->setLocalScaling(scale);
	hfShape->setUserPointer((void*) SU_Terrain);

	btCollisionObject* colObj = new btCollisionObject();
	colObj->setCollisionShape(hfShape);
	colObj->setFriction(0.9); colObj->setRestitution(0.0);
	colObj->setCollisionFlags(colObj->getCollisionFlags() |
		btCollisionObject::CF_STATIC_OBJECT | btCollisionObject::CF_DISABLE_VISUALIZE_OBJECT);
	mSim->getCollisionWorld()->getDynamicsWorld()->addCollisionObject(colObj);
	mSim->getCollisionWorld()->addShape(hfShape);

	// Border planes
	const float px[4] = {-1, 1, 0, 0};
	const float py[4] = { 0, 0,-1, 1};
	for (int i = 0; i < 4; ++i) {
		btVector3 vpl(px[i], py[i], 0);
		btCollisionShape* shp = new btStaticPlaneShape(vpl, 0);
		shp->setUserPointer((void*) SU_Border);

		btTransform tr;  tr.setIdentity();
		tr.setOrigin(vpl * -0.5 * worldSize);

		btCollisionObject* col = new btCollisionObject();
		col->setCollisionShape(shp);
		col->setWorldTransform(tr);
		col->setFriction(0.3);   //+
		col->setRestitution(0.0);
		col->setCollisionFlags(col->getCollisionFlags() |
			btCollisionObject::CF_STATIC_OBJECT | btCollisionObject::CF_DISABLE_VISUALIZE_OBJECT/**/);

		mSim->getCollisionWorld()->getDynamicsWorld()->addCollisionObject(col);
		mSim->getCollisionWorld()->addShape(shp);
	}
}
