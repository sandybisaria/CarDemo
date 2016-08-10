#include "Scene.hpp"

#include "RenderConst.hpp"

#include "../Sim.hpp"

#include "../road/Road.hpp"

#include "../vdrift/CollisionWorld.hpp"

#include <OgreEntity.h>

Scene::Scene(Ogre::SceneManager* sceneMgr)
	: mSim(0), mSceneMgr(sceneMgr), sun(0),
	  mTerrainGlobals(0), mTerrainGroup(0) {
	terrSize = 513; worldSize = 10000.0; //TODO Retrieve from a config?
}

Scene::~Scene() {
	OGRE_DELETE mTerrainGlobals;
	OGRE_DELETE mTerrainGroup;

	// Delete roads
	std::vector<Road*>::iterator i = mRoads.begin();
	for (; i != mRoads.end(); i++) {
		(*i)->destroy();
	}
	mRoads.clear();
}

void Scene::setup(Sim* sim) {
	mSim = sim;
	mSim->scene = this;

	// Lighting and sky
	mSceneMgr->setAmbientLight(Ogre::ColourValue(0.4, 0.4, 0.4));

	Ogre::Vector3 lightDir(0.55, -0.3f, 0.75);
	lightDir.normalise();

	sun = mSceneMgr->createLight("SunLight");
	sun->setType(Ogre::Light::LT_DIRECTIONAL);
	sun->setDirection(lightDir);
	sun->setDiffuseColour(Ogre::ColourValue::White);
	sun->setSpecularColour(Ogre::ColourValue(0.4, 0.4, 0.4));

	mSceneMgr->setSkyDome(true, "CloudySky");

	// Terrain
	mTerrainGlobals = OGRE_NEW Ogre::TerrainGlobalOptions();
	mTerrainGroup = OGRE_NEW Ogre::TerrainGroup(mSceneMgr, Ogre::Terrain::ALIGN_X_Z,
												terrSize, worldSize);

	configureTerrainDefaults();
	defineTerrain();

	mTerrainGroup->loadAllTerrains(true);
	Ogre::TerrainGroup::TerrainIterator ti = mTerrainGroup->getTerrainIterator();
	while (ti.hasMoreElements()) {
		Ogre::Terrain* t = ti.getNext()->instance;
		t->setVisibilityFlags(RV_Terrain);
	}
	mTerrainGroup->freeTemporaryResources();

	createBulletTerrain();

	// Road
//	setupRoad();

	// Objects
	setupObjects();
}

//---- Terrain methods
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
	//TODO Determine the terrain textures from a config
	importData.layerList[0].textureNames.push_back("asphalt_ds.dds");
	importData.layerList[0].textureNames.push_back("asphalt_nh.dds");
}

void Scene::defineTerrain() {
	Ogre::Image img;
	//TODO Determine the terrain heightmap from a config
	img.load("terrain.png", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
	mTerrainGroup->defineTerrain(0, 0, &img);
}

void Scene::createBulletTerrain() {
	Ogre::Terrain* baseTerrain = mTerrainGroup->getTerrain(0, 0);
	btHeightfieldTerrainShape* hfShape = new btHeightfieldTerrainShape(
			baseTerrain->getSize(), baseTerrain->getSize(), baseTerrain->getHeightData(), 1, 0, 0,
			2, PHY_FLOAT, false);
	// Min and max height were set to 0 to force the terrain to be completely flat
	hfShape->setUseDiamondSubdivision(true);

	// This scale is based on the tutorial, but it may not be true...
	float metersBetweenVerts = baseTerrain->getWorldSize() / (baseTerrain->getSize() - 1);
	btVector3 scale(metersBetweenVerts, metersBetweenVerts, 1);
	hfShape->setLocalScaling(scale);

	hfShape->setUserPointer((void*) SU_Terrain);

	btCollisionObject* colObj = new btCollisionObject();
	colObj->setCollisionShape(hfShape);
	colObj->setFriction(0.9); colObj->setRestitution(0.0);
	colObj->setCollisionFlags(colObj->getCollisionFlags() |
							  btCollisionObject::CF_STATIC_OBJECT |
							  btCollisionObject::CF_DISABLE_VISUALIZE_OBJECT);
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
							   btCollisionObject::CF_STATIC_OBJECT |
							   btCollisionObject::CF_DISABLE_VISUALIZE_OBJECT);

		mSim->getCollisionWorld()->getDynamicsWorld()->addCollisionObject(col);
		mSim->getCollisionWorld()->addShape(shp);
	}
}

//---- Road methods
void Scene::setupRoad() {
	Ogre::Terrain* baseTerrain = mTerrainGroup->getTerrain(0, 0);

	//TODO Different road XML files for different scenes
	Ogre::String roadsFile = "../data/scene/roads.xml";
	tinyxml2::XMLDocument doc;
	tinyxml2::XMLError e = doc.LoadFile(roadsFile.c_str());
	if (e != tinyxml2::XML_SUCCESS) { return; }

	tinyxml2::XMLElement* root = doc.RootElement();
	if (!root) { return; }

	tinyxml2::XMLElement* roadXML = root->FirstChildElement("Road");
	while (roadXML) {
		Road* road = new Road(mSim);
		road->setup(baseTerrain, mSceneMgr);

		if (!road->loadFromXML(roadXML)) {
			std::cout << "Road failed to load" << std::endl;
			continue;
		}

		// PSSM materials not included...

		road->rebuildRoadGeometry();

		mRoads.push_back(road);

		roadXML = roadXML->NextSiblingElement("Road");
	}
}

//---- Objects methods
void Scene::setupObjects() {
	Ogre::Vector3 pos(100, 0, 0);
	Ogre::Quaternion rot(Ogre::Radian(M_PI), Ogre::Vector3::UNIT_Y);
	StopSign* stopSign = new StopSign(mSceneMgr, pos, rot, 0);
	mObjs.push_back(stopSign);

	pos = Ogre::Vector3(200, 0, 0);
	stopSign = new StopSign(mSceneMgr, pos, rot, 1);
	mObjs.push_back(stopSign);

	pos = Ogre::Vector3(150, 0, 0);
	rot = Ogre::Quaternion(Ogre::Radian(M_PI_2), Ogre::Vector3::UNIT_Y);
	TrafficLight* trafficLight = new TrafficLight(mSceneMgr, pos, rot, 2);
	mObjs.push_back(trafficLight);

	pos = Ogre::Vector3(250, 0, 0);
	rot = Ogre::Quaternion(Ogre::Radian(M_PI_2), Ogre::Vector3::UNIT_Y);
	trafficLight = new TrafficLight(mSceneMgr, pos, rot, 3);
	mObjs.push_back(trafficLight);
}

//---- Update methods
void Scene::update(float dt) {
	std::vector<Road*>::const_iterator i;
	for (i = mRoads.begin(); i != mRoads.end(); i++) {
		(*i)->update();
	}

	std::vector<SceneObject*>::const_iterator j;
	for (j = mObjs.begin(); j != mObjs.end(); j++) {
		(*j)->update(dt);
	}
}