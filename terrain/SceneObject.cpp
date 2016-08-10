#include "SceneObject.hpp"

#include <OgreEntity.h>
#include <OgreSubEntity.h>

//---- Shared across all SceneObject subclasses
const std::string SceneObject::PREFIX = "SO_";

//---- SceneObject definitions
SceneObject::SceneObject(Ogre::SceneManager *sceneMgr, Ogre::Vector3 pos,
						 Ogre::Quaternion rot, int id, std::string meshFile)
	: mSceneMgr(sceneMgr), mPos(pos), mRot(rot), mId(id), pMeshFile(meshFile) {
	mainNode = mSceneMgr->getRootSceneNode()->createChildSceneNode(getName());
	mainNode->attachObject(mSceneMgr->createEntity(meshFile));

	mainNode->setPosition(mPos); mainNode->setOrientation(mRot);

	mainNode->getUserObjectBindings().setUserAny(Ogre::Any(this));
}

SceneObject::~SceneObject() {
	mSceneMgr->destroySceneNode(mainNode);
}

std::string SceneObject::getName() {
	return PREFIX + getType() + "_" + Ogre::StringConverter::toString(mId);
}

//---- StopSign definitions
StopSign::StopSign(Ogre::SceneManager *sceneMgr, Ogre::Vector3 pos,
				   Ogre::Quaternion rot, int id)
	: SceneObject(sceneMgr, pos, rot, id, "StopSign.mesh") {
}

//---- TrafficLight definitions
TrafficLight::TrafficLight(Ogre::SceneManager *sceneMgr, Ogre::Vector3 pos,
						   Ogre::Quaternion rot, int id)
	: SceneObject(sceneMgr, pos, rot, id, "TrafficLight.mesh") {
	preloadTextures();

	trafficLightEntity = ((Ogre::Entity*) mainNode->getAttachedObject(0));

	// Initial state is green (though could be configurable?)
	tlStatus = TL_RED;
	changeState();

	//TODO Replace with more realistic times (25 - 3 - 32)?
	waitTimes[TL_GREEN] = 5;
	waitTimes[TL_YELLOW] = 3;
	waitTimes[TL_RED] = 12;
}

void TrafficLight::update(float dt) {
	timeWaiting += dt;
	if (timeWaiting >= waitTimes[tlStatus]) { changeState(); }
}

void TrafficLight::changeState() {
	std::string nextMaterial = "TrafficLight_Front_";
	switch (tlStatus) {
	case TL_GREEN:
		tlStatus = TL_YELLOW;
		nextMaterial += "Yellow";
		break;
	case TL_YELLOW:
		tlStatus = TL_RED;
		nextMaterial += "Red";
		break;
	case TL_RED:
		tlStatus = TL_GREEN;
		nextMaterial += "Green";
		break;
	}

	//Note: SubEntity idx hard-coded based on the current mesh!
	trafficLightEntity->getSubEntity(2)->setMaterialName(nextMaterial);

	timeWaiting = 0;
}

bool TrafficLight::mLoaded = false;
void TrafficLight::preloadTextures() {
	if (mLoaded) return;
	mLoaded = true;

	Ogre::MaterialManager::getSingleton().load("TrafficLight_Front_Red",
											   Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
	Ogre::MaterialManager::getSingleton().load("TrafficLight_Front_Yellow",
											   Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
	Ogre::MaterialManager::getSingleton().load("TrafficLight_Front_Green",
											   Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
}