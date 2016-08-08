#include "SceneObject.hpp"

#include <OgreEntity.h>

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
//	Ogre::Light* redLight = mSceneMgr->createLight(getName() + "_LIGHT_RED");
//	redLight->setType(Ogre::Light::LT_SPOTLIGHT);
//	redLight->setDiffuseColour(Ogre::ColourValue::Red);
//	redLight->setSpecularColour(1.f, 0.5f, 0.f);
//
//	redLight->setPosition(0, 0, 10);
//	std::cout << redLight->getPosition() << std::endl;

//	redLightNode = mainNode->createChildSceneNode(getName() + "_NODE_RED");
//	redLightNode->attachObject(redLight);
}

//TrafficLight::~TrafficLight() {
//	mSceneMgr->destroySceneNode(mainNode);
//	mSceneMgr->destroySceneNode(redLightNode);
//}