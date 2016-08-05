#include "SceneObject.hpp"

#include <OgreSceneManager.h>
#include <OgreEntity.h>

// Shared across all SceneObject subclasses
const std::string SceneObject::PREFIX = "SO_";

// Unique to each subclass
const std::string SceneObject::TYPE = "Generic_";

SceneObject::SceneObject(Ogre::SceneManager *sceneMgr, Ogre::Vector3 pos,
						 Ogre::Quaternion rot, int id, std::string meshFile)
	: mSceneMgr(sceneMgr), mPos(pos), mRot(rot), mId(id), pMeshFile(meshFile) {
	mainNode = mSceneMgr->getRootSceneNode()->createChildSceneNode(getName());
	mainNode->attachObject(mSceneMgr->createEntity(meshFile));

	mainNode->setPosition(mPos); mainNode->setOrientation(mRot);
}

SceneObject::~SceneObject() {
	mSceneMgr->destroySceneNode(mainNode);
}

std::string SceneObject::getName() {
	return PREFIX + TYPE + Ogre::StringConverter::toString(mId);
}
