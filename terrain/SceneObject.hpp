#pragma once

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

// Generic SceneObject class
class SceneObject {
public:
	SceneObject(Ogre::SceneManager* sceneMgr, Ogre::Vector3 pos,
				Ogre::Quaternion rot, int id, std::string meshFile);
	virtual ~SceneObject();

	virtual void update(float dt) { }

	std::string getName();
	virtual std::string getType() { return "GENERIC"; }

protected:
	const static std::string PREFIX;

	int mId;
	std::string pMeshFile;

	Ogre::SceneManager* mSceneMgr;

	Ogre::SceneNode* mainNode;
	Ogre::Vector3 mPos;
	Ogre::Quaternion mRot;
};

// Stop sign
class StopSign : public SceneObject {
public:
	StopSign(Ogre::SceneManager* sceneMgr, Ogre::Vector3 pos,
			 Ogre::Quaternion rot, int id);

	virtual std::string getType() { return "StopSign"; }
};