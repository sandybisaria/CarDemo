#pragma once

#include <OgreSceneNode.h>

class SceneObject {
public:
	SceneObject(Ogre::SceneManager* sceneMgr, Ogre::Vector3 pos,
				Ogre::Quaternion rot, int id, std::string meshFile);
	virtual ~SceneObject();

	virtual void update(float dt) { }

//---- Getter methods
	std::string getName();
	std::string getType() {
		return TYPE.size() > 0 ? TYPE.substr(0, TYPE.size() - 1) : "";
	}

protected:
	const static std::string PREFIX;

	int mId;
	std::string pMeshFile;

	Ogre::SceneManager* mSceneMgr;

	Ogre::SceneNode* mainNode;
	Ogre::Vector3 mPos;
	Ogre::Quaternion mRot;

private:
	const static std::string TYPE;
};
