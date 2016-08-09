#pragma once

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

//---- Generic SceneObject class
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

//---- Stop sign
class StopSign : public SceneObject {
public:
	StopSign(Ogre::SceneManager* sceneMgr, Ogre::Vector3 pos,
			 Ogre::Quaternion rot, int id);

	virtual std::string getType() { return "StopSign"; }
};

//---- Traffic light
enum TrafficLightState { TL_GREEN = 0, TL_YELLOW, TL_RED };

class TrafficLight : public SceneObject {
public:
	TrafficLight(Ogre::SceneManager* sceneMgr, Ogre::Vector3 pos,
				 Ogre::Quaternion rot, int id);

	virtual void update(float dt);

	virtual std::string getType() { return "TrafficLight"; }
	TrafficLightState getState() const { return tlState; }

private:
	Ogre::Entity* trafficLightEntity;

	TrafficLightState tlState;
	void changeState();

	double waitTimes[3];
	double timeWaiting;
};