#pragma once

#include <iostream>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

class Car {
public:
	Car();
	~Car();

	void setup(std::string carName, Ogre::SceneManager* sceneMgr);

	static const int MIN_WHEEL_COUNT = 2;
	static const int MAX_WHEEL_COUNT = 8;

private:
	void setNumWheels(int nw);
	void loadFromConfig();
	void loadModel();

	int numWheels;
	std::string mCarName;
	std::string carPath;

	Ogre::SceneManager* mSceneMgr;

	Ogre::SceneNode* mainNode;
	std::vector<Ogre::SceneNode*> wheelNodes;
	std::vector<Ogre::SceneNode*> brakeNodes;

	std::vector<Ogre::SceneNode*> nodesToDelete;
	void forDeletion(Ogre::SceneNode* node);

	std::vector<Ogre::Entity*> entitiesToDelete;
	void forDeletion(Ogre::Entity* entity);

};
