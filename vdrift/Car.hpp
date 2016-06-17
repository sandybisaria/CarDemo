#pragma once

#include "CarPosInfo.hpp"
#include "CarDynamics.hpp"
#include "../shiny/Main/MaterialInstance.hpp"

#include <iostream>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

class Car
	: public sh::MaterialInstanceListener {
public:
	Car();
	~Car();

	void setup(std::string carName, Ogre::SceneManager* sceneMgr);

	void update(float dt);

	virtual void requestedConfiguration(sh::MaterialInstance* m, const std::string& configuration);
	virtual void createdConfiguration(sh::MaterialInstance* m, const std::string& configuration);

private:
	bool loadFromConfig();
	void loadModel();
	void loadMaterials();

	void setNumWheels(int nw);
	void changeColor();

	void updateLightMap();

	int numWheels;
	std::string mCarName;
	std::string carPath;

	enum eMaterials {mtrCarBody, mtrCarBrake, numMaterials};
	std::string mtrNames[numMaterials];

	CarPosInfo info;
	CarDynamics dyn;

	Ogre::SceneManager* mSceneMgr;

	Ogre::SceneNode* mainNode;
	std::vector<Ogre::SceneNode*> wheelNodes;
	std::vector<Ogre::SceneNode*> brakeNodes;

	std::vector<Ogre::SceneNode*> nodesToDelete;
	void forDeletion(Ogre::SceneNode* node);
	std::vector<Ogre::Entity*> entitiesToDelete;
	void forDeletion(Ogre::Entity* entity);

	Ogre::ColourValue carColor;
};
