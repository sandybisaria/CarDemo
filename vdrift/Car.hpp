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
	Car(int id);
	~Car();

	void setup(std::string carName, Ogre::SceneManager* sceneMgr);

	void update(float dt);

	virtual void requestedConfiguration(sh::MaterialInstance* m, const std::string& configuration);
	virtual void createdConfiguration(sh::MaterialInstance* m, const std::string& configuration);

private:
	bool loadFromConfig();
	void loadModel();
	void loadMaterials();

	Ogre::Entity* loadPart(std::string partType, int partId = -1);

	void setNumWheels(int nw);
	void changeColor();

	void updateLightMap();

	int numWheels;

	std::string mCarName;
	int mId; std::string resGrpId;
	std::string carPath;

	enum eMaterials {mtrCarBody, mtrCarBrake, numMaterials};
	std::string mtrNames[numMaterials];

	CarDynamics dyn;

	Ogre::SceneManager* mSceneMgr;

	Ogre::SceneNode* mainNode;
	std::vector<Ogre::SceneNode*> wheelNodes;
	std::vector<Ogre::SceneNode*> brakeNodes;

	std::vector<Ogre::SceneNode*> nodesToDelete;
	void forDeletion(Ogre::SceneNode* node) { nodesToDelete.push_back(node); }
	std::vector<Ogre::Entity*> entitiesToDelete;
	void forDeletion(Ogre::Entity* entity) { entitiesToDelete.push_back(entity); }

	Ogre::ColourValue carColor;
};
