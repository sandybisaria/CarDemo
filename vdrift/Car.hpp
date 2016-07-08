#pragma once

class CarDynamics;
class CollisionWorld;

#include "CarDynamics.hpp"
#include "../shiny/Main/MaterialInstance.hpp"
#include "CollisionWorld.hpp"
#include "../util/Axes.hpp"

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <vector>

class Car
	: public sh::MaterialInstanceListener {
public:
	Car(int id);
	~Car();

	void setup(std::string carName, Ogre::SceneManager* sceneMgr, CollisionWorld& world);

//	void updatePreviousVelocity();
	void update();
	void handleInputs(const std::vector<double>& inputs);

	// Interface methods
	double getSpeedDir();
	double getSpeedMPS(); // Speed as "measured" internally by the car
	Ogre::Vector3 getPosition() { return mainNode->getPosition(); }
	Ogre::Quaternion getOrientation() { return mainNode->getOrientation(); }
	MathVector<double, 3> getDownVector();
	MathVector<double, 3> getForwardVector();

	// MaterialInstanceListener methods
	virtual void requestedConfiguration(sh::MaterialInstance* m, const std::string& configuration) { }
	virtual void createdConfiguration(sh::MaterialInstance* m, const std::string& configuration);

private:
	bool loadFromConfig(CollisionWorld& world); // Load .car file
	void loadModel(); // Instantiate model nodes and meshes
	void loadMaterials();

	Ogre::Entity* loadPart(std::string partType, int partId = -1);

	void setNumWheels(int nw);
	void changeColor();

	void updateModel(); // Retrieves info from CarDynamics to update model
	void updateLightMap();

	int numWheels;

	std::string mCarName;
	int mId; std::string resGrpId;
	std::string carPath;

	enum eMaterials {mtrCarBody, mtrCarBrake, numMaterials};
	std::string mtrNames[numMaterials];

	CarDynamics* dyn;

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
