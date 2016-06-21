#pragma once

#include "Sim.hpp"
class Sim;

#include "vdrift/Car.hpp"
class Car;
#include "vdrift/mathvector.h"
#include "vdrift/configfile.h"

#include "shiny/Main/MaterialInstance.hpp"
#include "shiny/Main/Factory.hpp"

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreRoot.h>
#include <OgreQuaternion.h>
#include <OgreEntity.h>
#include <OgreMeshManager.h>
#include <OgreMaterialManager.h>

#include <string>
#include <vector>

// "Ogre" part of the car, mostly handling rendering/visualization
// Based on Stuntrally's CarModel class
class CarModel
	: public sh::MaterialInstanceListener {
public:
	CarModel(int id, std::string carModelName, Ogre::SceneManager* sceneMgr, Sim* sim);
	~CarModel();

	void setModelDefaults();

	int mId; // To uniquely identify the CarModel instance (and match with corresponding Car)
	std::string mCarModelName; // Car model identifier
	std::string resGrpId; // Identifier for car model's resource group
	std::string carResPath; // Top-level directory for car resources
	std::string mtrId; // Identifier for car materials

	// Creation
	void load();
	bool loadConfig(); // Load params from .car; return true on success

	void create(); // Create SceneNodes for car mesh
	void recreateMaterials(); // Load car materials

	Sim* mSim;

	Car* mCar; // The vdrift Car, constructed during load()

	int numWheels;
	void setNumWheels(int n);

	// Model params from .car config
	float interiorOffset[CONF_VEC_SIZE];

	std::vector<MATHVECTOR<float, 3> > wheelPos;
	std::vector<float> wheelRadius, wheelWidth;

	bool rotationFix; // Whether to fix the rotation of certain objects

	Ogre::SceneManager* mSceneMgr;
	Ogre::SceneNode* mMainNode;

	// Material names
	enum materialTypes { carBodyMtr, carBrakeMtr, NUM_MTRS };
	std::string materialNames[NUM_MTRS];

	// For manual deletion
	std::vector<Ogre::SceneNode*> nodesToDel;
	void forDel(Ogre::SceneNode* node) { nodesToDel.push_back(node); }
	std::vector<Ogre::Entity*> entsToDel;
	void forDel(Ogre::Entity* ent) { entsToDel.push_back(ent); }

	void destroyOgreObjects();

	// Wheel nodes
	std::vector<Ogre::SceneNode*> wheelNodes, brakeNodes;

	// Color
	void changeColor();
	Ogre::ColourValue carColor; //TODO Make configurable

	// Lightmap
	void updateLightMap();

	// MaterialInstanceListener methods
	virtual void requestedConfiguration(sh::MaterialInstance* m, const std::string& configuration);
	virtual void createdConfiguration(sh::MaterialInstance* m, const std::string& configuration);
};
