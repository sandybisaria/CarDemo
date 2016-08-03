#pragma once

#include "../shiny/Main/MaterialInstance.hpp"

#include "../util/Axes.hpp"

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <vector>

// The Car class combines Stuntrally's CAR and CarModel classes.
// Each instance is responsible for both its Ogre model and its dynamics sim
class Car : public sh::MaterialInstanceListener {
public:
	Car(unsigned int id);
	~Car();

	void setup(std::string carName, Ogre::SceneManager* sceneMgr,
			   class CollisionWorld& world);

	void update();
	void handleInputs(const std::vector<double>& inputs);

//---- Interface methods
	double getSpeedDir(); // Speed in dir the car's facing
	double getSpeedMPS(); // Speed as "measured" internally by the car
	double getSpeed(); // The "actual" speed
	double getMaxAngle() const;
	double getRangeMul() const { return rangeMul; }
	Ogre::Vector3 getPosition() { return mainNode->getPosition(); }
	Ogre::Quaternion getOrientation() { return mainNode->getOrientation(); }
	MathVector<double, 3> getDownVector();
	MathVector<double, 3> getForwardVector();

	void reset();

//---- MaterialInstanceListener methods
	virtual void requestedConfiguration(sh::MaterialInstance* m,
										const std::string& configuration) { }
	virtual void createdConfiguration(sh::MaterialInstance* m,
									  const std::string& configuration);

private:
	bool loadFromConfig(class CollisionWorld& world); // Load .car file
	void loadModel(); // Instantiate model nodes and meshes
	void loadMaterials();

	Ogre::Entity* loadPart(std::string partType, int partId = -1);

	void setNumWheels(int nw);
	void changeColor();

	void updateModel(); // Retrieves info from CarDynamics to update model
	void updateLightMap();

	int numWheels;

	unsigned int mId; // Make sure each car has a unique ID (
	std::string mCarName; // Really the type of car
	std::string resGrpIdStr;
	std::string carPath;

	enum eMaterials {mtrCarBody, mtrCarBrake, numMaterials};
	std::string mtrNames[numMaterials];

	class CarDynamics* dyn;
	class CollisionWorld* cw;

	MathVector<double, 3> initPos;
	Quaternion<double> initRot;

	const double rangeMul;

	Ogre::SceneManager* mSceneMgr;

	Ogre::SceneNode* mainNode;
	std::vector<Ogre::SceneNode*> wheelNodes;
	//TODO Brake nodes currently disabled because the CAD model does not have them.
	// It would be nice to support both models that have and that don't have them
//	std::vector<Ogre::SceneNode*> brakeNodes;

	std::vector<Ogre::SceneNode*> nodesToDelete;
	void forDeletion(Ogre::SceneNode* node) { nodesToDelete.push_back(node); }
	std::vector<Ogre::Entity*> entitiesToDelete;
	void forDeletion(Ogre::Entity* entity) { entitiesToDelete.push_back(entity); }

	Ogre::ColourValue carColor;
};
