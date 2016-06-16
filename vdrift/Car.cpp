#include "Car.hpp"

#include "../util/ConfigFile.hpp"

#include <OgreRoot.h>
#include <OgreEntity.h>

Car::Car()
	: mSceneMgr(0), mainNode(0) {
	setNumWheels(4); // Default to four wheels
}

Car::~Car() {
	// Destroy any created nodes, etc.
	for (int i = 0; i < nodesToDelete.size(); i++) {
		mSceneMgr->destroySceneNode(nodesToDelete[i]);
	}
	nodesToDelete.clear();

	for (int i = 0; i < entitiesToDelete.size(); i++) {
		mSceneMgr->destroyEntity(entitiesToDelete[i]);
	}
	entitiesToDelete.clear();
}

void Car::setup(std::string carName, Ogre::SceneManager* sceneMgr) {
	mCarName = carName;
	carPath = "../data/cars/" + mCarName;

	loadFromConfig();
	//TODO Determine/set starting position and rotation

	mSceneMgr = sceneMgr;
	loadModel();
}

void Car::setNumWheels(int nw) {
	numWheels = nw;
	wheelNodes.resize(numWheels);
	brakeNodes.resize(numWheels);

	/*TODO Keep track of the following in vectors:
	 * Wheel position/radius/width/trail/temperature
	 * SceneNodes* for "Wh" (Wheel), "WhE" (Wheel Emitter), "Brake"
	 * Suspension bump detection, "Last bezier patch that each wheel hit"
	 */
}

void Car::loadFromConfig() {
	// Top-level directory for all car data
	std::string carSimPath = carPath + "/sim/" + mCarName + ".car";

	ConfigFile cf;
	if (!cf.load(carSimPath)) {
		return; //TODO Error if car not found
	}

	int nw = 0;
	cf.getParam("wheels", nw);
	if (nw >= MIN_WHEEL_COUNT && nw <= MAX_WHEEL_COUNT)
		setNumWheels(nw);

	/*TODO Get vehicle type (AWD, FWD, etc.)
	 * Get params for model offsets, positions, etc.
	 *
	 * Refer to CarModel::LoadConfig
	 */

	/*TODO Load physical properties (CARDYNAMICS)
	 *
	 * Refer to CAR::Load and CARDYNAMICS::Load
	 */
}

void Car::loadModel() {
	// Each car has its own resource group
	Ogre::ResourceGroupManager::getSingleton().createResourceGroup(mCarName);
	Ogre::Root::getSingleton().addResourceLocation(carPath + "/mesh", "FileSystem", mCarName);
	Ogre::Root::getSingleton().addResourceLocation(carPath + "/textures", "FileSystem", mCarName);
	Ogre::ResourceGroupManager::getSingleton().initialiseResourceGroup(mCarName);

	mainNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	forDeletion(mainNode);
	mainNode->setPosition(Ogre::Vector3(0, 80, 20));

	Ogre::SceneNode* carNode = mainNode->createChildSceneNode();
	forDeletion(carNode);

	//TODO Allow for camera to follow car (using FollowCamera class)?
	//TODO Create car reflection (using CarReflection class)?

	//FIXME Load the materials!

	// Create car body
	std::string bodyMesh = mCarName + "_body";
	Ogre::Entity* body = mSceneMgr->createEntity(bodyMesh, bodyMesh + ".mesh", mCarName);
	forDeletion(body);
	carNode->attachObject(body);

	Ogre::AxisAlignedBox bodyBox = body->getBoundingBox();

	// Create interior
	std::string interiorMesh = mCarName + "_interior";
	Ogre::Entity* interior = mSceneMgr->createEntity(interiorMesh, interiorMesh + ".mesh", mCarName);
	forDeletion(interior);
	carNode->attachObject(interior);

	// Create glass
	std::string glassMesh = mCarName + "_glass";
	Ogre::Entity* glass = mSceneMgr->createEntity(glassMesh, glassMesh + ".mesh", mCarName);
	forDeletion(glass);
	carNode->attachObject(glass);

	// Create wheels and brakes
	for (int w = 0; w < numWheels; w++) {
		// Wheels
		wheelNodes[w] = mainNode->createChildSceneNode();
		forDeletion(wheelNodes[w]);
		std::string wheelMesh = mCarName + "_wheel";
		//TODO Support vehicles with specific wheel meshes (i.e. wheel_front,...)
		Ogre::Entity* wheel = mSceneMgr->createEntity(wheelMesh + Ogre::StringConverter::toString(w),
													  wheelMesh + ".mesh", mCarName);
		forDeletion(wheel);
		wheelNodes[w]->attachObject(wheel);

		// Brakes
		brakeNodes[w] = mainNode->createChildSceneNode();
		forDeletion(brakeNodes[w]);
		std::string brakeMesh = mCarName + "_brake";
		Ogre::Entity* brake = mSceneMgr->createEntity(brakeMesh + Ogre::StringConverter::toString(w),
													  brakeMesh + ".mesh", mCarName);
		forDeletion(brake);
		brakeNodes[w]->attachObject(brake);
	}
}

void Car::forDeletion(Ogre::SceneNode* node) {
	nodesToDelete.push_back(node);
}

void Car::forDeletion(Ogre::Entity* entity) {
	entitiesToDelete.push_back(entity);
}
