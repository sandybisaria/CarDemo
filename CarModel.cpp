#include "CarModel.hpp"

#include "vdrift/cardefs.h"

CarModel::CarModel(int id, std::string carModelName, Ogre::SceneManager* sceneMgr)
	: mId(id), mCarModelName(carModelName),
	  mSceneMgr(sceneMgr), mCar(0) {
	setNumWheels(DEF_WHEELS);

	//TODO Implement CarModel::Defaults?
}

CarModel::~CarModel() {

}

void CarModel::load() {

}

void CarModel::create() {
	// To identify this CarModel's resources
	std::string resGrpId = mCarModelName + Ogre::StringConverter::toString(mId);

	// Location of car resources
	std::string carResPath = "../data/cars/" + mCarModelName;

	Ogre::ResourceGroupManager::getSingleton().createResourceGroup(resGrpId);
	Ogre::Root::getSingleton().addResourceLocation(carResPath + "/mesh", "FileSystem", resGrpId);
	Ogre::Root::getSingleton().addResourceLocation(carResPath + "/textures", "FileSystem", resGrpId);
}

void CarModel::setNumWheels(int n) {
	numWheels = n;
	wheelPos.resize(n); wheelRadius.resize(n); wheelWidth.resize(n);
	//TODO whTrail and whTemp, for when implementing trails/particles
	wheelNodes.resize(n); brakeNodes.resize(n); //TODO wheelEmitNodes?
}
