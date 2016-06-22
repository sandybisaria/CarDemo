#include "Car.hpp"

#include "CarConstants.hpp"
#include "../shiny/Main/Factory.hpp"
#include "../util/ConfigFile.hpp"

#include <OgreRoot.h>
#include <OgreEntity.h>

Car::Car(int id)
	: mId(id), mSceneMgr(0), mainNode(0),
	  carColor(0, 1, 0) {
	setNumWheels(DEF_WHEEL_COUNT);
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
	resGrpId = mCarName + "_" + Ogre::StringConverter::toString(mId);
	carPath = "../data/cars/" + mCarName;

	if (!loadFromConfig()){
		return; //TODO With error
	}

	mSceneMgr = sceneMgr;
	loadModel();

	//TODO Load starting position/rotation from somewhere...
}

void Car::update(float dt) {
	//TODO Update CarDynamics, then update model

	updateLightMap();
}

void Car::requestedConfiguration(sh::MaterialInstance* m, const std::string& configuration) {

}

void Car::createdConfiguration(sh::MaterialInstance* m, const std::string& configuration) {
	changeColor();
	updateLightMap();
}

bool Car::loadFromConfig() {
	// Top-level directory for all car data
	std::string carSimPath = carPath + "/sim/" + mCarName + ".car";

	ConfigFile cf;
	if (!cf.load(carSimPath))
		return false; //TODO Error if car not found

	int nw = 0;
	cf.getParam("wheels", nw);
	if (nw >= MIN_WHEEL_COUNT && nw <= MAX_WHEEL_COUNT)
		setNumWheels(nw);

	if (!dyn.loadFromConfig(cf)) {
		std::cerr << "CarDynamics load failed" << std::endl;
		return false; //TODO Error if not all car params found
	}

	return true;
}

void Car::loadModel() {
	// Each car has its own resource group
	Ogre::ResourceGroupManager::getSingleton().createResourceGroup(resGrpId);
	Ogre::Root::getSingleton().addResourceLocation(carPath + "/mesh", "FileSystem", resGrpId);
	Ogre::Root::getSingleton().addResourceLocation(carPath + "/textures", "FileSystem", resGrpId);

	mainNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	forDeletion(mainNode);

	Ogre::SceneNode* carNode = mainNode->createChildSceneNode();
	forDeletion(carNode);

	//TODO Allow for camera to follow car (using FollowCamera class)?
	//TODO Create car reflection (using CarReflection class)?

	// Create car body, interior, and glass
	carNode->attachObject(loadPart("body"));
	carNode->attachObject(loadPart("interior"));
	carNode->attachObject(loadPart("glass"));

	// Create wheels and brakes
	for (int w = 0; w < numWheels; w++) {
		wheelNodes[w] = mainNode->createChildSceneNode();
		forDeletion(wheelNodes[w]);
		//TODO Add support for custom wheel types, as in CarModel::Create()
		wheelNodes[w]->attachObject(loadPart("wheel", w));

		brakeNodes[w] = mainNode->createChildSceneNode();
		forDeletion(brakeNodes[w]);
		brakeNodes[w]->attachObject(loadPart("brake", w));
	}

	loadMaterials();

	// Set material of car body
	mSceneMgr->getEntity(resGrpId + "_body")->setMaterialName(mtrNames[mtrCarBody]);

	Ogre::ResourceGroupManager::getSingleton().initialiseResourceGroup(resGrpId);
	Ogre::ResourceGroupManager::getSingleton().loadResourceGroup(resGrpId);
}

void Car::loadMaterials() {
	std::string carBodyMtr = "car_body";
	if (Ogre::MaterialManager::getSingleton().resourceExists(carBodyMtr + "_" + mCarName)) {
		carBodyMtr += "_" + mCarName;
	}
	mtrNames[mtrCarBody] = carBodyMtr;

	std::string carBrakeMtr = "car_glass";
	if (Ogre::MaterialManager::getSingleton().resourceExists(carBrakeMtr + "_" + mCarName)) {
		carBrakeMtr += "_" + mCarName;
	}
	mtrNames[mtrCarBrake] = carBrakeMtr;

	for (int i=0; i < 1; ++i) {
		//FIXME Currently altering "global" instance of material; refer to CarModel::RecreateMaterials
		sh::MaterialInstance* m = sh::Factory::getInstance().getMaterialInstance(mtrNames[i]);

		m->setListener(this);

		// Change textures for the car
		if (m->hasProperty("diffuseMap")) {
			std::string v = sh::retrieveValue<sh::StringValue>(m->getProperty("diffuseMap"), 0).get();
			m->setProperty("diffuseMap", sh::makeProperty<sh::StringValue>(new sh::StringValue(mCarName + "_" + v)));
		}
		if (m->hasProperty("carPaintMap")) {
			std::string v = sh::retrieveValue<sh::StringValue>(m->getProperty("carPaintMap"), 0).get();
			m->setProperty("carPaintMap", sh::makeProperty<sh::StringValue>(new sh::StringValue(mCarName + "_" + v)));
		}
		if (m->hasProperty("reflMap")) {
			std::string v = sh::retrieveValue<sh::StringValue>(m->getProperty("reflMap"), 0).get();
			m->setProperty("reflMap", sh::makeProperty<sh::StringValue>(new sh::StringValue(mCarName + "_" + v)));
		}
	}

	updateLightMap();
}

Ogre::Entity* Car::loadPart(std::string partType, int partId) {
	std::string extPartType = "_" + partType;

	Ogre::Entity* entity = mSceneMgr->createEntity(resGrpId + extPartType +
												   (partId != -1 ? Ogre::StringConverter::toString(partId) : ""),
												   mCarName + extPartType + ".mesh", resGrpId);
	forDeletion(entity);

	return entity;
}

void Car::setNumWheels(int nw) {
	numWheels = nw;
	wheelNodes.resize(numWheels);
	brakeNodes.resize(numWheels);
}

void Car::changeColor() {
//	int i = iColor;
//	float c_h = pSet->gui.car_hue[i], c_s = pSet->gui.car_sat[i],
//		  c_v = pSet->gui.car_val[i], gloss = pSet->gui.car_gloss[i], refl = pSet->gui.car_refl[i];
//	carColor.setHSB(1-c_h, c_s, c_v);  //set, mini pos clr

	//TODO Hard-coded color; will need to add setting later
	Ogre::MaterialPtr mtr = Ogre::MaterialManager::getSingleton().getByName(mtrNames[mtrCarBody]);
	if (!mtr.isNull()) {
		Ogre::Material::TechniqueIterator techIt = mtr->getTechniqueIterator();
		while (techIt.hasMoreElements()) {
			Ogre::Technique* tech = techIt.getNext();
			Ogre::Technique::PassIterator passIt = tech->getPassIterator();
			while (passIt.hasMoreElements()) {
				Ogre::Pass* pass = passIt.getNext();
				if (pass->hasFragmentProgram()) {
					Ogre::GpuProgramParametersSharedPtr params = pass->getFragmentProgramParameters();
					params->setNamedConstant("carColour", carColor);
//					params->setNamedConstant("glossiness", 1 - gloss);
//					params->setNamedConstant("reflectiveness", refl);
				}
			}
		}
	}

	//Refer to CarModel::ChangeClr
}

void Car::updateLightMap() {
	Ogre::MaterialPtr mtr;
	for (int i = 0; i < numMaterials; ++i) {
		mtr = Ogre::MaterialManager::getSingleton().getByName(mtrNames[i]);
		if (!mtr.isNull()) {
			Ogre::Material::TechniqueIterator techIt = mtr->getTechniqueIterator();
			while (techIt.hasMoreElements()) {
				Ogre::Technique* tech = techIt.getNext();
				Ogre::Technique::PassIterator passIt = tech->getPassIterator();
				while (passIt.hasMoreElements()) {
					Ogre::Pass* pass = passIt.getNext();
					if (pass->hasFragmentProgram())	{
						Ogre::GpuProgramParametersSharedPtr params = pass->getFragmentProgramParameters();
						params->setIgnoreMissingParams(true);  // Don't throw exception if material doesn't use lightmap
						params->setNamedConstant("enableTerrainLightMap", true); //TODO Hard-coded
					}
				}
			}
		}
	}

	//Refer to CarModel::UpdateLightMap
}
