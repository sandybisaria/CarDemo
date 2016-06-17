#include "Car.hpp"

#include "CarConstants.hpp"
#include "../shiny/Main/Factory.hpp"
#include "../util/ConfigFile.hpp"

#include <OgreRoot.h>
#include <OgreEntity.h>

Car::Car()
	: mSceneMgr(0), mainNode(0),
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
	carPath = "../data/cars/" + mCarName;

	if (!loadFromConfig()){
		return; //TODO With error
	}

	mSceneMgr = sceneMgr;
	loadModel();

	//TODO Load starting position from somewhere...
	Ogre::Vector3 startPos(0, 10, 0);
	info.setPos(startPos);
	dyn.setPos(startPos);

	info.setSource(&dyn);
}

void Car::update(float dt) {
	info.update();
	mainNode->setPosition(info.getPos());

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
	if (!cf.load(carSimPath)) {
		return false; //TODO Error if car not found
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
	if (!dyn.loadFromConfig(cf)) {
		return false;
	}

	return true;
}

void Car::loadModel() {
	// Each car has its own resource group
	Ogre::ResourceGroupManager::getSingleton().createResourceGroup(mCarName);
	Ogre::Root::getSingleton().addResourceLocation(carPath + "/mesh", "FileSystem", mCarName);
	Ogre::Root::getSingleton().addResourceLocation(carPath + "/textures", "FileSystem", mCarName);

	mainNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	forDeletion(mainNode);

	Ogre::SceneNode* carNode = mainNode->createChildSceneNode();
	forDeletion(carNode);

	//TODO Allow for camera to follow car (using FollowCamera class)?
	//TODO Create car reflection (using CarReflection class)?

	// Create car body
	std::string bodyMesh = mCarName + "_body";
	Ogre::Entity* body = mSceneMgr->createEntity(bodyMesh, bodyMesh + ".mesh", mCarName);
	forDeletion(body);
	carNode->attachObject(body);

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

	loadMaterials();

	// Set material of car body
	body->setMaterialName(mtrNames[mtrCarBody]);

	Ogre::ResourceGroupManager::getSingleton().initialiseResourceGroup(mCarName);
	Ogre::ResourceGroupManager::getSingleton().loadResourceGroup(mCarName);
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

void Car::setNumWheels(int nw) {
	numWheels = nw;
	wheelNodes.resize(numWheels);
	brakeNodes.resize(numWheels);

	dyn.setNumWheels(nw);

	/*TODO Keep track of the following in vectors:
	 * Wheel position/radius/width/trail/temperature
	 * SceneNodes* for "Wh" (Wheel), "WhE" (Wheel Emitter), "Brake"
	 * Suspension bump detection, "Last bezier patch that each wheel hit"
	 */
}

void Car::changeColor() {
//	int i = iColor;
//	float c_h = pSet->gui.car_hue[i], c_s = pSet->gui.car_sat[i],
//		  c_v = pSet->gui.car_val[i], gloss = pSet->gui.car_gloss[i], refl = pSet->gui.car_refl[i];
//	carColor.setHSB(1-c_h, c_s, c_v);  //set, mini pos clr

	//Hard-coded color; will need to add setting later
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

	//TODO Refer to CarModel::ChangeClr
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
						params->setNamedConstant("enableTerrainLightMap", true); //Hard-coded
					}
				}
			}
		}
	}

	//TODO Refer to CarModel::UpdateLightMap
}

void Car::forDeletion(Ogre::SceneNode* node) {
	nodesToDelete.push_back(node);
}

void Car::forDeletion(Ogre::Entity* entity) {
	entitiesToDelete.push_back(entity);
}
