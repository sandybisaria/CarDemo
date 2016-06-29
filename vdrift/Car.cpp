#include "Car.hpp"

#include "CarConstants.hpp"
#include "../shiny/Main/Factory.hpp"
#include "../util/ConfigFile.hpp"
#include "../vdrift/MathVector.hpp"
#include "../vdrift/Quaternion.hpp"

#include <OgreRoot.h>
#include <OgreEntity.h>

#include <iostream> //TODO DEL

Car::Car(int id)
	: mId(id), mSceneMgr(0), mainNode(0),
	  dyn(0),
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

	delete dyn;
}

void Car::setup(std::string carName, Ogre::SceneManager* sceneMgr, CollisionWorld& world) {
	mCarName = carName;
	resGrpId = mCarName + "_" + Ogre::StringConverter::toString(mId);
	carPath = "../data/cars/" + mCarName;

	dyn = new CarDynamics();

	if (!loadFromConfig(world)) return; //TODO With error

	mSceneMgr = sceneMgr;
	loadModel();
}

void Car::updatePreviousVelocity() { dyn->updatePreviousVelocity(); }

void Car::update(float dt) {
	dyn->update();

	updateModel();
	updateLightMap();
}

double Car::getSpeedDir() { return dyn->getSpeedDir(); }

void Car::handleInputs(const std::vector<float>& inputs, float dt) {
	assert(inputs.size() == CarInput::ALL);

	int curGear = dyn->getTransmission().getGear();
	bool rear = curGear == -1; // Is car in reverse?

	// We assume that we use -1 when trying to drive in reverse; change in the future...
	float brake = !rear? inputs[CarInput::THROTTLE] : inputs[CarInput::BRAKE];
	dyn->setBrake(brake);

	dyn->setHandBrake(inputs[CarInput::HANDBRAKE]);

	float steerValue = inputs[CarInput::STEER_RIGHT];
	if (std::abs(inputs[CarInput::STEER_LEFT]) > std::abs(inputs[CarInput::STEER_RIGHT]))
		steerValue = -inputs[CarInput::STEER_LEFT];
	dyn->setSteering(steerValue, 0.81 * 0.7); //TODO Hard-coded value based on default settings

	int gearChange = 0;
	if (inputs[CarInput::SHIFT_UP]   == 1.0) gearChange =  1.0;
	if (inputs[CarInput::SHIFT_DOWN] == 1.0) gearChange = -1.0;
	int newGear = curGear + gearChange;
	dyn->shiftGear(gearChange);

	float throttle = !rear ? inputs[CarInput::THROTTLE] : inputs[CarInput::BRAKE];
	throttle = 1;
	dyn->setThrottle(throttle);

	float clutch = 1 - inputs[CarInput::CLUTCH];
	dyn->setClutch(clutch);

	std::cout << throttle << " " << steerValue << std::endl;
}

void Car::requestedConfiguration(sh::MaterialInstance* m, const std::string& configuration) {

}

void Car::createdConfiguration(sh::MaterialInstance* m, const std::string& configuration) {
	changeColor();
	updateLightMap();
}

bool Car::loadFromConfig(CollisionWorld& world) {
	// Top-level directory for all car data
	std::string carSimPath = carPath + "/sim/" + mCarName + ".car";

	ConfigFile cf;
	if (!cf.load(carSimPath))
		return false; //TODO Error if car not found

	int nw = 0;
	cf.getParam("wheels", nw);
	if (nw >= MIN_WHEEL_COUNT && nw <= MAX_WHEEL_COUNT)
		setNumWheels(nw);

	// Load car collision params (Stuntrally puts it outside of CARDYNAMICS so...
	// com
	dyn->comOfsL = 0.f;  cf.getParam("collision.com_ofs_L", dyn->comOfsL);
	dyn->comOfsH = 0.f;  cf.getParam("collision.com_ofs_H", dyn->comOfsH);
	// dim
	dyn->collR   = 0.3f;  cf.getParam("collision.radius", 	 dyn->collR);
	dyn->collR2m = 0.6f;  cf.getParam("collision.radius2mul",dyn->collR2m);
	dyn->collH   = 0.45f; cf.getParam("collision.height", 	 dyn->collH);
	dyn->collW   = 0.5f;  cf.getParam("collision.width",  	 dyn->collW);
	// ofs
	dyn->collLofs = 0.f;  cf.getParam("collision.offsetL", dyn->collLofs);
	dyn->collWofs = 0.f;  cf.getParam("collision.offsetW", dyn->collWofs);
	dyn->collHofs = 0.f;  cf.getParam("collision.offsetH", dyn->collHofs);
	dyn->collLofs -= dyn->comOfsL;
	dyn->collHofs -= dyn->comOfsH;
	// L
	dyn->collPosLFront = 1.9f; cf.getParam("collision.posLfront", dyn->collPosLFront);
	dyn->collPosLBack = -1.9f; cf.getParam("collision.posLrear",  dyn->collPosLBack);
	// w
	dyn->collFrWMul  = 0.2f;   cf.getParam("collision.FrWmul",  dyn->collFrWMul);
	dyn->collFrHMul  = 1.0f;   cf.getParam("collision.FrHmul",  dyn->collFrHMul);
	dyn->collTopWMul = 0.8f;   cf.getParam("collision.TopWmul", dyn->collTopWMul);
	// Top L pos
	dyn->collTopFr    = 0.4f;  cf.getParam("collision.TopFr",    dyn->collTopFr);
	dyn->collTopMid   =-0.3f;  cf.getParam("collision.TopMid",   dyn->collTopMid);
	dyn->collTopBack  =-1.1f;  cf.getParam("collision.TopBack",  dyn->collTopBack);
	// Top h mul
	dyn->collTopFrHm  = 0.2f;  cf.getParam("collision.TopFrHm",  dyn->collTopFrHm);
	dyn->collTopMidHm = 0.4f;  cf.getParam("collision.TopMidHm", dyn->collTopMidHm);
	dyn->collTopBackHm= 0.2f;  cf.getParam("collision.TopBackHm",dyn->collTopBackHm);

	dyn->collFriction = 0.4f;  cf.getParam("collision.friction",  dyn->collFriction);
	dyn->collFlTrigH  = 0.f;   cf.getParam("collision.fluidTrigH",dyn->collFlTrigH);
	dyn->collFlTrigH -= dyn->comOfsH;

	if (!dyn->load(cf)) {
		std::cerr << "CarDynamics load failed" << std::endl;
		return false; //TODO Error if not all car params found
	}

	//TODO Load starting position/rotation from the scene
	MathVector<double, 3> pos(0, 0, 0);
	Quaternion<double> rot;

	float stOfsY = 0.f;
	cf.getParam("collision.start-offsetY", stOfsY);
	pos[2] += stOfsY - 0.4 + dyn->comOfsH;

	dyn->init(pos, rot, world);

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
		sh::Factory::getInstance().destroyMaterialInstance(mtrNames[i] + Ogre::StringConverter::toString(mId));
		sh::MaterialInstance* m = sh::Factory::getInstance().createMaterialInstance(mtrNames[i] + Ogre::StringConverter::toString(mId),
																					mtrNames[i]);

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

		mtrNames[i] = mtrNames[i] + Ogre::StringConverter::toString(mId);
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
					params->setNamedConstant("glossiness", Ogre::Real(1 - 0.5));
					params->setNamedConstant("reflectiveness", Ogre::Real(0.5));
				}
			}
		}
	}

	//Refer to CarModel::ChangeClr
}

void Car::updateModel() {
	// Main body
	Ogre::Vector3 pos = Axes::vectorToOgre(dyn->getPosition()) + Ogre::Vector3::UNIT_Y;
//	std::cout << "My position: " << pos << std::endl;
	mainNode->setPosition(pos);

	Ogre::Quaternion rot; rot = Axes::doQuatToOgre(dyn->getOrientation());
//	std::cout << "My orientation: " << rot << std::endl;
	mainNode->setOrientation(rot);

	// Wheels
	for (int w = 0; w < numWheels; w++) {
		WheelPosition wp; wp = WheelPosition(w);

		Ogre::Vector3 whPos = Axes::vectorToOgre(dyn->getWheelPosition(wp));
//		std::cout << "My wheel position: " << whPos << std::endl;
		wheelNodes[w]->setPosition(whPos);

		Ogre::Quaternion whRot; whRot = Axes::doWhQuatToOgre(dyn->getWheelOrientation(wp));
//		std::cout << "My wheel orientation: " << whRot << std::endl;
		wheelNodes[w]->setOrientation(whRot);
	}

	// Brakes
	for (int w = 0; w < numWheels; w++) {
		if (brakeNodes[w]) {
			brakeNodes[w]->_setDerivedOrientation(mainNode->getOrientation());

			// This transformation code is just so the brake mesh can have the same alignment as the wheel mesh
			brakeNodes[w]->yaw(Ogre::Degree(-90), Ogre::Node::TS_LOCAL);
			if (w%2 == 1) brakeNodes[w]->setScale(-1, 1, 1);

			brakeNodes[w]->pitch(Ogre::Degree(180), Ogre::Node::TS_LOCAL);

			// Turn only front wheels
//			if (w < 2) brakeNodes[w]->yaw(-Ogre::Degree(posInfo.whSteerAng[w])); TODO Steering angle

//			std::cout << "My brake orientation: " << brakeNodes[w]->getOrientation() << std::endl;
		}
	}
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
