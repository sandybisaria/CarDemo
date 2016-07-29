#include "Car.hpp"

#include "CarConstants.hpp"
#include "CarDynamics.hpp"

#include "../shiny/Main/Factory.hpp"

#include "../terrain/RenderConst.hpp"

#include <OgreRoot.h>
#include <OgreEntity.h>

#define toString(s) Ogre::StringConverter::toString(s)

Car::Car(int id)
	: mId(id), mSceneMgr(0), mainNode(0),
	  dyn(0), rangeMul(0.81 * 0.7),
	  carColor(1, 1, 1) {
	setNumWheels(DEF_WHEEL_COUNT);

	// For rangeMul: TODO May want to just put rangeMul as 1
	// 0.81 is due to "normal" (as opposed to "easy") sim (from Stuntrally)
	// 0.7 due to driving on asphalt (as opposed to other terrains) (from Stuntrally)
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
	resGrpIdStr = mCarName + "_" + toString(mId);
	carPath = "../data/cars/" + mCarName;

	dyn = new CarDynamics();
	if (!loadFromConfig(world)) { return; }

	dyn->shiftGear(1); // The car starts off in first gear

	mSceneMgr = sceneMgr;
	loadModel();
}

void Car::update() {
	dyn->update();
	updateModel();
	updateLightMap();
}

double Car::getSpeedDir() {
	return dyn->getSpeedDir();
}

double Car::getSpeedMPS() {
	return dyn->getSpeedMPS();
}

double Car::getMaxAngle() const {
	return dyn->getMaxAngle();
}

MathVector<double, 3> Car::getDownVector() {
	return dyn->getDownVector();
}

MathVector<double, 3> Car::getForwardVector() {
	return dyn->getForwardVector();
}

/* The format of the inputs vector is as follows:
 * 0 -> 1 (disengaged -> fully-engaged) = BRAKE, THROTTLE, HANDBRAKE, CLUTCH, STEER_RIGHT, STEER_LEFT
 * 0 or 1 (on or off) = SHIFT_UP/DOWN
 *
 * Note that SHIFT_DOWN takes "higher precedence" than SHIFT_UP (in case both are engaged)
 * Note that the steering value with greater magnitude takes precedence (0.7L vs 0.5R -> steer left)
 */
void Car::handleInputs(const std::vector<double>& inputs) {
	assert(inputs.size() == CarInput::ALL);

	int curGear = dyn->getTransmission().getGear();

	// Note that this in effect "inverts" the controls when in reverse, since the brake key acts like a throttle.
	// While not necessarily "realistic" it makes keyboard control more intuitive.
	bool rear = curGear == -1; // Is car currently in reverse?

	// We assume that we use -1 when trying to drive in reverse
	double brake = !rear ? inputs[CarInput::BRAKE] : inputs[CarInput::THROTTLE];
	dyn->setBrake(brake);

	dyn->setHandBrake(inputs[CarInput::HANDBRAKE]);

	double steerValue = inputs[CarInput::STEER_RIGHT];
	if (std::abs(inputs[CarInput::STEER_LEFT]) > std::abs(inputs[CarInput::STEER_RIGHT]))
		steerValue = -inputs[CarInput::STEER_LEFT];
	dyn->setSteering(steerValue, rangeMul);

	int gearChange = 0;
	if (inputs[CarInput::SHIFT_UP]   == 1.0) gearChange =  1;
	if (inputs[CarInput::SHIFT_DOWN] == 1.0) gearChange = -1;
	int newGear = curGear + gearChange;
	dyn->shiftGear(newGear);

	double throttle = !rear ? inputs[CarInput::THROTTLE] : inputs[CarInput::BRAKE];
	dyn->setThrottle(throttle);

	double clutch = 1 - inputs[CarInput::CLUTCH];
	dyn->setClutch(clutch);
}

void Car::createdConfiguration(sh::MaterialInstance* m, const std::string& configuration) {
	changeColor();
	updateLightMap();
}

bool Car::loadFromConfig(CollisionWorld& world) {
	// Top-level directory for all car data
	std::string carSimPath = carPath + "/sim/" + mCarName + ".car";

	ConfigFile cf;
	if (!cf.load(carSimPath)) { return false; }

	int nw = 0;
	cf.getParam("wheels", nw);
	if (nw >= MIN_WHEEL_COUNT && nw <= MAX_WHEEL_COUNT) { setNumWheels(nw); }

	// Load car collision params (Stuntrally puts this outside of CARDYNAMICS so... I will too, for now)
	// Center-of-mass offsets
	dyn->comOfsL = 0.f;  cf.getParam("collision.com_ofs_L", dyn->comOfsL);
	dyn->comOfsH = 0.f;  cf.getParam("collision.com_ofs_H", dyn->comOfsH);
	// Collision dimensions
	dyn->collR   = 0.3f;  cf.getParam("collision.radius", 	 dyn->collR);
	dyn->collR2m = 0.6f;  cf.getParam("collision.radius2mul",dyn->collR2m);
	dyn->collH   = 0.45f; cf.getParam("collision.height", 	 dyn->collH);
	dyn->collW   = 0.5f;  cf.getParam("collision.width",  	 dyn->collW);
	// Collision offsets
	dyn->collLofs = 0.f;  cf.getParam("collision.offsetL", dyn->collLofs);
	dyn->collWofs = 0.f;  cf.getParam("collision.offsetW", dyn->collWofs);
	dyn->collHofs = 0.f;  cf.getParam("collision.offsetH", dyn->collHofs);
	dyn->collLofs -= dyn->comOfsL;
	dyn->collHofs -= dyn->comOfsH;
	// Length ranges (bumpers)
	dyn->collPosLFront = 1.9f; cf.getParam("collision.posLfront", dyn->collPosLFront);
	dyn->collPosLBack = -1.9f; cf.getParam("collision.posLrear",  dyn->collPosLBack);
	// Multipliers
	dyn->collFrWMul  = 0.2f;   cf.getParam("collision.FrWmul",  dyn->collFrWMul);
	dyn->collFrHMul  = 1.0f;   cf.getParam("collision.FrHmul",  dyn->collFrHMul);
	dyn->collTopWMul = 0.8f;   cf.getParam("collision.TopWmul", dyn->collTopWMul);
	// Top L pos
	dyn->collTopFr    = 0.4f;  cf.getParam("collision.TopFr",    dyn->collTopFr);
	dyn->collTopMid   =-0.3f;  cf.getParam("collision.TopMid",   dyn->collTopMid);
	dyn->collTopBack  =-1.1f;  cf.getParam("collision.TopBack",  dyn->collTopBack);
	// Top H multiplier
	dyn->collTopFrHm  = 0.2f;  cf.getParam("collision.TopFrHm",  dyn->collTopFrHm);
	dyn->collTopMidHm = 0.4f;  cf.getParam("collision.TopMidHm", dyn->collTopMidHm);
	dyn->collTopBackHm= 0.2f;  cf.getParam("collision.TopBackHm",dyn->collTopBackHm);
	// Rigid body friction and fluid trigger height
	dyn->collFriction = 0.4f;  cf.getParam("collision.friction",  dyn->collFriction);
	dyn->collFlTrigH  = 0.f;   cf.getParam("collision.fluidTrigH",dyn->collFlTrigH);
	dyn->collFlTrigH -= dyn->comOfsH;

	if (!dyn->load(cf)) {
		std::cerr << "CarDynamics load failed" << std::endl;
		return false; // Error if not all car params found
	}

	//TODO Load starting position/rotation from the scene, or receive from setup()
	MathVector<double, 3> pos(0, mId * 10, 1); // mId * 10 is a cheap way to spread out multiple cars
	Quaternion<double> rot;

	float stOfsY = 0.f;
	cf.getParam("collision.start-offsetY", stOfsY);
	pos[2] += stOfsY - 0.4 + dyn->comOfsH;

	dyn->init(pos, rot, world);

	return true;
}

void Car::loadModel() {
	// Each car has its own resource group
	Ogre::ResourceGroupManager::getSingleton().createResourceGroup(resGrpIdStr);
	Ogre::Root::getSingleton().addResourceLocation(carPath + "/mesh", "FileSystem", resGrpIdStr);
	Ogre::Root::getSingleton().addResourceLocation(carPath + "/textures", "FileSystem", resGrpIdStr);

	mainNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	forDeletion(mainNode);

	Ogre::SceneNode* carNode = mainNode->createChildSceneNode();
	forDeletion(carNode);

	// Create car body, interior, and glass
	carNode->attachObject(loadPart("body"));
	carNode->attachObject(loadPart("interior"));
	carNode->attachObject(loadPart("glass"));

	// Create wheels and brakes
	//TODO Add support for custom wheel types, as in CarModel::Create()
	for (int w = 0; w < numWheels; w++) {
		// Wheel nodes must be parented to the root for proper model rendering
		wheelNodes[w] = mSceneMgr->getRootSceneNode()->createChildSceneNode();
		forDeletion(wheelNodes[w]);
		wheelNodes[w]->attachObject(loadPart("wheel", w));

		// The CAD model does not have separate brake meshes
		// Rather than specifically check, just don't render any brakes... (sorry for laziness)
//		brakeNodes[w] = wheelNodes[w]->createChildSceneNode();
//		forDeletion(brakeNodes[w]);
//		brakeNodes[w]->attachObject(loadPart("brake", w));
	}

	loadMaterials();

	// Set material of car body
	mSceneMgr->getEntity(resGrpIdStr + "_body")->setMaterialName(mtrNames[mtrCarBody]);

	Ogre::ResourceGroupManager::getSingleton().initialiseResourceGroup(resGrpIdStr);
	Ogre::ResourceGroupManager::getSingleton().loadResourceGroup(resGrpIdStr);
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
		sh::Factory::getInstance().destroyMaterialInstance(
			mtrNames[i] + toString(mId));
		sh::MaterialInstance* m = sh::Factory::getInstance().createMaterialInstance(
			mtrNames[i] + toString(mId), mtrNames[i]);

		m->setListener(this);

		// Set texture properties for the car
		if (m->hasProperty("diffuseMap")) {
			std::string v = sh::retrieveValue<sh::StringValue>(
				m->getProperty("diffuseMap"), 0).get();
			m->setProperty("diffuseMap", sh::makeProperty<sh::StringValue>(
				new sh::StringValue(mCarName + "_" + v)));
		}
		if (m->hasProperty("carPaintMap")) {
			std::string v = sh::retrieveValue<sh::StringValue>(
				m->getProperty("carPaintMap"), 0).get();
			m->setProperty("carPaintMap", sh::makeProperty<sh::StringValue>(
				new sh::StringValue(mCarName + "_" + v)));
		}
		if (m->hasProperty("reflMap")) {
			std::string v = sh::retrieveValue<sh::StringValue>(
				m->getProperty("reflMap"), 0).get();
			m->setProperty("reflMap", sh::makeProperty<sh::StringValue>(
				new sh::StringValue(mCarName + "_" + v)));
		}

		mtrNames[i] = mtrNames[i] + toString(mId);
	}

	updateLightMap();
}

Ogre::Entity* Car::loadPart(std::string partType, int partId) {
	std::string extPartType = "_" + partType;

	// partId has default value of -1 if nothing is passed in
	std::string entityName = resGrpIdStr + extPartType + (partId != -1 ? toString(partId) : "");
	Ogre::Entity* entity = mSceneMgr->createEntity(entityName,
												   mCarName + extPartType + ".mesh",
												   resGrpIdStr);
	forDeletion(entity);

	if (partType == "glass") {
		entity->setRenderQueueGroup(RQG_CarGlass);
		entity->setVisibilityFlags(RV_CarGlass);
	} else {
		entity->setVisibilityFlags(RV_Car);
	}

	return entity;
}

void Car::setNumWheels(int nw) {
	numWheels = nw;
	wheelNodes.resize((unsigned int) numWheels);
//	brakeNodes.resize(numWheels);
}

void Car::changeColor() {
//	int i = iColor;
//	float c_h = pSet->gui.car_hue[i], c_s = pSet->gui.car_sat[i],
//		  c_v = pSet->gui.car_val[i], gloss = pSet->gui.car_gloss[i], refl = pSet->gui.car_refl[i];
//	carColor.setHSB(1-c_h, c_s, c_v);  //set, mini pos clr
	//TODO Hard-coded color, glossiness, and reflectiveness; will need to add setting later

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

	// Refer to Stuntrally's CarModel::ChangeClr
}

void Car::updateModel() {
	// Main body
	Ogre::Vector3 pos = Axes::vectorToOgre(dyn->getPosition());
	mainNode->setPosition(pos);

	Ogre::Quaternion rot; rot = Axes::doQuatToOgre(dyn->getOrientation());
	mainNode->setOrientation(rot);

	// Wheels
	for (int w = 0; w < numWheels; w++) {
		WheelPosition wp; wp = WheelPosition(w);

		Ogre::Vector3 whPos = Axes::vectorToOgre(dyn->getWheelPosition(wp));
		wheelNodes[w]->setPosition(whPos);

		Ogre::Quaternion whRot; whRot = Axes::doWhQuatToOgre(dyn->getWheelOrientation(wp));
		wheelNodes[w]->setOrientation(whRot);
	}

	// Brakes TODO Temporarily disabled
//	for (int w = 0; w < numWheels; w++) {
//		if (brakeNodes[w]) {
//			WheelPosition wp; wp = WheelPosition(w);
//			brakeNodes[w]->_setDerivedOrientation(mainNode->getOrientation());
//
//			// This transformation code is needed so the brake mesh can have the same alignment as the wheel mesh
//			brakeNodes[w]->yaw(Ogre::Degree(-90), Ogre::Node::TS_LOCAL);
//			if (w % 2 == 1) brakeNodes[w]->setScale(-1, 1, 1);
//
//			brakeNodes[w]->pitch(Ogre::Degree(180), Ogre::Node::TS_LOCAL);
//
//			// Turn only front wheels
//			if (w < 2) brakeNodes[w]->yaw(-Ogre::Degree(dyn->getWheelSteerAngle(wp)));
//		}
//	}
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

	// Refer to Stuntrally's CarModel::UpdateLightMap
}
