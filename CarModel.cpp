#include "CarModel.hpp"

#include "vdrift/cardefs.h"

#define toStr(s) Ogre::StringConverter::toString(s)

CarModel::CarModel(int id, std::string carModelName, Ogre::SceneManager* sceneMgr, Sim* sim)
	: mId(id), mCarModelName(carModelName),
	  mSceneMgr(sceneMgr), mSim(sim), mCar(0),
	  carColor(0, 1, 0) {

	resGrpId = mCarModelName + toStr(mId);
	carResPath = "../data/cars/" + mCarModelName; //TODO Pathmanager for paths?
	mtrId = toStr(mId);

	setModelDefaults();
}

CarModel::~CarModel() {
	destroyOgreObjects();

	if (Ogre::ResourceGroupManager::getSingleton().resourceGroupExists(resGrpId))
		Ogre::ResourceGroupManager::getSingleton().destroyResourceGroup(resGrpId);
}

void CarModel::setModelDefaults() {
	setNumWheels(DEF_WHEELS); //TODO Keep in constructor?

	for (int i = 0; i < CONF_VEC_SIZE; i++) {
		interiorOffset[i] = 0.f;
	}

	rotationFix = false;

	for (int w = 0; w < numWheels; w++) {
		wheelRadius[w] = 0.3f;
		wheelWidth[w] = 0.2f;
	}
}

void CarModel::load() {
	setModelDefaults();

	if (!loadConfig()) {
		//TODO Error: CarModel can't load .car
		return;
	}

	//TODO Get start position and orientation from Sim (Scene)
	MATHVECTOR<double, 3> pos(0, 0, 0);
	QUATERNION<double> rot;

	std::string carConf = carResPath + "/sim/" + mCarModelName + ".car";
	CONFIGFILE cf;
	cf.Load(carConf);

	mCar = mSim->loadCar(mCarModelName, cf, pos, rot, mId);
	if (!mCar)
		return;
	else {
		mCar->mCarModel = this;
	}
}

bool CarModel::loadConfig() {
	std::string carConf = carResPath + "/sim/" + mCarModelName + ".car";

	CONFIGFILE cf;
	if (!cf.Load(carConf)) {
		return false;
	}

	int nw = 0;
	cf.GetParam("wheels", nw);
	if (nw >= MIN_WHEELS && nw <= MAX_WHEELS)
		setNumWheels(nw);

	// Custom interior model offset
	cf.GetParam("model_ofs.interior-x", interiorOffset[0]);
	cf.GetParam("model_ofs.interior-y", interiorOffset[1]);
	cf.GetParam("model_ofs.interior-z", interiorOffset[2]);
	cf.GetParam("model_ofs.rot_fix", rotationFix);

	//TODO Add other params as necessary

	// Tire params
	float val;
	bool both = cf.GetParam("tire-both.radius", val); // If param exists, both = true.

	int j = std::max(2, numWheels / 2);
	for (int i = 0; i < j; i++) {
		WHEEL_POSITION wl, wr;
		std::string pos;

		getWheelPosStr(i, numWheels, wl, wr, pos);
		if (both) pos = "both";

		float radius;
		cf.GetParamE("tire-"+pos+".radius", radius);
		wheelRadius[wl] = radius;  wheelRadius[wr] = radius;

		float width = 0.2f;
		cf.GetParam("tire-"+pos+".width-trail", width);
		wheelWidth[wl] = width;  wheelWidth[wr] = width;
	}

	// Wheel pos (for track's ghost or garage view only?)
	int version = 2;
	cf.GetParam("version", version);
	for (int i = 0; i < numWheels; ++i) {
		std::string sPos = sCfgWh[i];
		float pos[CONF_VEC_SIZE];
		MATHVECTOR<float,CONF_VEC_SIZE> vec;

		cf.GetParamE("wheel-"+sPos+".position", pos);
		if (version == 2)  convertV2to1(pos[0],pos[1],pos[2]);
		vec.Set(pos[0],pos[1], pos[2]);

		wheelPos[i] = vec;
	}
}

void CarModel::create() {
	Ogre::ResourceGroupManager::getSingleton().createResourceGroup(resGrpId);

	Ogre::Root::getSingleton().addResourceLocation(carResPath + "/mesh", "FileSystem", resGrpId);
	Ogre::Root::getSingleton().addResourceLocation(carResPath + "/textures", "FileSystem", resGrpId);

	Ogre::SceneNode* root = mSceneMgr->getRootSceneNode();
	mMainNode = root->createChildSceneNode(); forDel(mMainNode);

	Ogre::SceneNode* carNode = mMainNode->createChildSceneNode(); forDel(carNode);

	//TODO Create reflection (CarModel::CreateReflection)

	if (rotationFix) {
		carNode->setOrientation(Ogre::Quaternion(Ogre::Degree(90), Ogre::Vector3::UNIT_Y) *
								Ogre::Quaternion(Ogre::Degree(180), Ogre::Vector3::UNIT_X));
	}

	//TODO At some point, worry about RenderQueueGroups and RenderConst.h

	// Create car body
	Ogre::Entity* body = mSceneMgr->createEntity(resGrpId + "_body", mCarModelName + "_body.mesh", resGrpId); forDel(body);
	carNode->attachObject(body);

	// Create interior
	Ogre::Entity* interior = mSceneMgr->createEntity(resGrpId + "_interior", mCarModelName + "_interior.mesh", resGrpId); forDel(interior);
	carNode->attachObject(interior);

	// Create glass
	Ogre::Entity* glass = mSceneMgr->createEntity(resGrpId + "_glass", mCarModelName + "_glass.mesh", resGrpId); forDel(glass);
	carNode->attachObject(glass);

#define resExists(s) Ogre::MeshManager::getSingleton().resourceExists(s)

	// Create wheels and brakes
	int w2 = numWheels == 2? 1 : 2;
	for (int w = 0; w < numWheels; w++) {
		std::string wheelName = resGrpId + "_wheel_" + toStr(w);
		wheelNodes[w] = root->createChildSceneNode(); forDel(wheelNodes[w]);

		std::string wheelMeshFile = mCarModelName + "_wheel.mesh";

		// For custom wheels
		if (w < w2 && resExists(mCarModelName + "_wheel_front.mesh"))
			wheelMeshFile = mCarModelName + "_wheel_front.mesh";
		else if (w >= w2 && resExists(mCarModelName + "_wheel_rear.mesh"))
			wheelMeshFile = mCarModelName + "_wheel_rear.mesh";
		else if (w % 2 == 0 && resExists(mCarModelName + "_wheel_left.mesh"))
			wheelMeshFile = mCarModelName + "_wheel_left.mesh";
		else if (w % 2 == 1 && resExists(mCarModelName + "_wheel_right.mesh"))
			wheelMeshFile = mCarModelName + "_wheel_right.mesh";

		if (resExists(wheelMeshFile)) {
			Ogre::Entity* wheel = mSceneMgr->createEntity(wheelName, wheelMeshFile, resGrpId); forDel(wheel);
			wheelNodes[w]->attachObject(wheel);
		}

		std::string brakeName = resGrpId + "_brake_" + toStr(w);
		if (resExists(mCarModelName + "_brake.mesh")) {
			brakeNodes[w] = wheelNodes[w]->createChildSceneNode(); forDel(brakeNodes[w]);

			Ogre::Entity* brake = mSceneMgr->createEntity(brakeName, mCarModelName + "_brake.mesh", resGrpId); forDel(brake);
			brakeNodes[w]->attachObject(brake);
		}
	}

	recreateMaterials();
	// Set car body material
	body->setMaterialName(materialNames[carBodyMtr]);

	Ogre::ResourceGroupManager::getSingleton().initialiseResourceGroup(resGrpId);
	Ogre::ResourceGroupManager::getSingleton().loadResourceGroup(resGrpId);
#undef resExists
}

void CarModel::recreateMaterials() {
#define resExists(s) Ogre::MaterialManager::getSingleton().resourceExists(s)

	std::string bodyMat = "car_body";
	if (resExists(bodyMat + "_" + mCarModelName))
		bodyMat += "_" + mCarModelName;
	materialNames[carBodyMtr] = bodyMat;

	// Referred to as brake material in Stuntrally...
	std::string brakeMat = "car_glass";
	if (resExists(brakeMat + "_" + mCarModelName))
		brakeMat += "_" + mCarModelName;
	materialNames[carBrakeMtr] = brakeMat;

	Ogre::MaterialPtr mat;
	// Stuntrally iterates only once...
	for (int i = 0; i < 1; i++) {
		sh::Factory::getInstance().destroyMaterialInstance(materialNames[i] + mtrId);
		sh::MaterialInstance* m = sh::Factory::getInstance().createMaterialInstance(materialNames[i] + mtrId, materialNames[i]);

		m->setListener(this);

		// change textures for the car
		if (m->hasProperty("diffuseMap")) {
			std::string v = sh::retrieveValue<sh::StringValue>(m->getProperty("diffuseMap"), 0).get();
			m->setProperty("diffuseMap", sh::makeProperty<sh::StringValue>(new sh::StringValue(mCarModelName + "_" + v)));
		}
		if (m->hasProperty("carPaintMap")) {
			std::string v = sh::retrieveValue<sh::StringValue>(m->getProperty("carPaintMap"), 0).get();
			m->setProperty("carPaintMap", sh::makeProperty<sh::StringValue>(new sh::StringValue(mCarModelName + "_" + v)));
		}
		if (m->hasProperty("reflMap")) {
			std::string v = sh::retrieveValue<sh::StringValue>(m->getProperty("reflMap"), 0).get();
			m->setProperty("reflMap", sh::makeProperty<sh::StringValue>(new sh::StringValue(mCarModelName + "_" + v)));
		}

		materialNames[i] = materialNames[i] + mtrId;
	}

	updateLightMap();
#undef resExists
}

void CarModel::setNumWheels(int n) {
	numWheels = n;
	wheelPos.resize(n); wheelRadius.resize(n); wheelWidth.resize(n);
	//TODO whTrail and whTemp, for when implementing trails/particles
	wheelNodes.resize(n); brakeNodes.resize(n); //TODO wheelEmitNodes?
}

void CarModel::destroyOgreObjects() {
	for (int i = 0; i < nodesToDel.size(); i++)
		mSceneMgr->destroySceneNode(nodesToDel[i]);
	nodesToDel.clear();

	for (int i = 0; i < entsToDel.size(); i++)
		mSceneMgr->destroyEntity(entsToDel[i]);
	entsToDel.clear();
}

void CarModel::changeColor() {
//	int i = iColor;
//	float c_h = pSet->gui.car_hue[i], c_s = pSet->gui.car_sat[i],
//		  c_v = pSet->gui.car_val[i], gloss = pSet->gui.car_gloss[i], refl = pSet->gui.car_refl[i];
//	carColor.setHSB(1-c_h, c_s, c_v);  //set, mini pos clr

	//TODO Hard-coded color; will need to add setting later
	Ogre::MaterialPtr mtr = Ogre::MaterialManager::getSingleton().getByName(materialNames[carBodyMtr]);
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

void CarModel::updateLightMap() {
	Ogre::MaterialPtr mtr;
	for (int i = 0; i < NUM_MTRS; i++) {
		mtr = Ogre::MaterialManager::getSingleton().getByName(materialNames[i]);
		if (!mtr.isNull()) {
			Ogre::Material::TechniqueIterator ti = mtr->getTechniqueIterator();
			while (ti.hasMoreElements()) {
				Ogre::Technique* t = ti.getNext();
				Ogre::Technique::PassIterator pi = t->getPassIterator();
				while (pi.hasMoreElements()) {
					Ogre::Pass* p = pi.getNext();
					if (p->hasFragmentProgram()) {
						Ogre::GpuProgramParametersSharedPtr params = p->getFragmentProgramParameters();
						params->setIgnoreMissingParams(true);
						//TODO Find bLightMapEnabled;
//						params->setNamedConstant("enableTerrainLightMap", bLightMapEnabled ? 1.f : 0.f);
					}
				}
			}
		}
	}
}

void CarModel::requestedConfiguration(sh::MaterialInstance* m, const std::string& configuration){
}

void CarModel::createdConfiguration(sh::MaterialInstance* m, const std::string& configuration) {
	updateLightMap();
	changeColor();
}
