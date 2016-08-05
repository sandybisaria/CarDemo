#include "App.hpp"
#include "Sim.hpp"

#include "shiny/Platforms/Ogre/OgrePlatform.hpp"
#include "shiny/Platforms/Ogre/OgreMaterial.hpp"

#include "terrain/Scene.hpp"

#include "util/Axes.hpp"
#include "util/ConfigFile.hpp"

#include <OgreConfigFile.h>
#include <OgreRenderWindow.h>

App::App(Ogre::Root* root)
	: mShutDown(false), mFactory(0), mScene(0), mSim(0),
	  mRoot(root), mWindow(0), mSceneMgr(0), mCamera(0), mCameraNode(0),
	  followCam(true), mInputMgr(0), mKeyboard(0) {
	Axes::init(); // The Axes functions need to be initialized before use
}

App::~App() {
	mRoot->removeFrameListener(this);
	Ogre::WindowEventUtilities::removeWindowEventListener(mWindow, this);

	windowClosed(mWindow);

	delete mSim;
	delete mScene;

	//TODO Commented due to seg-fault
//	delete mFactory;
}

void App::run() {
	if (!setup()) {	return;	}

	// In the future, possibly allow for manual loop (locked fps)
	mRoot->startRendering();
}

bool App::setup() {
	// Change if your Ogre installation directory differs
	const Ogre::String OGRE_PLUGINS_DIR("/usr/local/lib/OGRE/");
	mRoot->loadPlugin(OGRE_PLUGINS_DIR + "RenderSystem_GL");
	mRoot->loadPlugin(OGRE_PLUGINS_DIR + "Plugin_OctreeSceneManager");
	// The plugin Plugin_ParticleFX was omitted due to the (current) lack of particles

	setupResources();

	if (!setupRenderSystem()) {	return false; }

	setupInputSystem();

	// ST_EXTERIOR_FAR seemed to be the most suitable scene manager type
	mSceneMgr = mRoot->createSceneManager(Ogre::ST_EXTERIOR_FAR);

	setupMaterials();

	setupSim();

	Ogre::TextureManager::getSingleton().setDefaultNumMipmaps(5);

	setupListeners();

	setupScene();

	return true;
}

void App::setupResources() {
	Ogre::ConfigFile cf;
	cf.load("../config/resources.cfg");
	Ogre::String name, locType;
	Ogre::ConfigFile::SectionIterator si = cf.getSectionIterator();

	while (si.hasMoreElements())	{
		Ogre::ConfigFile::SettingsMultiMap* settings = si.getNext();
		Ogre::ConfigFile::SettingsMultiMap::iterator i;

		for (i = settings->begin(); i != settings->end(); i++) {
			locType = i->first;
			name = i->second;

			Ogre::ResourceGroupManager::getSingleton().addResourceLocation(name, locType);
		}
	}
	Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
}

bool App::setupRenderSystem() {
	// Defaulting to OpenGL for cross-platform support
	const std::string RENDER_SYSTEM("OpenGL Rendering Subsystem");
	Ogre::RenderSystem* rs = mRoot->getRenderSystemByName(RENDER_SYSTEM);
	if (rs->getName() != RENDER_SYSTEM) return false;

	rs->setConfigOption("Full Screen", "No");
	rs->setConfigOption("Video Mode", "800 x 600 @ 32-bit");
	// Look up ConfigOptionMap if you want to set other config options

	mRoot->setRenderSystem(rs);

	mWindow = mRoot->initialise(true, "CarDemo");
	return true;
}

void App::setupInputSystem() {
	OIS::ParamList pl;
	size_t windowHandle = 0;
	std::ostringstream windowHandleStr;

	mWindow->getCustomAttribute("WINDOW", &windowHandle);
	windowHandleStr << windowHandle;
	pl.insert(std::make_pair(std::string("WINDOW"), windowHandleStr.str()));
	mInputMgr = OIS::InputManager::createInputSystem(pl);

	mKeyboard = static_cast<OIS::Keyboard*>(
		mInputMgr->createInputObject(OIS::OISKeyboard, true));
}

void App::setupMaterials() {
	// Shiny material scripts (*.mat, etc.) stored in materials2 dir
	sh::OgrePlatform* platform = new sh::OgrePlatform("General",
													  "../data/materials2");
	mFactory = new sh::Factory(platform);

	setMaterialFactoryDefaults();

	mFactory->setCurrentLanguage(sh::Language_GLSL); // Default for Linux
	mFactory->loadAllFiles();

	mFactory->setMaterialListener(this);
}

void App::setMaterialFactoryDefaults() {
	sh::Factory& fct = *mFactory;
	fct.setReadSourceCache(true);
	fct.setWriteSourceCache(true);
	fct.setReadMicrocodeCache(true);
	fct.setWriteMicrocodeCache(true);

	fct.setGlobalSetting("fog", "false");
	fct.setGlobalSetting("wind", "false");
	fct.setGlobalSetting("mrt_output", "false");
	fct.setGlobalSetting("shadows", "false");
	fct.setGlobalSetting("shadows_pssm", "false");
	fct.setGlobalSetting("shadows_depth", "false"); //Hard-coded
	fct.setGlobalSetting("lighting", "true");
//	fct.setGlobalSetting("terrain_composite_map", "false");
	fct.setGlobalSetting("soft_particles", "false");
	fct.setGlobalSetting("editor", "false");
	fct.setSharedParameter("terrainWorldSize", sh::makeProperty<sh::FloatValue>(new sh::FloatValue(10000.0)));
	//TODO What other defaults are needed (from App::SetFactoryDefaults)?
	// In the future, may need settings from CScene::UpdFog
}

void App::setupSim() {
	mSim = new Sim(this);
	mSim->setup(mSceneMgr);
}

void App::setupListeners() {
	mRoot->addFrameListener(this);
	Ogre::WindowEventUtilities::addWindowEventListener(mWindow, this);

	mKeyboard->setEventCallback(this);
}

void App::setupScene() {
	//Would entail managing cameras in separate class
	mCamera = mSceneMgr->createCamera("MainCamera");

	mCameraNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("MainCameraNode");
	mCameraNode->setPosition(Ogre::Vector3::ZERO);
	mCameraNode->setDirection(Ogre::Vector3::UNIT_X);
	mCameraNode->attachObject(mCamera);

	// In the future, make user-configurable a la Stuntrally
	mCamera->setNearClipDistance(0.2);
	mCamera->setFarClipDistance(0); // Infinite clip distance

	Ogre::Viewport* vp = mWindow->addViewport(mCamera);
	vp->setBackgroundColour(Ogre::ColourValue::Black);
	mCamera->setAspectRatio(Ogre::Real(vp->getActualWidth() /
									   vp->getActualHeight()));

	if (!loadSurfaces()) { return; }

	mScene = new Scene(mSceneMgr);
	mScene->setup(mSim);
}

bool App::loadSurfaces() {
	surfaces.clear();
	surfaceMap.clear();

	tires.clear();
	tireMap.clear();

	std::string path = "../data/scene/surfaces.cfg";
	ConfigFile params;
	if (!params.load(path)) return false;

	std::list<std::string> secList;
	params.getSectionList(secList);
	for (std::list<std::string>::const_iterator sec = secList.begin(); sec != secList.end(); sec++) {
		TerrainSurface* ts = new TerrainSurface();
		ts->name = *sec;
		surfaces.push_back(*ts);
		surfaceMap[ts->name] = surfaces.size() - 1;

		int id;
		params.getParam(*sec + ".ID", id); // The ID may only be used for sounds
		ts->setType((unsigned int) id);

		float temp = 0.0;
		params.getParam(*sec + ".BumpWaveLength", temp);
		ts->bumpWavelength = temp;
		params.getParam(*sec + ".BumpAmplitude", temp);
		ts->bumpAmplitude = temp;

		params.getParam(*sec + ".FrictionTread", temp);
		ts->friction = temp;
		if (params.getParam(*sec + ".FrictionX", temp))
			ts->frictionX = temp;
		if (params.getParam(*sec + ".FrictionY", temp))
			ts->frictionY = temp;

		if (params.getParam(*sec + ".RollResistance", temp))
			ts->rollingResist = temp;
		params.getParam(*sec + ".RollingDrag", temp);
		ts->rollingDrag = temp;

		// Get the tire
		std::string tireFile;
		if (!params.getParam(*sec + "." + "Tire", tireFile)) {
			tireFile = "Default"; // Default tire if not found
		}
		ts->tireName = tireFile;

		// If the tire was not already initialized, load the file
		if (tireMap.find(ts->tireName) == tireMap.end()) {
			if (!loadTire(ts->tireName)) { return false; }
		}

		ts->tire = getTire(ts->tireName);
	}

	return true;
}

bool App::loadTire(std::string name) {
	std::string tirePath = "../data/cars/common/" + name + ".tire";
	ConfigFile tireParams;
	if (!tireParams.load(tirePath)) { return false; }

	CarTire* t = new CarTire();
	t->name = name;
	tires.push_back(*t);
	tireMap[t->name] = tires.size() - 1;

	float value;
	for (int i = 0; i < 15; ++i) {
		int numinfile = i;
			 if (i == 11)	numinfile = 111;
		else if (i == 12)	numinfile = 112;
		else if (i > 12)	numinfile -= 1;

		std::stringstream str;  str << "params.a" << numinfile;
		if (!tireParams.getParam(str.str(), value)) { return false; }
		t->lateral[i] = value;
	}
	for (int i = 0; i < 11; ++i) {
		std::stringstream str;  str << "params.b" << i;
		if (!tireParams.getParam(str.str(), value)) { return false; }
		t->longitudinal[i] = value;
	}
	for (int i = 0; i < 18; ++i) {
		std::stringstream str;  str << "params.c" << i;
		if (!tireParams.getParam(str.str(), value)) { return false; }
		t->aligning[i] = value;
	}

	t->calculateSigmaHatAlphaHat();
	return true;
}

void App::materialCreated(sh::MaterialInstance* m,
						  const std::string& configuration,
						  unsigned short lodIndex) {
	Ogre::Technique* t = static_cast<sh::OgreMaterial*>(m->getMaterial())->
			getOgreTechniqueForConfiguration(configuration, lodIndex);

	if (m->hasProperty("instancing") && sh::retrieveValue<sh::StringValue>(
				m->getProperty("instancing"), 0).get() == "true") {
		t->setShadowCasterMaterial("shadowcaster_instancing");
	}

	if (!m->hasProperty("transparent") || !sh::retrieveValue<sh::BooleanValue>(
				m->getProperty("transparent"), 0).get()) {
		t->setShadowCasterMaterial("shadowcaster_noalpha");
	}
}

bool App::frameRenderingQueued(const Ogre::FrameEvent& evt) {
	if (mShutDown) { return false; }
	if (mWindow->isClosed()) { return false; }

	mKeyboard->capture();
	updateCamera(evt);

	mSim->update(evt.timeSinceLastFrame);
	mScene->update(evt.timeSinceLastFrame);

	return true;
}

// To control the main camera
void App::updateCamera(const Ogre::FrameEvent& evt) {
	if (!followCam) {
		Ogre::Vector3 translation(Ogre::Vector3::ZERO);
		if (mKeyboard->isKeyDown(OIS::KC_W)) {
			translation += Ogre::Vector3::NEGATIVE_UNIT_Z;
		}
		if (mKeyboard->isKeyDown(OIS::KC_S)) {
			translation += Ogre::Vector3::UNIT_Z;
		}
		if (mKeyboard->isKeyDown(OIS::KC_E)) {
			translation += Ogre::Vector3::UNIT_Y;
		}
		if (mKeyboard->isKeyDown(OIS::KC_Q)) {
			translation += Ogre::Vector3::NEGATIVE_UNIT_Y;
		}
		translation *= 0.10;
		mCameraNode->translate(translation, Ogre::Node::TS_LOCAL);

		if (mKeyboard->isKeyDown(OIS::KC_A)) {
			mCameraNode->yaw(Ogre::Radian(0.005f), Ogre::Node::TS_LOCAL);
		}
		if (mKeyboard->isKeyDown(OIS::KC_D)) {
			mCameraNode->yaw(Ogre::Radian(-0.005f), Ogre::Node::TS_LOCAL);
		}
	}

	else {
		mCameraNode->setPosition(mSim->getCameraPosition());
		mCameraNode->setOrientation(mSim->getCameraOrientation());
	}
}

void App::windowClosed(Ogre::RenderWindow* rw) {
	if (rw == mWindow) {
		if (mInputMgr) {
			mInputMgr->destroyInputObject(mKeyboard);

			OIS::InputManager::destroyInputSystem(mInputMgr);
			mInputMgr = 0;
		}
	}
}

bool App::keyPressed(const OIS::KeyEvent& ke) {
	mSim->keyPressed(ke);

	switch (ke.key) {
	// Shut down
	case OIS::KC_ESCAPE:
		mShutDown = true;
		break;

	// Toggle follow-cam
	case OIS::KC_C:
		followCam = !followCam;
		break;

	default:
		break;
	}

	return true;
}

bool App::keyReleased(const OIS::KeyEvent& ke) {
	mSim->keyReleased(ke);

	return true;
}
