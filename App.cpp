#include <iostream>

#include "shiny/Platforms/Ogre/OgrePlatform.hpp"
#include "shiny/Platforms/Ogre/OgreMaterial.hpp"
#include "shiny/Main/PropertyBase.hpp"

#include <OgreConfigFile.h>
#include <OgreRenderWindow.h>

#include "App.hpp"
#include "util/Axes.hpp"

App::App(Ogre::Root* root)
	: mShutDown(false), mFactory(0), mScene(0), mSim(0),
	  mRoot(root), mWindow(0), mSceneMgr(0), mCamera(0), mCameraNode(0),
	  mInputMgr(0), mKeyboard(0) {
	Axes::init();
}

App::~App() {
	mRoot->removeFrameListener(this);
	Ogre::WindowEventUtilities::removeWindowEventListener(mWindow, this);

	windowClosed(mWindow);

	delete mSim;
	delete mScene;

	delete mFactory;
}

void App::run() {
	if (!setup()) {
		return;
	}

	mRoot->startRendering(); //TODO Allow for manual loop (locked fps)
}

bool App::setup() {
	//TODO Look up in user's system
	const Ogre::String OGRE_PLUGINS_DIR("/usr/local/lib/OGRE/");

	mRoot->loadPlugin(OGRE_PLUGINS_DIR + "RenderSystem_GL");
	mRoot->loadPlugin(OGRE_PLUGINS_DIR + "Plugin_OctreeSceneManager");
	//TODO Add Plugin_ParticleFX for particles

	setupResources();

	if (!setupRenderSystem()) {
		return false;
	}

	setupInputSystem();

	mSceneMgr = mRoot->createSceneManager(Ogre::ST_EXTERIOR_FAR);

	setupMaterials();
	setupSim();

	Ogre::TextureManager::getSingleton().setDefaultNumMipmaps(5);

	//TODO Init GUI system

	setupListeners();
	setupScene();

	//TODO Setup compositors for rendering effects?

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

			Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
					name, locType);
		}
	}
	Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
}

bool App::setupRenderSystem() {
	const std::string RENDER_SYSTEM("OpenGL Rendering Subsystem");
	Ogre::RenderSystem* rs = mRoot->getRenderSystemByName(RENDER_SYSTEM);
	if (rs->getName() != RENDER_SYSTEM) {
		return false;
	}

	rs->setConfigOption("Full Screen", "No");
//	rs->setConfigOption("Video Mode", "800 x 600 @ 32-bit");

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

	mKeyboard = static_cast<OIS::Keyboard*>(mInputMgr->createInputObject(OIS::OISKeyboard, true));
}

void App::setupMaterials() {
	sh::OgrePlatform* platform = new sh::OgrePlatform("General", "../data/materials2");
	mFactory = new sh::Factory(platform);

	setMaterialFactoryDefaults();

	mFactory->setCurrentLanguage(sh::Language_GLSL); // Default, for Linux
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

	//TODO NEED STUFF FROM CScene::UpdFog TOO??
//	fct.setSharedParameter("fogColorSun",  sh::makeProperty<sh::Vector4>(new sh::Vector4(0, 0, 0, 0))); //Hard-coded
//	fct.setSharedParameter("fogColorAway",  sh::makeProperty<sh::Vector4>(new sh::Vector4(0, 0, 0, 0))); //Hard-coded
//	fct.setSharedParameter("fogColorH",  sh::makeProperty<sh::Vector4>(new sh::Vector4(0, 0, 0, 0))); //Hard-coded
//	fct.setSharedParameter("fogParamsH",  sh::makeProperty<sh::Vector4>(new sh::Vector4(0, 0, 0, 0))); //Hard-coded
}

void App::setupSim() {
	mSim = new Sim();
	mSim->setup(mSceneMgr);
}

void App::setupListeners() {
	mRoot->addFrameListener(this);
	Ogre::WindowEventUtilities::addWindowEventListener(mWindow, this);

	mKeyboard->setEventCallback(this);
}

void App::setupScene() {
	//TODO Explore multiple viewports for split-screen effects
	//Would entail managing cameras in separate class
	mCamera = mSceneMgr->createCamera("MainCamera");

	mCameraNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("MainCameraNode");
	mCameraNode->setPosition(Ogre::Vector3(0, 1, -2));
	mCameraNode->setDirection(Ogre::Vector3::UNIT_Z);
	mCameraNode->attachObject(mCamera);

	//TODO Make configurable a la Stuntrally
	mCamera->setNearClipDistance(0.2);
	mCamera->setFarClipDistance(0); // Infinite clip distance

	Ogre::Viewport* vp = mWindow->addViewport(mCamera);
	vp->setBackgroundColour(Ogre::ColourValue::Black);
	mCamera->setAspectRatio(Ogre::Real(vp->getActualWidth() /
									   vp->getActualHeight()));

	if (!loadSurfaces()) ; //TODO Error: Can't load surfaces

	//TODO Set up GUI?
	mScene = new Scene(mSceneMgr);
	mScene->setupTerrain(mSim);
}

bool App::loadSurfaces() {
	surfaces.clear();
	surfaceMap.clear();

	std::string path = "../data/scene/surfaces.cfg";
	ConfigFile params;
	if (!params.load(path)) return false;

	std::list<std::string> secList;
	params.getSectionList(secList);
	for (std::list<std::string>::const_iterator sec = secList.begin(); sec != secList.end(); sec++) {
		TerrainSurface surf;
		surf.name = *sec;

		int id;
		params.getParam(*sec + ".ID", id); // For sound..?
		surf.setType(id);

		float temp = 0.0;
		params.getParam(*sec + ".BumpWaveLength", temp);	surf.bumpWavelength = temp;
		params.getParam(*sec + ".BumpAmplitude", temp);		surf.bumpAmplitude = temp;

		params.getParam(*sec + ".FrictionTread", temp);		surf.friction = temp;
		if (params.getParam(*sec + ".FrictionX", temp))  	surf.frictionX = temp;
		if (params.getParam(*sec + ".FrictionY", temp))  	surf.frictionY = temp;

		if (params.getParam(*sec + ".RollResistance", temp))surf.rollingResist = temp;
		params.getParam(*sec + ".RollingDrag", temp);		surf.rollingDrag = temp;

		// Tire
		std::string tireFile;
		if (!params.getParam(*sec + "." + "Tire", tireFile))
			tireFile = "Default"; // Default surface if not found
		surf.tireName = tireFile;
		//TODO tire_map of GAME??

		surfaces.push_back(surf);
		surfaceMap[surf.name] = (int)surfaces.size(); //+1, 0 = not found
	}

	return true;
}

//TODO Investigate if required
void App::materialCreated(sh::MaterialInstance* m, const std::string& configuration, unsigned short lodIndex) {
	Ogre::Technique* t = static_cast<sh::OgreMaterial*>(m->getMaterial())->
			getOgreTechniqueForConfiguration(configuration, lodIndex);

	if (m->hasProperty("instancing") &&
			sh::retrieveValue<sh::StringValue>(m->getProperty("instancing"), 0).get() == "true") {
		t->setShadowCasterMaterial("shadowcaster_instancing");
	}

	if (!m->hasProperty("transparent") ||
			!sh::retrieveValue<sh::BooleanValue>(m->getProperty("transparent"), 0).get()) {
		t->setShadowCasterMaterial("shadowcaster_noalpha");
	}
}

bool App::frameRenderingQueued(const Ogre::FrameEvent& evt) {
	if (mShutDown) {
		return false;
	}

	if (mWindow->isClosed()) {
		return false;
	}

	mKeyboard->capture();
	updateCamera(evt);

	mSim->update(evt.timeSinceLastFrame);

	return true;
}

void App::updateCamera(const Ogre::FrameEvent& evt) {
	Ogre::Vector3 translation(Ogre::Vector3::ZERO);
	if (mKeyboard->isKeyDown(OIS::KC_W)) {
		translation += Ogre::Vector3::NEGATIVE_UNIT_Z;
	} if (mKeyboard->isKeyDown(OIS::KC_S)) {
		translation += Ogre::Vector3::UNIT_Z;
	} if (mKeyboard->isKeyDown(OIS::KC_E)) {
		translation += Ogre::Vector3::UNIT_Y;
	} if (mKeyboard->isKeyDown(OIS::KC_Q)) {
		translation += Ogre::Vector3::NEGATIVE_UNIT_Y;
	}
//	translation *= 0.01;
	mCameraNode->translate(translation, Ogre::Node::TS_LOCAL);

	if (mKeyboard->isKeyDown(OIS::KC_A)) {
		mCameraNode->yaw(Ogre::Radian(0.005), Ogre::Node::TS_LOCAL);
	} if (mKeyboard->isKeyDown(OIS::KC_D)) {
		mCameraNode->yaw(Ogre::Radian(-0.005), Ogre::Node::TS_LOCAL);
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
	switch (ke.key) {
	case OIS::KC_ESCAPE:
		mShutDown = true;
		break;

	default:
		break;
	}

	return true;
}

bool App::keyReleased(const OIS::KeyEvent& ke) {
	return true;
}
