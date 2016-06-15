#include <iostream>

#include <OgreConfigFile.h>
#include <OgreRenderWindow.h>

#include "App.hpp"
#include "Scene.hpp"

App::App(Ogre::Root* root)
	: mShutDown(false), mScene(0),
	  mRoot(root), mWindow(0), mSceneMgr(0), mCamera(0), mCameraNode(0),
	  mInputMgr(0), mKeyboard(0) {
}

App::~App() {
	mRoot->removeFrameListener(this);
	Ogre::WindowEventUtilities::removeWindowEventListener(mWindow, this);

	windowClosed(mWindow);

	delete mScene;
}

void App::run() {
	if (!setup()) {
		return;
	}

	mRoot->startRendering(); //TODO Manual loop for locked fps
}

bool App::setup() {
	//TODO Look up in user's system
	const Ogre::String OGRE_PLUGINS_DIR("/usr/local/lib/OGRE/");

// Switch between debug and release plugins
#ifdef _DEBUG
#define D_SUFFIX "_d"
#else
#define D_SUFFIX ""
#endif

	mRoot->loadPlugin(OGRE_PLUGINS_DIR + "RenderSystem_GL" + D_SUFFIX);
	mRoot->loadPlugin(OGRE_PLUGINS_DIR + "Plugin_OctreeSceneManager" + D_SUFFIX);
	//TODO Add Plugin_ParticleFX for particles

	setupResources();

	if (!setupRenderSystem()) {
		return false;
	}

	setupInputSystem();

	mSceneMgr = mRoot->createSceneManager(Ogre::ST_EXTERIOR_FAR);
	mScene = new Scene(mSceneMgr);

	//TODO Explore multiple viewports for split-screen effects
	//Would entail managing cameras in separate class
	setupViewSystem();

	Ogre::TextureManager::getSingleton().setDefaultNumMipmaps(5);

	//TODO Init GUI system

	Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();

	setupListeners();
	setupScene();

	//TODO Setup compositors for rendering effects?

	//TODO Setup material factory?

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

void App::setupViewSystem() {
	mCamera = mSceneMgr->createCamera("MainCamera");

	mCameraNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("MainCameraNode");
	mCameraNode->setPosition(Ogre::Vector3(0, 80, 0));
	mCameraNode->setDirection(Ogre::Vector3::ZERO);
	mCameraNode->attachObject(mCamera);

	//TODO Make configurable a la Stuntrally
	mCamera->setNearClipDistance(0.2);
	mCamera->setFarClipDistance(0); // Infinite clip distance

	Ogre::Viewport* vp = mWindow->addViewport(mCamera);
	vp->setBackgroundColour(Ogre::ColourValue::Black);
	mCamera->setAspectRatio(Ogre::Real(vp->getActualWidth() /
									   vp->getActualHeight()));
}

void App::setupListeners() {
	mRoot->addFrameListener(this);
	Ogre::WindowEventUtilities::addWindowEventListener(mWindow, this);

	mKeyboard->setEventCallback(this);
}

void App::setupScene() {
	//TODO Set up GUI?

	mScene->setupTerrain();
}

bool App::frameRenderingQueued(const Ogre::FrameEvent& evt) {
	if (mShutDown) {
		return false;
	}

	if (mWindow->isClosed()) {
		return false;
	}

	mKeyboard->capture();
	moveCamera(evt);

	return true;
}

void App::moveCamera(const Ogre::FrameEvent& evt) {
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
