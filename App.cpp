#include <iostream>

#include <OgreConfigFile.h>
#include <OgrePlugin.h>
#include <OgreViewport.h>

#include "App.hpp"

App::App()
	: mRoot(0),
	  mWindow(0),
	  mSceneMgr(0),
	  mCamera(0),
	  mTerrainGlobals(0),
	  mTerrainGroup(0),
	  mInputMgr(0),
	  mKeyboard(0) {
}

App::~App() {
	Ogre::WindowEventUtilities::removeWindowEventListener(mWindow, this);
	windowClosed(mWindow);

	delete mTerrainGroup;
	delete mTerrainGlobals;
	delete mRoot;
}

bool App::setupPlugins() {
	// Set up the plugins
	Ogre::StringVector requiredPlugins;
	requiredPlugins.push_back("GL RenderSystem");
	requiredPlugins.push_back("Octree Scene Manager");

	Ogre::StringVector pluginsToLoad;
	pluginsToLoad.push_back("RenderSystem_GL");
	pluginsToLoad.push_back("Plugin_OctreeSceneManager");

	const Ogre::String OGRE_PLUGINS_DIR("/usr/local/lib/OGRE/");

	for (Ogre::StringVector::iterator i = pluginsToLoad.begin(); i != pluginsToLoad.end(); i++) {
#ifdef _DEBUG
		mRoot->loadPlugin(OGRE_PLUGINS_DIR + *i + Ogre::String("_d"));
#else
		mRoot->loadPlugin(OGRE_PLUGINS_DIR + *i);
#endif
	}

	Ogre::Root::PluginInstanceList ip = mRoot->getInstalledPlugins();
	for (Ogre::StringVector::iterator i = requiredPlugins.begin(); i != requiredPlugins.end(); i++) {
		bool found = false;
		for (Ogre::Root::PluginInstanceList::iterator j = ip.begin(); j!= ip.end(); j++) {
			if ((*j)->getName() == *i) {
				found = true;
				break;
			}
		}
		if (!found)
			return false;
	}

	return true;
}

void App::setupResources() {
	Ogre::ConfigFile resourcesFile;
	resourcesFile.load("../config/resources.cfg");
	Ogre::String name, locType;
	Ogre::ConfigFile::SectionIterator secIt = resourcesFile.getSectionIterator();

	while (secIt.hasMoreElements())	{
		Ogre::ConfigFile::SettingsMultiMap* settings = secIt.getNext();
		Ogre::ConfigFile::SettingsMultiMap::iterator it;

		for (it = settings->begin(); it != settings->end(); ++it) {
			locType = it->first;
			name = it->second;

			Ogre::ResourceGroupManager::getSingleton().addResourceLocation(name, locType);
		}
	}

	Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
}

bool App::setupRenderSystem() {
	Ogre::RenderSystem* rs = mRoot->getRenderSystemByName("OpenGL Rendering Subsystem");
	if (rs->getName() != "OpenGL Rendering Subsystem")
		return false;

	rs->setConfigOption("Full Screen", "No");
	rs->setConfigOption("Video Mode", "800 x 600 @ 32-bit");

	mRoot->setRenderSystem(rs);

	return true;
}

void App::setupScene() {
	Ogre::TextureManager::getSingleton().setDefaultNumMipmaps(5);

	mSceneMgr = mRoot->createSceneManager(Ogre::ST_EXTERIOR_FAR);
	mSceneMgr->setSkyDome(true, "CloudySky");

	mSceneMgr->setAmbientLight(Ogre::ColourValue(0.5, 0.5, 0.5));

	Ogre::Light* sun = mSceneMgr->createLight("SunLight");
	sun->setType(Ogre::Light::LT_DIRECTIONAL);
	sun->setDirection(Ogre::Vector3::NEGATIVE_UNIT_Y);
	sun->setDiffuseColour(Ogre::ColourValue::White);
	sun->setSpecularColour(Ogre::ColourValue(0.4, 0.4, 0.4));
}

void App::setupCamera() {
	mCamera = mSceneMgr->createCamera("MainCam");

	mCamera->setPosition(0, 0, 0);
	mCamera->setDirection(Ogre::Vector3::UNIT_Z);

	mCamera->setNearClipDistance(5);
	if (mRoot->getRenderSystem()->getCapabilities()->hasCapability(
			Ogre::RSC_INFINITE_FAR_PLANE))
		mCamera->setFarClipDistance(0);
	else
		mCamera->setFarClipDistance(50000);
}

void App::setupViewport() {
	// Set up the viewport
	Ogre::Viewport* vp = mWindow->addViewport(mCamera);
	vp->setBackgroundColour(Ogre::ColourValue::Black);

	mCamera->setAspectRatio(Ogre::Real(vp->getActualWidth()) /
							Ogre::Real(vp->getActualHeight()));
}

void App::setupInput() {
	Ogre::LogManager::getSingleton().logMessage("*** Initializing OIS ***");

	OIS::ParamList pl;
	size_t windowHnd = 0;
	std::ostringstream windowHndStr;

	mWindow->getCustomAttribute("WINDOW", &windowHnd);
	windowHndStr << windowHnd;
	pl.insert(std::make_pair(std::string("WINDOW"), windowHndStr.str()));

	mInputMgr = OIS::InputManager::createInputSystem(pl);

	mKeyboard = static_cast<OIS::Keyboard*>(
	mInputMgr->createInputObject(OIS::OISKeyboard, true));

	mKeyboard->setEventCallback(this);

	windowResized(mWindow);

	Ogre::LogManager::getSingletonPtr()->logMessage("Finished");
}

void App::setupTerrain() {
//	mTerrainGlobals = new Ogre::TerrainGlobalOptions();
//
//	// Terrain lies on the x-z plane (perp to y axis)
//	const int TERRAIN_SIZE = 513;
//	const float WORLD_SIZE = 12000.0;
//	mTerrainGroup = new Ogre::TerrainGroup(mSceneMgr, Ogre::Terrain::ALIGN_X_Z, TERRAIN_SIZE, WORLD_SIZE);
//	mTerrainGroup->setOrigin(Ogre::Vector3::ZERO);
//
//	mTerrainGlobals->setMaxPixelError(8);
//	mTerrainGlobals->setCompositeMapDistance(3000);
//	mTerrainGlobals->setLightMapDirection(mSceneMgr->getLight("SunLight")->getDerivedDirection());
//	mTerrainGlobals->setCompositeMapAmbient(mSceneMgr->getAmbientLight());
//	mTerrainGlobals->setCompositeMapDiffuse(mSceneMgr->getLight("SunLight")->getDiffuseColour());
//
//	Ogre::Terrain::ImportData& importData = mTerrainGroup->getDefaultImportSettings();
//	importData.terrainSize = TERRAIN_SIZE;
//	importData.worldSize = WORLD_SIZE;
//	importData.inputScale = 600;
//	importData.minBatchSize = 33;
//	importData.maxBatchSize = 65;
//
//	importData.layerList.resize(1); // For now, only using one terrain
//	importData.layerList[0].worldSize = 100; // Size of each splat
//	importData.layerList[0].textureNames.push_back("grass_green_diffusespecular.dds");
//	importData.layerList[0].textureNames.push_back("grass_green_normalheight.dds");
//
//	Ogre::Image img;
//	img.load("terrain.png", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
//	mTerrainGroup->defineTerrain(0, 0, &img);
//
//	mTerrainGroup->loadAllTerrains(true);
//
//	mTerrainGroup->freeTemporaryResources();
}

bool App::frameRenderingQueued(const Ogre::FrameEvent& evt) {
	if (mWindow->isClosed())
		return false;

	mKeyboard->capture();

	return true;
}

void App::windowResized(Ogre::RenderWindow* rw) {

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
	default:
		break;
	}

	return true;
}

bool App::keyReleased(const OIS::KeyEvent& ke) {
	return true;
}

void App::run() {
	// Set up the root without a plugin or config file
	mRoot = new Ogre::Root("", "");

	if (!setupPlugins())
		return;
	setupResources();
	if (!setupRenderSystem())
		return;

	// Set up window
	mWindow = mRoot->initialise(true, "CarDemo");

	setupScene();
	setupCamera();
	setupViewport();
	setupInput();

//	setupTerrain();

	mRoot->addFrameListener(this);
	Ogre::WindowEventUtilities::addWindowEventListener(mWindow, this);

	mRoot->startRendering();
}
