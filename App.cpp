#include <iostream>

#include <OgrePlugin.h>
#include <OgreViewport.h>
#include <OgreWindowEventUtilities.h>

#include "App.hpp"

App::App()
	: mRoot(0),
	  mWindow(0),
	  mSceneMgr(0),
	  mCamera(0) {
}

App::~App() {
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
	Ogre::ResourceGroupManager::getSingleton().addResourceLocation("../media/materials", "FileSystem");

	Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
}

bool App::setupRenderSystem() {
	// Set up the rendering system
	Ogre::RenderSystem* rs = mRoot->getRenderSystemByName("OpenGL Rendering Subsystem");
	if (rs->getName() != "OpenGL Rendering Subsystem")
		return false;

	rs->setConfigOption("Full Screen", "No");
	rs->setConfigOption("Video Mode", "800 x 600 @ 32-bit");

	mRoot->setRenderSystem(rs);

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

	// Set up scene manager
	mSceneMgr = mRoot->createSceneManager(Ogre::ST_EXTERIOR_FAR);

	// Set up the camera
	mCamera = mSceneMgr->createCamera("MainCam");
	mSceneMgr->setSkyDome(true, "CloudySky");

	// Set up the viewport
	Ogre::Viewport* vp = mWindow->addViewport(mCamera);
	vp->setBackgroundColour(Ogre::ColourValue::Black);

	mCamera->setAspectRatio(Ogre::Real(vp->getActualWidth()) /
							Ogre::Real(vp->getActualHeight()));

	// Start the render loop
	while (true) {
		// Listen to window events (e.g. window closing)
		Ogre::WindowEventUtilities::messagePump();

		if (!mRoot->renderOneFrame())
			return;

		if (mWindow->isClosed())
			return;
	}
}
