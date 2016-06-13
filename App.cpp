#include <iostream>

#include <OgreViewport.h>

#include "App.hpp"

App::App() {
	// No .cfg files, yet
	mRoot = OGRE_NEW Ogre::Root("", "");

	mWindow = 0; mSceneMgr = 0; mCamera = 0;
}

App::~App() {
	delete mRoot;
}

void App::run() {
	// Set up window
	mWindow = mRoot->initialise(true, "CarDemo");

	// Set up scene manager
	mSceneMgr = mRoot->createSceneManager(Ogre::ST_GENERIC);

	// Set up the camera
	mCamera = mSceneMgr->createCamera("MainCam");

	// Set up the viewport
	Ogre::Viewport* vp = mWindow->addViewport(mCamera);
	vp->setBackgroundColour(Ogre::ColourValue::Black);

	mCamera->setAspectRatio(Ogre::Real(vp->getActualWidth()) /
							Ogre::Real(vp->getActualHeight()));

	// Start the render loop
	mRoot->startRendering();
}
