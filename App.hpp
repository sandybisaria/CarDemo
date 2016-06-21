#pragma once

#include "Scene.hpp"
#include "Sim.hpp"

#include "shiny/Main/Factory.hpp"

#include <OgreRoot.h>
#include <OgreFrameListener.h>
#include <OgreWindowEventUtilities.h>

#include <OISInputManager.h>
#include <OISKeyboard.h>

class App
	: public sh::MaterialListener,
	  public Ogre::FrameListener,
	  public Ogre::WindowEventListener,
	  public OIS::KeyListener {
public:
	App(Ogre::Root* root);
	~App();

	void run();

private:
	// Setup methods
	bool setup();
	void setupResources();
	bool setupRenderSystem();
	void setupInputSystem();
	void setupSim();
	void setupViewSystem();
	void setupListeners();
	void setupScene();
	void setupMaterials();
	void setMaterialFactoryDefaults();

	virtual void materialCreated(sh::MaterialInstance* m, const std::string& configuration, unsigned short lodIndex);

	virtual bool frameRenderingQueued(const Ogre::FrameEvent& evt);
	void updateCamera(const Ogre::FrameEvent& evt);

	virtual void windowClosed(Ogre::RenderWindow* rw);

	virtual bool keyPressed(const OIS::KeyEvent& ke);
	virtual bool keyReleased(const OIS::KeyEvent& ke);

	bool mShutDown; // Setting this to true will make the app shut down

	Sim* mSim;
	Scene* mScene;

	sh::Factory* mFactory;

	Ogre::Root* mRoot;
	Ogre::RenderWindow* mWindow;
	Ogre::SceneManager* mSceneMgr;
	Ogre::Camera* mCamera;
	Ogre::SceneNode* mCameraNode;

	OIS::InputManager* mInputMgr;
	OIS::Keyboard* mKeyboard;
};
