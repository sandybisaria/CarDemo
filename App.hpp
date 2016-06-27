#pragma once

#include "terrain/Scene.hpp"
#include "Sim.hpp"

#include "vdrift/TerrainSurface.hpp"

#include "shiny/Main/Factory.hpp"

#include <OgreRoot.h>
#include <OgreFrameListener.h>
#include <OgreWindowEventUtilities.h>

#include <OISInputManager.h>
#include <OISKeyboard.h>

#include <vector>
#include <map>

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
	bool setup();

	void setupResources();
	bool setupRenderSystem();
	void setupInputSystem();
	void setupSim();
	void setupListeners();

	void setupScene();
	bool loadSurfaces();

	void setupMaterials();
	void setMaterialFactoryDefaults();

	virtual void materialCreated(sh::MaterialInstance* m, const std::string& configuration, unsigned short lodIndex);

	virtual bool frameRenderingQueued(const Ogre::FrameEvent& evt);
	void updateCamera(const Ogre::FrameEvent& evt);

	virtual void windowClosed(Ogre::RenderWindow* rw);

	virtual bool keyPressed(const OIS::KeyEvent& ke);
	virtual bool keyReleased(const OIS::KeyEvent& ke);

	bool mShutDown;

	Sim* mSim;
	Scene* mScene;

	std::vector<TerrainSurface> surfaces;
	std::map<std::string, int> surfaceMap; // Map surface name to ID

	sh::Factory* mFactory;

	Ogre::Root* mRoot;
	Ogre::RenderWindow* mWindow;
	Ogre::SceneManager* mSceneMgr;
	Ogre::Camera* mCamera;
	Ogre::SceneNode* mCameraNode;

	OIS::InputManager* mInputMgr;
	OIS::Keyboard* mKeyboard;
};
