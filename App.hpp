#pragma once

#include "vdrift/TerrainSurface.hpp"
#include "vdrift/CarTire.hpp"

#include "shiny/Main/Factory.hpp"

#include <OgreRoot.h>
#include <OgreFrameListener.h>
#include <OgreWindowEventUtilities.h>

#include <OISInputManager.h>
#include <OISKeyboard.h>

#include <vector>
#include <map>

// The App class is the main class of the simulation.
// It sets up Ogre, the input system,
class App
	: public sh::MaterialListener,
	  public Ogre::FrameListener,
	  public Ogre::WindowEventListener,
	  public OIS::KeyListener {
public:
	App(Ogre::Root* root);
	~App();

	void run();

	TerrainSurface* getTerrainSurface(std::string name) {
		return &surfaces.at(surfaceMap.at(name));
	}
	CarTire* getTire(std::string name) { return &tires.at(tireMap.at(name)); }

private:
//---- Setup methods
	bool setup();

	void setupResources();
	bool setupRenderSystem();
	void setupInputSystem();
	void setupSim();
	void setupListeners();

	void setupScene();

//---- Stuntrally loads the surfaces and car tires in their App class...
	bool loadSurfaces();
	bool loadTire(std::string name);
	// Each surface has its own tire type that is loaded on request

	std::vector<TerrainSurface> surfaces;
	std::map<std::string, size_t> surfaceMap; // Maps surface name to ID
	std::vector<CarTire> tires;
	std::map<std::string, size_t> tireMap; // Maps tire name to ID

//---- sh::MaterialListener methods
	void setupMaterials();
	void setMaterialFactoryDefaults();

	virtual void materialCreated(sh::MaterialInstance* m,
								 const std::string& configuration,
								 unsigned short lodIndex);

	sh::Factory* mFactory;

//---- Window/input/frame listener methods
	virtual bool frameRenderingQueued(const Ogre::FrameEvent& evt);
	void updateCamera(const Ogre::FrameEvent& evt);

	virtual void windowClosed(Ogre::RenderWindow* rw);

	virtual bool keyPressed(const OIS::KeyEvent& ke);
	virtual bool keyReleased(const OIS::KeyEvent& ke);

	bool mShutDown; // When true, trigger the app shutdown

	class Sim* mSim;
	class Scene* mScene;

	Ogre::Root* mRoot;
	Ogre::RenderWindow* mWindow;
	Ogre::SceneManager* mSceneMgr;
	Ogre::Camera* mCamera; bool followCam;
	Ogre::SceneNode* mCameraNode;

	OIS::InputManager* mInputMgr;
	OIS::Keyboard* mKeyboard;
};
