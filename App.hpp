#ifndef APP_HPP
#define APP_HPP

#include <OgreRoot.h>
#include <OgreRenderWindow.h>
#include <OgreSceneManager.h>
#include <OgreCamera.h>
#include <OgreWindowEventUtilities.h>

#include <Terrain/OgreTerrain.h>
#include <Terrain/OgreTerrainGroup.h>

#include <OISInputManager.h>
#include <OISKeyboard.h>

class App : public Ogre::FrameListener,
			public Ogre::WindowEventListener,
			public OIS::KeyListener {
public:
	App();
	~App();

	void run();

private:
	bool setupPlugins();
	void setupResources();
	bool setupRenderSystem();
	void setupScene();
	void setupCamera();
	void setupViewport();
	void setupInput();

	void setupTerrain();

	virtual bool frameRenderingQueued(const Ogre::FrameEvent& evt);

	virtual void windowResized(Ogre::RenderWindow* rw);
	virtual void windowClosed(Ogre::RenderWindow* rw);

	virtual bool keyPressed(const OIS::KeyEvent& ke);
	virtual bool keyReleased(const OIS::KeyEvent& ke);

	Ogre::Root* mRoot;
	Ogre::RenderWindow* mWindow;
	Ogre::SceneManager* mSceneMgr;
	Ogre::Camera* mCamera;

	Ogre::TerrainGlobalOptions* mTerrainGlobals;
	Ogre::TerrainGroup* mTerrainGroup;

	OIS::InputManager* mInputMgr;
	OIS::Keyboard* mKeyboard;
};

#endif
